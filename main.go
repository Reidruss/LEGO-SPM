package main

import (
  "bufio"
	"context"
	"fmt"
	"log"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/tarm/serial"
	"tinygo.org/x/bluetooth"
)

const (
	SCAN_TIMEOUT                     = 25 * time.Second
	ARDUINO_PORT                     = "/dev/ttyACM0"
	ARDUINO_BAUDRATE                 = 9600
	DEVICE_NOTIFICATION_INTERVAL_MS  = 5000
	PROGRAM_SLOT                     = 2

	// BLE UUIDs for SPIKE Prime
	SERVICE = "0000fd02-0000-1000-8000-00805f9b34fb"
	RX_CHAR = "0000fd02-0001-1000-8000-00805f9b34fb"
	TX_CHAR = "0000fd02-0002-1000-8000-00805f9b34fb"
)

// PID Controller parameters
const (
	KP                  = 0.5
	KI                  = 0.1
	KD                  = 0.05
	SETPOINT_RANGE_LOW  = 4000
	SETPOINT_RANGE_HIGH = 6000
	MOTOR_SPEED         = 30
)

// PIDController implements a PID controller
type PIDController struct {
	kp             float64
	ki             float64
	kd             float64
	setpointLow    float64
	setpointHigh   float64
	integral       float64
	previousError  float64
}

func NewPIDController(kp, ki, kd, low, high float64) *PIDController {
	return &PIDController{
		kp:           kp,
		ki:           ki,
		kd:           kd,
		setpointLow:  low,
		setpointHigh: high,
		integral:     0,
		previousError: 0,
	}
}

func (p *PIDController) Update(currentValue float64) float64 {
	var err float64

	// Case 1: Within acceptable range → do nothing
	if currentValue >= p.setpointLow && currentValue <= p.setpointHigh {
		p.integral = 0
        p.previousError = 0
		return 0
	}

	// Case 2: Below range (too low) → want positive output
	if currentValue < p.setpointLow {
		err = currentValue - p.setpointLow // will be negative → output positive
	}

	// Case 3: Above range (too high) → want negative output
	if currentValue > p.setpointHigh {
		err = currentValue - p.setpointHigh // will be positive → output negative
	}

	// PID math
	p.integral += err
	derivative := err - p.previousError
	p.previousError = err

	output := (p.kp * err) + (p.ki * p.integral) + (p.kd * derivative)

	return output
}


// SerialQueue manages serial data in a thread-safe way
type SerialQueue struct {
	mu   sync.Mutex
	data []string
}

func NewSerialQueue() *SerialQueue {
	return &SerialQueue{
		data: make([]string, 0),
	}
}

func (q *SerialQueue) Put(item string) {
	q.mu.Lock()
	defer q.mu.Unlock()
	q.data = append(q.data, item)
}

func (q *SerialQueue) Get() (string, bool) {
	q.mu.Lock()
	defer q.mu.Unlock()
	if len(q.data) == 0 {
		return "", false
	}
	item := q.data[0]
	q.data = q.data[1:]
	return item, true
}

func (q *SerialQueue) Clear() {
	q.mu.Lock()
	defer q.mu.Unlock()
	q.data = q.data[:0]
}

func mustParseUUID(s string) bluetooth.UUID {
	uuid, err := bluetooth.ParseUUID(s)
	if err != nil {
		panic(err)
	}
	return uuid
}

// SerialReader reads from Arduino continuously
func SerialReader(port *serial.Port, queue *SerialQueue, ctx context.Context) {
	reader := bufio.NewReader(port)

	for {
		select {
		case <-ctx.Done():
			return
		default:
			// Request data
			_, err := port.Write([]byte{'g'})
			if err != nil {
				log.Printf("Error writing to serial: %v", err)
				time.Sleep(2 * time.Second)
				continue
			}

			// Read response with timeout
			line, err := reader.ReadString('\n')
			if err != nil {
				// Check if it's a timeout or actual error
				if err.Error() == "EOF" || strings.Contains(err.Error(), "timeout") {
					// Timeout or EOF - just try again
					time.Sleep(100 * time.Millisecond)
					continue
				}
				log.Printf("Error reading from serial: %v", err)
				time.Sleep(2 * time.Second)
				continue
			}

			resistance := strings.TrimSpace(line)
			if resistance != "" {
				queue.Put(resistance)
			}
			time.Sleep(50 * time.Millisecond)
		}
	}
}

// BLEController manages BLE communication
type BLEController struct {
	adapter          *bluetooth.Adapter
	device           bluetooth.Device
	rxChar           bluetooth.DeviceCharacteristic
	txChar           bluetooth.DeviceCharacteristic
	pendingResponses map[byte]chan []byte
	pendingMu        sync.Mutex
	maxChunkSize     int
}

func NewBLEController() (*BLEController, error) {
	adapter := bluetooth.DefaultAdapter
	err := adapter.Enable()
	if err != nil {
		return nil, fmt.Errorf("failed to enable BLE adapter: %w", err)
	}

	return &BLEController{
		adapter:          adapter,
		pendingResponses: make(map[byte]chan []byte),
	}, nil
}

func (b *BLEController) ScanAndConnect(ctx context.Context) error {
	log.Println("Scanning for SPIKE Prime hub...")

	var foundDevice *bluetooth.ScanResult

	err := b.adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		// Check if this device advertises our service
		if result.AdvertisementPayload.HasServiceUUID(mustParseUUID(SERVICE)) {
			foundDevice = &result
			adapter.StopScan()
			return
		}
	})

	if err != nil {
		return fmt.Errorf("scan error: %w", err)
	}

	time.Sleep(5 * time.Second)
	b.adapter.StopScan()

	if foundDevice == nil {
		return fmt.Errorf("no SPIKE Prime hub found")
	}

	log.Printf("Hub detected! Connecting to %s", foundDevice.Address.String())

	device, err := b.adapter.Connect(foundDevice.Address, bluetooth.ConnectionParams{})
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}

	b.device = device
	log.Println("Connected!")

	// Discover services
	log.Println("Discovering services...")
	services, err := device.DiscoverServices(nil)
	if err != nil {
		return fmt.Errorf("failed to discover services: %w", err)
	}
	log.Printf("Found %d services", len(services))

	for _, service := range services {
		if service.UUID().String() == mustParseUUID(SERVICE).String() {
			log.Println("Found SPIKE Prime service, discovering characteristics...")
			chars, err := service.DiscoverCharacteristics(nil)
			if err != nil {
				return fmt.Errorf("failed to discover characteristics: %w", err)
			}
			log.Printf("Found %d characteristics", len(chars))

			for _, char := range chars {
				if char.UUID().String() == mustParseUUID(RX_CHAR).String() {
					b.rxChar = char
				} else if char.UUID().String() == mustParseUUID(TX_CHAR).String() {
					b.txChar = char
				}
			}
		}
	}

	if b.rxChar.UUID() == (bluetooth.UUID{}) || b.txChar.UUID() == (bluetooth.UUID{}) {
		return fmt.Errorf("required characteristics not found")
	}

	// Enable notifications
	log.Println("Enabling notifications on TX characteristic...")
	err = b.txChar.EnableNotifications(func(data []byte) {
		b.onData(data)
	})
	if err != nil {
		return fmt.Errorf("failed to enable notifications: %w", err)
	}
	log.Println("Notifications enabled successfully!")

	return nil
}

func (b *BLEController) onData(data []byte) {
	if len(data) == 0 {
		log.Printf("onData: empty notification")
		return
	}

	//log.Printf("onData: raw notification (%d bytes): % x", len(data), data)

	if data[len(data)-1] != 0x02 {
		xorred := make([]byte, len(data))
		for i := range data {
			xorred[i] = data[i] ^ XOR
		}
		log.Printf("onData: received incomplete frame (post-xor): % x", xorred)
		return
	}

	unpacked := Unpack(data)
	//log.Printf("onData: unpacked payload (%d bytes): % x", len(unpacked), unpacked)

	if len(unpacked) == 0 {
		log.Println("onData: unpacked to empty payload")
		return
	}

	msgID := unpacked[0]
	b.pendingMu.Lock()
	ch, ok := b.pendingResponses[msgID]
	b.pendingMu.Unlock()

	if ok {
		select {
		case ch <- unpacked:
		default:
			log.Printf("Response channel for msgID 0x%02X is full, dropping message", msgID)
		}
	} else {
		// log.Printf("Received unsolicited message with ID 0x%02X", msgID)
	}
}

func (b *BLEController) SendRequestWithResponse(message BaseMessage, responseID byte) ([]byte, error) {
	respChan := make(chan []byte, 1)
	b.pendingMu.Lock()
	b.pendingResponses[responseID] = respChan
	b.pendingMu.Unlock()

	defer func() {
		b.pendingMu.Lock()
		delete(b.pendingResponses, responseID)
		b.pendingMu.Unlock()
		close(respChan)
	}()

	frame := Pack(message.Serialize())
	//log.Printf("SendRequest: sending frame (%d bytes): % x", len(frame), frame)

	_, err := b.rxChar.WriteWithoutResponse(frame)
	if err != nil {
		return nil, fmt.Errorf("write failed: %w", err)
	}

	select {
	case resp := <-respChan:
		return resp, nil
	case <-time.After(5 * time.Second):
		return nil, fmt.Errorf("response timeout for msgID 0x%02X", responseID)
	}
}

func (b *BLEController) UploadAndRunWait(ctx context.Context, programCode []byte, slot int) error {
	programCRC := CRC(programCode, 0, 4)

	log.Printf("Uploading program (CRC: 0x%08x)", programCRC)

		startUploadReq := StartFileUploadRequest{
		FileName: "program.py",
		Slot:     byte(slot),
		CRC:      programCRC,
	}
	respBytes, err := b.SendRequestWithResponse(startUploadReq, StartFileUploadResponse{}.GetID())
	if err != nil {
		return fmt.Errorf("start upload request failed: %w", err)
	}
	startResp, err := DeserializeStartFileUploadResponse(respBytes)
	if err != nil {
		return fmt.Errorf("failed to deserialize start upload response: %w", err)
	}
	if !startResp.Success {
		return fmt.Errorf("start upload failed (hub response)")
	}

	runningCRC := uint32(0)
	for i := 0; i < len(programCode); i += b.maxChunkSize {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
		}

		end := i + b.maxChunkSize
		if end > len(programCode) {
			end = len(programCode)
		}
		chunk := programCode[i:end]
		runningCRC = CRC(chunk, runningCRC, 4)

		chunkReq := TransferChunkRequest{

			RunningCRC: runningCRC,
			Payload:    chunk,
		}
		respBytes, err := b.SendRequestWithResponse(chunkReq, TransferChunkResponse{}.GetID())
		if err != nil {
			return fmt.Errorf("chunk transfer request failed: %w", err)
		}
		chunkResp, err := DeserializeTransferChunkResponse(respBytes)
		if err != nil {
			return fmt.Errorf("failed to deserialize chunk response: %w", err)
		}
		if !chunkResp.Success {
			return fmt.Errorf("chunk transfer failed (hub response)")
		}
	}

	// Step 3: Start the program
	programFlowReq := ProgramFlowRequest{
		Stop: false,
		Slot: byte(slot),
	}
	respBytes, err = b.SendRequestWithResponse(programFlowReq, ProgramFlowResponse{}.GetID())
	if err != nil {
		return fmt.Errorf("program flow request failed: %w", err)
	}
	flowResp, err := DeserializeProgramFlowResponse(respBytes)
	if err != nil {
		return fmt.Errorf("failed to deserialize program flow response: %w", err)
	}
	if !flowResp.Success {
		return fmt.Errorf("failed to start program (hub response)")
	}


	programFlowNotif := ProgramFlowNotification {
		Stop: false,
	}
	respBytesNotif, err := b.SendRequestWithResponse(programFlowNotif, ProgramFlowNotification{}.GetID())
	if err != nil {
		return fmt.Errorf("program flow request failed: %w", err)
	}
	flowRespNotif, err := DeserializeProgramFlowNotification(respBytesNotif)
	if err != nil {
		return fmt.Errorf("failed to deserialize program flow response: %w", err)
	}

	for flowRespNotif.Stop {
		respBytesNotif, err = b.SendRequestWithResponse(programFlowNotif, ProgramFlowNotification{}.GetID())
		if err != nil {
			return fmt.Errorf("program flow request failed: %w", err)
		}

		flowRespNotif, err = DeserializeProgramFlowNotification(respBytesNotif)
		if err != nil {
			return fmt.Errorf("failed to deserialize program flow response: %w", err)
		}
	}

	log.Println("Program started and ended successfully.")
	return nil
}

func (b *BLEController) UploadAndRun(ctx context.Context, programCode []byte, slot int) error {
	programCRC := CRC(programCode, 0, 4)
	log.Printf("Uploading program (CRC: 0x%08x)", programCRC)

	// Step 1: Start file upload
	startUploadReq := StartFileUploadRequest{
		FileName: "program.py",
		Slot:     byte(slot),
		CRC:      programCRC,
	}
	respBytes, err := b.SendRequestWithResponse(startUploadReq, StartFileUploadResponse{}.GetID())
	if err != nil {
		return fmt.Errorf("start upload request failed: %w", err)
	}
	startResp, err := DeserializeStartFileUploadResponse(respBytes)
	if err != nil {
		return fmt.Errorf("failed to deserialize start upload response: %w", err)
	}
	if !startResp.Success {
		return fmt.Errorf("start upload failed (hub response)")
	}

	// Step 2: Send chunks
	runningCRC := uint32(0)
	for i := 0; i < len(programCode); i += b.maxChunkSize {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
		}

		end := i + b.maxChunkSize
		if end > len(programCode) {
			end = len(programCode)
		}
		chunk := programCode[i:end]
		runningCRC = CRC(chunk, runningCRC, 4)

		chunkReq := TransferChunkRequest{

			RunningCRC: runningCRC,
			Payload:    chunk,
		}
		respBytes, err := b.SendRequestWithResponse(chunkReq, TransferChunkResponse{}.GetID())
		if err != nil {
			return fmt.Errorf("chunk transfer request failed: %w", err)
		}
		chunkResp, err := DeserializeTransferChunkResponse(respBytes)
		if err != nil {
			return fmt.Errorf("failed to deserialize chunk response: %w", err)
		}
		if !chunkResp.Success {
			return fmt.Errorf("chunk transfer failed (hub response)")
		}
	}

	// Step 3: Start the program
	programFlowReq := ProgramFlowRequest{
		Stop: false,
		Slot: byte(slot),
	}
	respBytes, err = b.SendRequestWithResponse(programFlowReq, ProgramFlowResponse{}.GetID())
	if err != nil {
		return fmt.Errorf("program flow request failed: %w", err)
	}
	flowResp, err := DeserializeProgramFlowResponse(respBytes)
	if err != nil {
		return fmt.Errorf("failed to deserialize program flow response: %w", err)
	}
	if !flowResp.Success {
		return fmt.Errorf("failed to start program (hub response)")
	}

	log.Println("Program started successfully.")
	return nil
}

func clamp(value, min, max float64) float64 {

	if value == 0 {
		return value;
	}

	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

func main() {
	fmt.Printf("This example will override the program in slot %d. Continue? [Y/n] ", PROGRAM_SLOT)

	reader := bufio.NewReader(os.Stdin)
	answer, _ := reader.ReadString('\n')

	if strings.HasPrefix(strings.ToLower(strings.TrimSpace(answer)), "n") {
		fmt.Println("Aborted by user.")
		return
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Handle interrupt signal
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sigChan
		log.Println("Interrupted by user")
		cancel()
		<-sigChan
		log.Println("Force exiting...")
		os.Exit(0)
	}()

	// Open serial port
	config := &serial.Config{
		Name:        ARDUINO_PORT,
		Baud:        ARDUINO_BAUDRATE,
		ReadTimeout: 1 * time.Second,
	}
	serialPort, err := serial.OpenPort(config)
	if err != nil {
		log.Fatalf("Failed to open serial port: %v", err)
	}

    // --- NEW: FLUSH BUFFER ---
    // Flush the OS buffer to remove stale data from the previous run
    // This fixes the issue where it reads old values on restart
	if err := serialPort.Flush(); err != nil {
		log.Printf("Warning: Failed to flush serial buffer: %v", err)
	}
    // -------------------------

	defer serialPort.Close()

	// Start serial reader
	serialQueue := NewSerialQueue()
	go SerialReader(serialPort, serialQueue, ctx)

	// Initialize BLE
	bleController, err := NewBLEController()
	if err != nil {
		log.Fatalf("Failed to initialize BLE: %v", err)
	}

	err = bleController.ScanAndConnect(ctx)
	if err != nil {
		log.Fatalf("Failed to connect to hub: %v", err)
	}
	log.Println("BLE setup complete!")

	// --- Initialize Hub ---
	infoRespBytes, err := bleController.SendRequestWithResponse(InfoRequest{}, InfoResponse{}.GetID())
	if err != nil {
		log.Fatalf("InfoRequest failed: %v", err)
	}
	info, err := DeserializeInfoResponse(infoRespBytes)
	if err != nil {
		log.Fatalf("Failed to decode InfoResponse: %v", err)
	}
	log.Printf("InfoResponse: %v", info)
	bleController.maxChunkSize = int(info.MaxChunkSize)
	log.Printf("Set maxChunkSize from hub: %d", bleController.maxChunkSize)

	if bleController.maxChunkSize == 0 {
		bleController.maxChunkSize = MAX_BLOCK_SIZE
		log.Printf("Warning: maxChunkSize is 0, using fallback: %d", bleController.maxChunkSize)
	}

    // =========================================================================
    // CALIBRATION SEQUENCE
    // =========================================================================
	log.Println("Waiting for initial sensor value from Arduino...")
	var line string
	var ok bool
	for i := 0; i < 20; i++ { // Try for 2 seconds
		line, ok = serialQueue.Get()
		if ok {
			break
		}
		time.Sleep(100 * time.Millisecond)
	}
	if !ok {
		log.Fatalf("Arduino is not producing outputs. Aborting.")
	}

	current_value, err := strconv.ParseFloat(line, 64)
	if err != nil {
		log.Fatalf("Arduino is producing incorrect output: %v", err)
	}

	log.Printf("Current Value: %.2f", current_value)

    // Calibration: If too low, tighten until contact
	if current_value < SETPOINT_RANGE_LOW {
		programCode := fmt.Sprintf(`import motor
from hub import port
import time
motor.run(port.A, -%d)
motor.run(port.C, %d)
while True:
    time.sleep(1)
`, MOTOR_SPEED, MOTOR_SPEED)
		log.Println("Uploading calibration program to start motor...")
		err := bleController.UploadAndRun(ctx, []byte(programCode), PROGRAM_SLOT)
		if err != nil {
			log.Fatalf("Failed to upload program to start motor: %v", err)
		}
		log.Println("Calibration program uploaded, motor should be running.")

		serialQueue.Clear()
		time.Sleep(20 * time.Millisecond)
	}

    // Monitor Calibration loop
	for {
		var latestValue string
		var valueFound bool

		// Drain the queue to get the latest value
		for {
			line, ok := serialQueue.Get()
			if !ok {
				break // Queue is empty
			}
			latestValue = line
			valueFound = true
		}

		if !valueFound {
			time.Sleep(50 * time.Millisecond)
			continue
		}

		current_value, err := strconv.ParseFloat(latestValue, 64)
		if err != nil {
			log.Printf("Could not parse float: %v", err)
			time.Sleep(50 * time.Millisecond)
			continue
		}

		if current_value > SETPOINT_RANGE_LOW {
			programCode := `import motor
from hub import port
motor.stop(port.A, stop_action='hold')
motor.stop(port.C, stop_action='hold')
`
			log.Println("Flex sensor detected contact. Stopping motor.")
			err := bleController.UploadAndRun(ctx, []byte(programCode), PROGRAM_SLOT)
			if err != nil {
				log.Printf("Failed to upload program to stop motor: %v", err)
			}
			log.Println("Motor stopped. Calibration complete.")
			break
		}

		time.Sleep(50 * time.Millisecond)
	}

	time.Sleep(500 * time.Millisecond)

    // =========================================================================
    // MAIN CONTROL LOOP (Replaces PID with Threshold/Bang-Bang Logic)
    // =========================================================================
	log.Println("Starting main control loop...")

	ticker := time.NewTicker(250 * time.Millisecond) // Check 4 times a second
	defer ticker.Stop()

	log.Println("Entering main loop...")
	for {
		select {
		case <-ctx.Done():
			log.Println("Main loop cancelled.")
			serialQueue.Clear()
			return
		case <-ticker.C:
			var latestValue string
			var valueFound bool

			// Drain the queue to get the latest value
			for {
				line, ok := serialQueue.Get()
				if !ok {
					break // Queue is empty
				}
				latestValue = line
				valueFound = true
			}

			if !valueFound {
				continue
			}

			currentValue, err := strconv.ParseFloat(latestValue, 64)
			if err != nil {
				log.Printf("Could not parse float in main loop: %v", err)
				continue
			}

            // Case 1: Value is too LOW. (Needs to tighten)
            // We reuse the exact logic from calibration.
            if currentValue < SETPOINT_RANGE_LOW {
                log.Printf("Value %.0f is LOW. Starting continuous adjustment...", currentValue)

                // 1. Start Motor
                programCode := fmt.Sprintf(`import motor
from hub import port
import time
motor.run(port.A, -%d)
motor.run(port.C, %d)
while True:
    time.sleep(1)
`, MOTOR_SPEED, MOTOR_SPEED)

                err := bleController.UploadAndRun(ctx, []byte(programCode), PROGRAM_SLOT)
                if err != nil {
                    log.Printf("Failed to start motor: %v", err)
                    continue
                }

                // 2. Loop until back in range
                for {
                    // Quick read of queue
                    var reading string
                    var found bool

                    // Drain buffer
                    for {
                        l, ok := serialQueue.Get()
                        if !ok { break }
                        reading = l
                        found = true
                    }

                    if !found {
                        time.Sleep(50 * time.Millisecond)
                        continue
                    }

                    val, _ := strconv.ParseFloat(reading, 64)
                    // Stop if we cross the low threshold back into safety
                    if val >= SETPOINT_RANGE_LOW {
                        break
                    }
                    time.Sleep(50 * time.Millisecond)
                }

                // 3. Stop Motor
                stopCode := `import motor
from hub import port
motor.stop(port.A, stop_action='hold')
motor.stop(port.C, stop_action='hold')
`
                bleController.UploadAndRun(ctx, []byte(stopCode), PROGRAM_SLOT)
                log.Println("Adjustment complete. Stopping.")
            }

            // Case 2: Value is too HIGH. (Needs to loosen)
            // Same logic, reversed direction.
            if currentValue > SETPOINT_RANGE_HIGH {
                log.Printf("Value %.0f is HIGH. Starting continuous adjustment...", currentValue)

                // 1. Start Motor (Reversed)
                programCode := fmt.Sprintf(`import motor
from hub import port
import time
motor.run(port.A, %d)
motor.run(port.C, -%d)
while True:
    time.sleep(1)
`, MOTOR_SPEED, MOTOR_SPEED)

                err := bleController.UploadAndRun(ctx, []byte(programCode), PROGRAM_SLOT)
                if err != nil {
                    log.Printf("Failed to start motor: %v", err)
                    continue
                }

                // 2. Loop until back in range
                for {
                    var reading string
                    var found bool
                    for {
                        l, ok := serialQueue.Get()
                        if !ok { break }
                        reading = l
                        found = true
                    }

                    if !found {
                        time.Sleep(50 * time.Millisecond)
                        continue
                    }

                    val, _ := strconv.ParseFloat(reading, 64)
                    // Stop if we cross the high threshold back into safety
                    if val <= SETPOINT_RANGE_HIGH {
                        break
                    }
                    time.Sleep(50 * time.Millisecond)
                }

                // 3. Stop Motor
                stopCode := `import motor
from hub import port
motor.stop(port.A, stop_action='hold')
motor.stop(port.C, stop_action='hold')
`
                bleController.UploadAndRun(ctx, []byte(stopCode), PROGRAM_SLOT)
                log.Println("Adjustment complete. Stopping.")
            }
		}
	}
}

