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
	SETPOINT_RANGE_LOW  = 100
	SETPOINT_RANGE_HIGH = 200
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

	if currentValue >= p.setpointLow && currentValue <= p.setpointHigh {
		err = 0
		p.integral = 0
	} else if currentValue < p.setpointLow {
		err = p.setpointLow - currentValue
		p.integral += err
	} else {
		err = p.setpointHigh - currentValue
		p.integral += err
	}

	derivative := err - p.previousError
	output := (p.kp * err) + (p.ki * p.integral) + (p.kd * derivative)
	p.previousError = err
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
			time.Sleep(100 * time.Millisecond)
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

	log.Printf("onData: raw notification (%d bytes): % x", len(data), data)

	if data[len(data)-1] != 0x02 {
		xorred := make([]byte, len(data))
		for i := range data {
			xorred[i] = data[i] ^ XOR
		}
		log.Printf("onData: received incomplete frame (post-xor): % x", xorred)
		return
	}

	unpacked := Unpack(data)
	log.Printf("onData: unpacked payload (%d bytes): % x", len(unpacked), unpacked)

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
		log.Printf("Received unsolicited message with ID 0x%02X", msgID)
		// Here you could handle notifications that are not direct responses
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
	log.Printf("SendRequest: sending frame (%d bytes): % x", len(frame), frame)

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

	// Fallback if InfoResponse didn't set it
	if bleController.maxChunkSize == 0 {
		bleController.maxChunkSize = MAX_BLOCK_SIZE // A sensible default
		log.Printf("Warning: maxChunkSize is 0, using fallback: %d", bleController.maxChunkSize)
	}

	// Initialize PID controller
	log.Println("Initializing PID controller...")
	pid := NewPIDController(KP, KI, KD, SETPOINT_RANGE_LOW, SETPOINT_RANGE_HIGH)
	motorAdjusting := false
	log.Println("Starting main control loop...")

	// Main control loop
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	log.Println("Entering main loop...")
	for {
		select {
		case <-ctx.Done():
			log.Println("Main loop cancelled.")
			return
		case <-ticker.C:
			log.Println("Ticker ticked.")
			if motorAdjusting {
				continue
			}

			log.Println("Getting data from serial queue...")
			line, ok := serialQueue.Get()
			if !ok {
				log.Println("Serial queue empty.")
				continue
			}

			currentValue, err := strconv.ParseFloat(line, 64)
			if err != nil {
				continue
			}

			log.Printf("Flex sensor: %.2f", currentValue)

			output := pid.Update(currentValue)
			output = clamp(output, 100, 100)

			if output > 5 || output < -5 {
				motorAdjusting = true
				degrees := int(output)

				programCode := fmt.Sprintf(`import motor
from hub import port
import time

motor.run_for_degrees(port.A, %d, 400)
time.sleep(0.1)
`, degrees)

				log.Printf("Adjusting motor by %d degrees", degrees)

				err := bleController.UploadAndRun(ctx, []byte(programCode), PROGRAM_SLOT)
				if err != nil {
					log.Printf("Failed to upload program: %v", err)
				}
				serialQueue.Clear()
				motorAdjusting = false
			}
		}
	}
}