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
	ARDUINO_PORT                     = "COM3"
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
	SETPOINT_RANGE_LOW  = 8000.0
	SETPOINT_RANGE_HIGH = 9500.0
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

func mustParseUUID(s string) [16]byte {
    var uuid [16]byte
    s = strings.ReplaceAll(s, "-", "")

    for i := 0; i < 16; i++ {
        fmt.Sscanf(s[i*2:i*2+2], "%02x", &uuid[i])
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

			// Read response
			line, err := reader.ReadString('\n')
			if err != nil {
				log.Printf("Error reading from serial: %v", err)
				time.Sleep(2 * time.Second)
				continue
			}

			resistance := strings.TrimSpace(line)
			if resistance != "" {
				queue.Put(resistance)
			}

			time.Sleep(10 * time.Millisecond)
		}
	}
}

// BLEController manages BLE communication
type BLEController struct {
	adapter     *bluetooth.Adapter
	device      bluetooth.Device
	rxChar      bluetooth.DeviceCharacteristic
	txChar      bluetooth.DeviceCharacteristic
	pendingResp chan []byte
	maxChunkSize int
}

func NewBLEController() (*BLEController, error) {
	adapter := bluetooth.DefaultAdapter
	err := adapter.Enable()
	if err != nil {
		return nil, fmt.Errorf("failed to enable BLE adapter: %w", err)
	}

	return &BLEController{
		adapter:     adapter,
		pendingResp: make(chan []byte, 10),
	}, nil
}

func (b *BLEController) ScanAndConnect(ctx context.Context) error {
	log.Println("Scanning for SPIKE Prime hub...")

	var foundDevice *bluetooth.ScanResult

	err := b.adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
    	// Check if this device advertises our service
    	if result.AdvertisementPayload.HasServiceUUID(bluetooth.NewUUID(mustParseUUID(SERVICE))) {
        	foundDevice = &result
        	adapter.StopScan()
        	return
    	}
	})

	if err != nil {
		return fmt.Errorf("scan error: %w", err)
	}

	time.Sleep(SCAN_TIMEOUT)
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
	services, err := device.DiscoverServices(nil)
	if err != nil {
		return fmt.Errorf("failed to discover services: %w", err)
	}

	for _, service := range services {
		if strings.ToLower(service.UUID().String()) == SERVICE {
			chars, err := service.DiscoverCharacteristics(nil)
			if err != nil {
				return fmt.Errorf("failed to discover characteristics: %w", err)
			}

			for _, char := range chars {
				uuid := strings.ToLower(char.UUID().String())
				if uuid == RX_CHAR {
					b.rxChar = char
				} else if uuid == TX_CHAR {
					b.txChar = char
				}
			}
		}
	}

	if b.rxChar.UUID() == (bluetooth.UUID{}) || b.txChar.UUID() == (bluetooth.UUID{}) {
		return fmt.Errorf("required characteristics not found")
	}

	// Enable notifications
	err = b.txChar.EnableNotifications(func(data []byte) {
		b.onData(data)
	})
	if err != nil {
		return fmt.Errorf("failed to enable notifications: %w", err)
	}

	return nil
}

func (b *BLEController) onData(data []byte) {
	if len(data) == 0 || data[len(data)-1] != 0x02 {
		log.Printf("Received incomplete message")
		return
	}

	unpacked := Unpack(data)
	log.Printf("Received data: %v", unpacked)

	// Send to pending response channel
	select {
	case b.pendingResp <- unpacked:
	default:
		log.Println("Response channel full, dropping message")
	}
}

func (b *BLEController) SendRequest(message []byte) ([]byte, error) {
	frame := Pack(message)

	_, err := b.rxChar.WriteWithoutResponse(frame)
	if err != nil {
		return nil, fmt.Errorf("write failed: %w", err)
	}

	// Wait for response with timeout
	select {
	case resp := <-b.pendingResp:
		return resp, nil
	case <-time.After(5 * time.Second):
		return nil, fmt.Errorf("response timeout")
	}
}

func (b *BLEController) UploadAndRun(programCode []byte, slot int) error {
	programCRC := CRC(programCode, 0, 4)

	log.Printf("Uploading program (CRC: 0x%08x)", programCRC)

	// For simplicity, this is a placeholder
	// You would need to implement the full message protocol
	// from messages.py (StartFileUploadRequest, TransferChunkRequest, etc.)

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
	}()

	// Open serial port
	config := &serial.Config{
		Name: ARDUINO_PORT,
		Baud: ARDUINO_BAUDRATE,
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

	// Initialize PID controller
	pid := NewPIDController(KP, KI, KD, SETPOINT_RANGE_LOW, SETPOINT_RANGE_HIGH)
	motorAdjusting := false

	// Main control loop
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if motorAdjusting {
				continue
			}

			line, ok := serialQueue.Get()
			if !ok {
				continue
			}

			currentValue, err := strconv.ParseFloat(line, 64)
			if err != nil {
				continue
			}

			log.Printf("Flex sensor: %.2f", currentValue)

			output := pid.Update(currentValue)
			output = clamp(output, -15, 15)

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

				err := bleController.UploadAndRun([]byte(programCode), PROGRAM_SLOT)
				if err != nil {
					log.Printf("Failed to upload program: %v", err)
				}

				serialQueue.Clear()
				motorAdjusting = false
			}
		}
	}
}