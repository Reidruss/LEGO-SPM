package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"strings"
)

// BaseMessage interface for all messages
type BaseMessage interface {
	GetID() byte
	Serialize() []byte
}

// InfoRequest (ID: 0x00)
type InfoRequest struct{}

func (m InfoRequest) GetID() byte     { return 0x00 }
func (m InfoRequest) Serialize() []byte { return []byte{0x00} }

// InfoResponse (ID: 0x01)
type InfoResponse struct {
	RPCMajor           byte
	RPCMinor           byte
	RPCBuild           uint16
	FirmwareMajor      byte
	FirmwareMinor      byte
	FirmwareBuild      uint16
	MaxPacketSize      uint16
	MaxMessageSize     uint16
	MaxChunkSize       uint16
	ProductGroupDevice uint16
}

func (m InfoResponse) GetID() byte { return 0x01 }
func (m InfoResponse) Serialize() []byte { return nil } // Response only

func DeserializeInfoResponse(data []byte) (*InfoResponse, error) {
	if len(data) < 17 {
		return nil, fmt.Errorf("data too short for InfoResponse")
	}

	return &InfoResponse{
		RPCMajor:           data[1],
		RPCMinor:           data[2],
		RPCBuild:           binary.LittleEndian.Uint16(data[3:5]),
		FirmwareMajor:      data[5],
		FirmwareMinor:      data[6],
		FirmwareBuild:      binary.LittleEndian.Uint16(data[7:9]),
		MaxPacketSize:      binary.LittleEndian.Uint16(data[9:11]),
		MaxMessageSize:     binary.LittleEndian.Uint16(data[11:13]),
		MaxChunkSize:       binary.LittleEndian.Uint16(data[13:15]),
		ProductGroupDevice: binary.LittleEndian.Uint16(data[15:17]),
	}, nil
}

func (m InfoResponse) String() string {
	return fmt.Sprintf("InfoResponse(rpc=%d.%d.%d, fw=%d.%d.%d, max_chunk=%d)",
		m.RPCMajor, m.RPCMinor, m.RPCBuild,
		m.FirmwareMajor, m.FirmwareMinor, m.FirmwareBuild,
		m.MaxChunkSize)
}

// ClearSlotRequest (ID: 0x46)
type ClearSlotRequest struct {
	Slot byte
}

func (m ClearSlotRequest) GetID() byte { return 0x46 }
func (m ClearSlotRequest) Serialize() []byte {
	return []byte{m.GetID(), m.Slot}
}

// ClearSlotResponse (ID: 0x47)
type ClearSlotResponse struct {
	Success bool
}

func (m ClearSlotResponse) GetID() byte { return 0x47 }
func (m ClearSlotResponse) Serialize() []byte { return nil }

func DeserializeClearSlotResponse(data []byte) (*ClearSlotResponse, error) {
	if len(data) < 2 {
		return nil, fmt.Errorf("data too short")
	}
	return &ClearSlotResponse{Success: data[1] == 0x00}, nil
}

// StartFileUploadRequest (ID: 0x0C)
type StartFileUploadRequest struct {
	FileName string
	Slot     byte
	CRC      uint32
}

func (m StartFileUploadRequest) GetID() byte { return 0x0C }
func (m StartFileUploadRequest) Serialize() []byte {
	encodedName := []byte(m.FileName)
	if len(encodedName) > 31 {
		panic(fmt.Sprintf("file name too long: %d >= 32", len(encodedName)+1))
	}

	buf := new(bytes.Buffer)
	buf.WriteByte(m.GetID())
	buf.Write(encodedName)
	buf.WriteByte(0) // null terminator
	buf.WriteByte(m.Slot)
	binary.Write(buf, binary.LittleEndian, m.CRC)

	return buf.Bytes()
}

// StartFileUploadResponse (ID: 0x0D)
type StartFileUploadResponse struct {
	Success bool
}

func (m StartFileUploadResponse) GetID() byte { return 0x0D }
func (m StartFileUploadResponse) Serialize() []byte { return nil }

func DeserializeStartFileUploadResponse(data []byte) (*StartFileUploadResponse, error) {
	if len(data) < 2 {
		return nil, fmt.Errorf("data too short")
	}
	return &StartFileUploadResponse{Success: data[1] == 0x00}, nil
}

// TransferChunkRequest (ID: 0x10)
type TransferChunkRequest struct {
	RunningCRC uint32
	Payload    []byte
}

func (m TransferChunkRequest) GetID() byte { return 0x10 }
func (m TransferChunkRequest) Serialize() []byte {
	size := uint16(len(m.Payload))

	buf := new(bytes.Buffer)
	buf.WriteByte(m.GetID())
	binary.Write(buf, binary.LittleEndian, m.RunningCRC)
	binary.Write(buf, binary.LittleEndian, size)
	buf.Write(m.Payload)

	return buf.Bytes()
}

// TransferChunkResponse (ID: 0x11)
type TransferChunkResponse struct {
	Success bool
}

func (m TransferChunkResponse) GetID() byte { return 0x11 }
func (m TransferChunkResponse) Serialize() []byte { return nil }

func DeserializeTransferChunkResponse(data []byte) (*TransferChunkResponse, error) {
	if len(data) < 2 {
		return nil, fmt.Errorf("data too short")
	}
	return &TransferChunkResponse{Success: data[1] == 0x00}, nil
}

// ProgramFlowRequest (ID: 0x1E)
type ProgramFlowRequest struct {
	Stop bool
	Slot byte
}

func (m ProgramFlowRequest) GetID() byte { return 0x1E }
func (m ProgramFlowRequest) Serialize() []byte {
	stop := byte(0)
	if m.Stop {
		stop = 1
	}
	return []byte{m.GetID(), stop, m.Slot}
}

// ProgramFlowResponse (ID: 0x1F)
type ProgramFlowResponse struct {
	Success bool
}

func (m ProgramFlowResponse) GetID() byte { return 0x1F }
func (m ProgramFlowResponse) Serialize() []byte { return nil }

func DeserializeProgramFlowResponse(data []byte) (*ProgramFlowResponse, error) {
	if len(data) < 2 {
		return nil, fmt.Errorf("data too short")
	}
	return &ProgramFlowResponse{Success: data[1] == 0x00}, nil
}

// ProgramFlowNotification (ID: 0x20)
type ProgramFlowNotification struct {
	Stop bool
}

func (m ProgramFlowNotification) GetID() byte { return 0x20 }
func (m ProgramFlowNotification) Serialize() []byte { return nil }

func DeserializeProgramFlowNotification(data []byte) (*ProgramFlowNotification, error) {
	if len(data) < 2 {
		return nil, fmt.Errorf("data too short")
	}
	return &ProgramFlowNotification{Stop: data[1] != 0}, nil
}

// ConsoleNotification (ID: 0x21)
type ConsoleNotification struct {
	Text string
}

func (m ConsoleNotification) GetID() byte { return 0x21 }
func (m ConsoleNotification) Serialize() []byte { return nil }

func DeserializeConsoleNotification(data []byte) (*ConsoleNotification, error) {
	if len(data) < 1 {
		return nil, fmt.Errorf("data too short")
	}
	textBytes := bytes.TrimRight(data[1:], "\x00")
	return &ConsoleNotification{Text: string(textBytes)}, nil
}

func (m ConsoleNotification) String() string {
	return fmt.Sprintf("ConsoleNotification(%q)", m.Text)
}

// DeviceNotificationRequest (ID: 0x28)
type DeviceNotificationRequest struct {
	IntervalMS uint16
}

func (m DeviceNotificationRequest) GetID() byte { return 0x28 }
func (m DeviceNotificationRequest) Serialize() []byte {
	buf := new(bytes.Buffer)
	buf.WriteByte(m.GetID())
	binary.Write(buf, binary.LittleEndian, m.IntervalMS)
	return buf.Bytes()
}

// DeviceNotificationResponse (ID: 0x29)
type DeviceNotificationResponse struct {
	Success bool
}

func (m DeviceNotificationResponse) GetID() byte { return 0x29 }
func (m DeviceNotificationResponse) Serialize() []byte { return nil }

func DeserializeDeviceNotificationResponse(data []byte) (*DeviceNotificationResponse, error) {
	if len(data) < 2 {
		return nil, fmt.Errorf("data too short")
	}
	return &DeviceNotificationResponse{Success: data[1] == 0x00}, nil
}

// DeviceMessage represents a parsed device message
type DeviceMessage struct {
	Name    string
	Payload interface{}
}

// Device-specific message structs
type BatteryMessage struct {
	ID      byte
	Voltage byte
}

type IMUMessage struct {
	ID     byte
	Port   byte
	Mode   byte
	AccelX int16
	AccelY int16
	AccelZ int16
	GyroX  int16
	GyroY  int16
	GyroZ  int16
	Val7   int16
	Val8   int16
	Val9   int16
	Val10  int16
}

type FiveByFiveMessage struct {
	ID    byte
	Grid [25]byte
}

type MotorMessage struct {
	ID       byte
	Port     byte
	Mode     byte
	Speed    int16
	Position int16
	AbsPos   int8
	Power    int32
}

type ForceMessage struct {
	ID    byte
	Port  byte
	Mode  byte
	Value byte
}

type ColorMessage struct {
	ID    byte
	Port  byte
	Mode  int8
	Red   uint16
	Green uint16
	Blue  uint16
}

type DistanceMessage struct {
	ID       byte
	Port     byte
	Distance int16
}

type ThreeByThreeMessage struct {
	ID    byte
	Port  byte
	Grid [9]byte
}

// DeviceNotification (ID: 0x3C)
type DeviceNotification struct {
	Size     uint16
	Payload  []byte
	Messages []DeviceMessage
}

func (m DeviceNotification) GetID() byte { return 0x3C }
func (m DeviceNotification) Serialize() []byte { return nil }

var deviceMessageDeserializers = map[byte]struct {
	name         string
	size         int
	deserializer func([]byte) (interface{}, error)
}{
	0x00: {"Battery", 2, func(b []byte) (interface{}, error) {
		var msg BatteryMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
	0x01: {"IMU", 23, func(b []byte) (interface{}, error) {
		var msg IMUMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
	0x02: {"5x5", 26, func(b []byte) (interface{}, error) {
		var msg FiveByFiveMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
	0x0A: {"Motor", 12, func(b []byte) (interface{}, error) {
		var msg MotorMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
	0x0B: {"Force", 4, func(b []byte) (interface{}, error) {
		var msg ForceMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
	0x0C: {"Color", 9, func(b []byte) (interface{}, error) {
		var msg ColorMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
	0x0D: {"Distance", 4, func(b []byte) (interface{}, error) {
		var msg DistanceMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
	0x0E: {"3x3", 11, func(b []byte) (interface{}, error) {
		var msg ThreeByThreeMessage
		err := binary.Read(bytes.NewReader(b), binary.LittleEndian, &msg)
		return msg, err
	}},
}

func DeserializeDeviceNotification(data []byte) (*DeviceNotification, error) {
	if len(data) < 3 {
		return nil, fmt.Errorf("data too short")
	}

	size := binary.LittleEndian.Uint16(data[1:3])
	payload := data[3:]

	if len(payload) != int(size) {
		return nil, fmt.Errorf("payload size mismatch: expected %d, got %d", size, len(payload))
	}

	dn := &DeviceNotification{
		Size:     size,
		Payload:  payload,
		Messages: make([]DeviceMessage, 0),
	}

	remaining := payload
	for len(remaining) > 0 {
		msgID := remaining[0]

		if info, ok := deviceMessageDeserializers[msgID]; ok {
			if len(remaining) < info.size {
				break
			}

			msgData := remaining[:info.size]
			parsedMsg, err := info.deserializer(msgData)
			if err != nil {
				fmt.Printf("Error deserializing device message ID 0x%02X: %v\n", msgID, err)
				break
			}

			dn.Messages = append(dn.Messages, DeviceMessage{
				Name:    info.name,
				Payload: parsedMsg,
			})

			remaining = remaining[info.size:]
		} else {
			fmt.Printf("Unknown device message ID: 0x%02X\n", msgID)
			break
		}
	}

	return dn, nil
}

func (m DeviceNotification) String() string {
	names := make([]string, len(m.Messages))
	for i, msg := range m.Messages {
		names[i] = msg.Name
	}
	return fmt.Sprintf("DeviceNotification([%s])", strings.Join(names, ", "))
}

// Deserialize determines message type and deserializes accordingly
func Deserialize(data []byte) (interface{}, error) {
	if len(data) == 0 {
		return nil, fmt.Errorf("empty data")
	}

	messageType := data[0]

	switch messageType {
	case 0x01:
		return DeserializeInfoResponse(data)
	case 0x0D:
		return DeserializeStartFileUploadResponse(data)
	case 0x11:
		return DeserializeTransferChunkResponse(data)
	case 0x1F:
		return DeserializeProgramFlowResponse(data)
	case 0x20:
		return DeserializeProgramFlowNotification(data)
	case 0x21:
		return DeserializeConsoleNotification(data)
	case 0x29:
		return DeserializeDeviceNotificationResponse(data)
	case 0x3C:
		return DeserializeDeviceNotification(data)
	case 0x47:
		return DeserializeClearSlotResponse(data)
	default:
		return nil, fmt.Errorf("unknown message type: 0x%02X", messageType)
	}
}