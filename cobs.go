package main

const (
	/* DELIMETER marks end of frame. */
	DELIMITER byte = 0x02

	NO_DELIMITER byte = 0xFF

	COBS_CODE_OFFSET byte = DELIMITER

	MAX_BLOCK_SIZE = 84

	XOR byte = 0x03
)

/**
 *  The encoding and framing is broken into 3 steps:
 *   1. Byte values 0x00, 0x01, and 0x02 are escaped using COBS.
 *   2. All bytes are XORed with 0x03 to prevent problematic control characters in the output.
 *   3. A delimeter is appended to the message.
*/
func Encode(data []byte) []byte {
	buffer := make([]byte, 0, len(data)+len(data)/MAX_BLOCK_SIZE+1)
	var code_index, block int

	beginBlock := func() {
		code_index = len(buffer)
		buffer = append(buffer, NO_DELIMITER)
		block = 1
	}

	beginBlock()

	for _, b := range data {
		if b > DELIMITER {
			buffer = append(buffer, b)
			block++
		}

		if b <= DELIMITER || b > MAX_BLOCK_SIZE {
			if b <= DELIMITER {
				delimeter_base := b * MAX_BLOCK_SIZE
				block_offset := byte(block) * COBS_CODE_OFFSET
				buffer[code_index] = delimeter_base + block_offset
			}
			beginBlock()
		}
	}
	buffer[code_index] = byte(block) + COBS_CODE_OFFSET

	return buffer
}

func Decode(data []byte) []byte {
	buffer := make([]byte, 0, len(data)+len(data)/MAX_BLOCK_SIZE+1)

	unescape := func(code byte) (*byte, int) {
		if code == 0xFF {
			return nil, MAX_BLOCK_SIZE + 1
		}
		value := (code - COBS_CODE_OFFSET) / MAX_BLOCK_SIZE
		block := int((code - COBS_CODE_OFFSET) % MAX_BLOCK_SIZE)

		if block == 0 {
			block = MAX_BLOCK_SIZE
			value--
		}

		return &value, block
	}

	value, block := unescape(data[0])
	for _, b := range data[1:] {
		block--
		if block > 0 {
			buffer = append(buffer, b)
			continue
		}

		if value != nil {
			buffer = append(buffer, *value)
		}
		value, block = unescape(b)
	}

	return buffer
}

func Pack(data []byte) []byte {
	buffer := Encode(data)

	for i := range buffer {
		buffer[i] ^= XOR
	}

	buffer = append(buffer, DELIMITER)

	return buffer
}

func Unpack(frame []byte) []byte {
	start := 0

	if frame[0] == 0x01 {
		start++
	}

	unframed := make([]byte, len(frame)-start-1)
	for i, b := range frame[start : len(frame)-1] {
		unframed[i] = b ^ XOR
	}

	return Decode(unframed)
}