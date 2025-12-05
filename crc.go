package main

func CRC(data []byte, runningCRC uint32, num int) uint32 {
	if num == 4 {
		// Pad data to a multiple of 4 bytes
		remainder := len(data) % 4
		if remainder > 0 {
			padding := make([]byte, 4-remainder)
			data = append(data, padding...)
		}

		crcVal := 0xFFFFFFFF ^ runningCRC
		for _, b := range data {
			crcVal ^= uint32(b)
			for i := 0; i < 8; i++ {
				if crcVal&1 == 1 {
					crcVal = (crcVal >> 1) ^ 0xEDB88320
				} else {
					crcVal >>= 1
				}
			}
		}
		return 0xFFFFFFFF ^ crcVal
	}
	
	crcVal := runningCRC
	for _, b := range data {
		crcVal = (crcVal + uint32(b))
	}
	return crcVal
}
