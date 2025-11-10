package main

import (
	"hash/crc32"
)

// CRC calculates the CRC32 of data with an optional seed and alignment.
func CRC(data []byte, seed uint32, align int) uint32 {

	dataCopy := make([]byte, len(data))
	copy(dataCopy, data)

	remainder := len(dataCopy) % align
	if remainder != 0 {
		padding := make([]byte, align-remainder)
		dataCopy = append(dataCopy, padding...)
	}

	table := crc32.MakeTable(crc32.IEEE)
	crc := crc32.Update(seed, table, dataCopy)

	return crc
}