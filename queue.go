package main

import (
	"sync"
)

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