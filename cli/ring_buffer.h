#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include <stdint.h>
#include <string.h>

typedef struct ring_buffer {
	uint8_t* buffer;
	uint32_t capacity;
	uint8_t* head;
	uint8_t* tail;
	/*
	 * head increment only by readed
	 * tail increment only by writer
	 * if there is only one reader and one writer - syncronization not needed
	 * while reading avaliable data can only increase (by writing)
	 * while writing free space can only inclrease (by reading)
	 *
	 * allowed to write capacity-1 byte
	 * so there will not be state when head==tail
	 * and nobody knows if buffer is empty or full
	 * so if head==tail - then buffer is empty
	*/
} ring_buffer_t;

static inline void ring_buffer_init (ring_buffer_t* ring, void* buffer, uint32_t size)
{
	ring->buffer = (uint8_t*)buffer;
	ring->capacity = size;
	ring->head = ring->buffer;
	ring->tail = ring->head;
}

static inline uint32_t ring_buffer_av_space (ring_buffer_t* ring)
{
	if (ring->head <= ring->tail) {
		return ring->capacity - (ring->tail - ring->head) - 1;
	} else {
		return ring->head - ring->tail - 1;
	}
}

static inline uint32_t ring_buffer_av_data (ring_buffer_t* ring)
{
	if (ring->head <= ring->tail) {
		return ring->tail - ring->head;
	} else {
		return ring->capacity - (ring->head - ring->tail);
	}
}

static inline int ring_buffer_next_byte (ring_buffer_t* ring, uint8_t* byte)
{
	if (ring->head == ring->tail) {
		return 0;
	} else {
		*byte = *ring->head++;
		if (ring->head == ring->buffer + ring->capacity) {
			ring->head = ring->buffer;
		}
		return 1;
	}
}

static inline int ring_buffer_add_byte (ring_buffer_t* ring, uint8_t byte)
{
	if (ring->head <= ring->tail) {
		if (ring->tail < ring->buffer + ring->capacity) {
			*ring->tail++ = byte;
			if (ring->tail == ring->buffer + ring->capacity) {
				ring->tail = ring->buffer;
			}
			return 1;
		} else {
			return 0;
		}
	} else if (ring->head > ring->tail + 1) {
		*ring->tail++ = byte;
		return 1;
	} else {
		return 0;
	}
}

static inline uint32_t ring_buffer_av_data_cont (ring_buffer_t* ring)
{
	if (ring->head <= ring->tail) {
		return ring->tail - ring->head;
	} else {
		return ring->buffer + ring->capacity - ring->head;
	}
}

static inline void ring_buffer_clear (ring_buffer_t* ring)
{
	ring->head = (uint8_t*)ring->buffer;
	ring->tail = ring->head;
}

#if 0

static inline void _ring_buffer_push (ring_buffer_t* ring, void* ptr, uint32_t size)
{
	if (ring->head < ring->tail) {
		uint32_t to_end = ring->buffer + ring->capacity - ring->tail;
		if (to_end <= size) {
			memcpy(ring->tail, ptr, to_end);
			ring->tail = ring->buffer;
			size -= to_end;
			if (!size) {
				return;
			}
			ptr += to_end;
		}
	}
	memcpy(ring->tail, ptr, size);
	ring->tail += size;
}

#endif

/*
 * Already knew that required space avaliable - just write data into buffer.
 */
static inline void _ring_buffer_push (ring_buffer_t* ring, const void* ptr, uint32_t size)
{
	if (ring->head > ring->tail) {
		memcpy(ring->tail, ptr, size);
		ring->tail += size;
	} else {
		uint32_t to_end = ring->buffer + ring->capacity - ring->tail;
		if (to_end > size) {
			memcpy(ring->tail, ptr, size);
			ring->tail += size;
		} else {
			memcpy(ring->tail, ptr, to_end);
			size -= to_end;
			ptr += to_end;
			ring->tail = ring->buffer;
			if (size) {
				memcpy(ring->tail, ptr, size);
				ring->tail += size;
			}
		}
	}
}

static inline uint32_t ring_buffer_push (ring_buffer_t* ring, const void* ptr, uint32_t size)
{
	uint32_t space = ring_buffer_av_space(ring);
	if (space) {
		if (space < size) {
			size = space;
		}
		_ring_buffer_push(ring, ptr, size);
		return size;
	} else {
		return 0;
	}
}

static inline uint32_t ring_buffer_push_strict (ring_buffer_t* ring, const void* ptr, uint32_t size)
{
	uint32_t space = ring_buffer_av_space(ring);
	if (space < size) {
		return 0;
	} else {
		_ring_buffer_push(ring, ptr, size);
		return size;
	}
}

static inline void ring_buffer_push_overwrite (ring_buffer_t* ring, const void* ptr, uint32_t size)
{
	// have to change head, but writer should not change head
}

static inline void _ring_buffer_fflush_head (ring_buffer_t* ring, uint32_t size)
{
	if (ring->tail > ring->head) {
		ring->head += size;
	} else {
		uint32_t to_end = ring->buffer + ring->capacity - ring->head;
		if (to_end > size) {
			ring->head += size;
		} else {
			size -= to_end;
			ring->head = ring->buffer;
			if (size) {
				ring->head += size;
			}
		}
	}
}

static inline uint32_t ring_buffer_fflush_head (ring_buffer_t* ring, uint32_t size)
{
	uint32_t data_length = ring_buffer_av_data(ring);
	if (data_length < size) {
		size = data_length;
	}
	_ring_buffer_fflush_head(ring, size);
	return size;
}


#if 0
static inline void _ring_buffer_pop (ring_buffer_t* ring, void* ptr, uint32_t size)
{
	if (ring->tail < ring->head) {
		uint32_t to_end = ring->buffer + ring->capacity - ring->head;
		if (to_end <= size) {
			memcpy(ptr, ring->head, to_end);
			ring->head = ring->buffer;
			size -= to_end;
			if (!size) {
				return;
			}
			ptr += to_end;
		}
	}
	memcpy(ptr, ring->head, size);
	ring->head += size;
}
#endif
/*
 * Already knew that required amount of data avaliable - just read.
 */
static inline void _ring_buffer_pop (ring_buffer_t* ring, void* ptr, uint32_t size)
{
	if (ring->tail > ring->head) {
		memcpy(ptr, ring->head, size);
		ring->head += size;
	} else {
		uint32_t to_end = ring->buffer + ring->capacity - ring->head;
		if (to_end > size) {
			memcpy(ptr, ring->head, size);
			ring->head += size;
		} else {
			memcpy(ptr, ring->head, to_end);
			size -= to_end;
			ptr += to_end;
			ring->head = ring->buffer;
			if (size) {
				memcpy(ptr, ring->head, size);
				ring->head += size;
			}
		}
	}
}

static inline uint32_t ring_buffer_pop (ring_buffer_t* ring, void* ptr, uint32_t size)
{
	uint32_t data_length = ring_buffer_av_data(ring);
	if (data_length) {
		if (data_length < size) {
			size = data_length;
		}
		_ring_buffer_pop(ring, ptr, size);
		return size;
	} else {
		return 0;
	}
}

static inline int ring_buffer_pop_strict (ring_buffer_t* ring, void* ptr, uint32_t size)
{
	uint32_t data_length = ring_buffer_av_data(ring);
	if (data_length < size) {
		return 0;
	} else {
		_ring_buffer_pop(ring, ptr, size);
		return size;
	}
}


#endif
