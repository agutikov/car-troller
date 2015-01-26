
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hy_stm32_100p/ring_buffer.h"


char buf[16];

int main (int argc, const char* argv[])
{
	ring_buffer_t ring;
	ring_buffer_init(&ring, buf, sizeof(buf));

	while (1) {
		printf("av: %d/%d\nhead=%ld, tail=%ld\n\n> ",
			ring_buffer_av_data(&ring), ring_buffer_av_space(&ring),
			ring.head - ring.buffer, ring.tail - ring.buffer
		);

		size_t sz = 128;
		char* str = malloc(sz);
		int bytes = getline(&str, &sz, stdin);

		bytes--; // remove '\n'

		if (bytes > 0) {

			uint32_t pushed = ring_buffer_push(&ring, str, bytes);
			printf("push(%d) %d\n", bytes, pushed);

		} else if (bytes == 0) {

			uint32_t to_pop = 10;
			uint32_t popped = ring_buffer_pop(&ring, str, to_pop);
			printf("pop(%d) %d\n", to_pop, popped);
			if (popped > 0) {
				for (int i = 0; i < popped; i++) {
					printf("%c", str[i]);
				}
				printf("\n");
			}

		} else if (bytes < 0) {
			return -1;
		}
	}

	return 0;
}


