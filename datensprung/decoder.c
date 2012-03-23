#include <stdint.h>
#include <string.h>
#include "ds_frame.h"
#include "decoder.h"

/* how many frames do we want buffer? */
#define BUFFER_FRAMES 4
#define FRAME_BITS (sizeof(struct ds_frame_t)*8)

/* ring buffer and positions for reader and writer */
static struct ds_frame_t buffer[BUFFER_FRAMES];
static uint8_t w_pos = 0;
static uint8_t r_pos = 0;
/* bit position inside the current frame */
static uint8_t buffer_pos = 0;

static int8_t last_state = 0;

static void shift_bit(int8_t val) {
	uint8_t *bbuf = (uint8_t*)&buffer[w_pos];
	uint8_t *b = &bbuf[buffer_pos/8];
	if (val) {
		*b |= 1<<(buffer_pos%8);
	} else {
		*b &= ~(1<<(buffer_pos%8));
	}
	buffer_pos = (buffer_pos+1)%FRAME_BITS;
	/* did we just read the last bit of a frame? */
	if (buffer_pos == FRAME_BITS-1) {
		/* advance write buffer position */
		w_pos = (w_pos+1)%BUFFER_FRAMES;
	}
}

void decoder_reset(void) {
	buffer_pos = 0;
	last_state = 0;
}

void decoder_feed(int8_t state) {
	/* if we are changing the state, memorize the last bit */
	if (state != last_state) {
		if (last_state == 1) {
			shift_bit(1);
		} else if (last_state == -1) {
			shift_bit(0);
		}
	}
	last_state = state;
}

uint8_t decoder_complete(void) {
	/* do we have at least one complete frame? */
	return (BUFFER_FRAMES+w_pos-r_pos)%BUFFER_FRAMES;
}

uint8_t decoder_get_frame(struct ds_frame_t *t) {
	if (decoder_complete()) {
		memcpy(t, &buffer[r_pos], sizeof(*t));
		r_pos = (r_pos+1)%BUFFER_FRAMES;
		return 1;
	} else {
		return 0;
	}
}
