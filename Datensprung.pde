extern "C" {
#include "datensprung/decoder.c"
};

static struct {
	uint16_t min;
	uint16_t max;
} datensprung_values = {~0, 0};

static int8_t datensprung_tristate(uint16_t value) {
	uint16_t center = (datensprung_values.min+datensprung_values.max)/2;
	if (value > center) {
		return (datensprung_values.max - value < value - center) ? 1 : 0;
	} else {
		return (value - datensprung_values.min < center - value) ? -1 : 0;
	}
}

void datensprung_feed(uint16_t value, uint32_t timestamp) {
	static int8_t last_rstate = 0;
	static uint32_t last_change = 0;
	/* calibrate */
	if (value < datensprung_values.min) datensprung_values.min = value;
	if (value > datensprung_values.max) datensprung_values.max = value;

	int8_t state = datensprung_tristate(value);

	if (state != last_rstate) {
		decoder_feed(state);
		last_change = timestamp;
	} else {
		/* 90 ms without a state change indicates a reset condition */
		if (timestamp-last_change > 90*1000) {
			decoder_reset();
		}
	}

	last_rstate = state;
}

void datensprung_process() {
	struct ds_frame_t frame;
	while (decoder_get_frame(&frame)) {
		/* evaluate the received frames */
	}
}
