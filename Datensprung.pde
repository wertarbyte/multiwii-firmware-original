extern "C" {
#include "datensprung/decoder.c"
};

/* this code is used to store persistent flight assistance data,
 * e.g. whether the ACC has been enabled by Datensprung frames
 */
#define DATENSPRUNG_FA_ACC 0
#define DATENSPRUNG_FA_BARO 1
static struct {
	/* only settings masked with 1 are touched by Datensprung */
	uint8_t mask;
	uint8_t values;
} datensprung_fa_settings = {0,0};

static void datensprung_apply_setting(uint8_t bit, uint8_t *var) {
	// Do we touch the setting?
	if ( ~datensprung_fa_settings.mask & 1<<bit) return;

	if (datensprung_fa_settings.values & 1<<bit) {
		*var = 1;
	} else {
		*var = 0;
	}
}

void datensprung_apply_fa_settings() {
	datensprung_apply_setting(DATENSPRUNG_FA_ACC, &rcOptions[BOXACC]);
	datensprung_apply_setting(DATENSPRUNG_FA_BARO, &rcOptions[BOXBARO]);
}

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
		if (timestamp-last_change > 90L*1000) {
			decoder_reset();
		}
	}

	last_rstate = state;
}

void datensprung_reset() {
	decoder_reset();
	datensprung_values.min = ~0;
	datensprung_values.max = 0;
}

void datensprung_process() {
	struct ds_frame_t frame;
	while (decoder_get_frame(&frame)) {
		if (! decoder_verify_frame(&frame)) continue;

		/* evaluate the received frames */

		switch (frame.cmd) {
			case 0xFA:
				/* flight assistance data has 4 payload bytes:
				 * mask data (which systems do we wish to control?)
				 * mask data mask [sic] (which mask bits do we want to change?)
				 * value data (which systems do we want enabled?)
				 * value data mask (which states do we want to switch?)
				 */
				datensprung_fa_settings.mask = (datensprung_fa_settings.mask & ~frame.data[1]) | (frame.data[0] & frame.data[1]);
				datensprung_fa_settings.values = (datensprung_fa_settings.values & ~frame.data[3]) | (frame.data[2] & frame.data[3]);
				break;
			default:
				break;
		}
	}
}
