#if defined(DATENSCHLAG_CHANNEL)

#include <stddef.h>

/* each command has a fixed amount of payload associated with it.
 * The number of payload bytes is encoded in the highest 3 bits of
 * the command byte (although capped at DS_FRAME_MAX_PAYLOAD_SIZE)
 */
#define DS_CMD_PAYLOAD_SIZE(x) ( (x & 0xFF) >> 5 )

#define DS_FRAME_MAX_PAYLOAD_SIZE 5
struct ds_frame_t {
	uint8_t cmd; /* command type */
	uint8_t data[DS_FRAME_MAX_PAYLOAD_SIZE]; /* payload */
	uint8_t chk; /* checksum (XOR all other fields) */
};

#define DS_FRAME_BUFFER_SIZE 5
/* ring buffer and positions for reader and writer */
static struct ds_frame_t ds_buffer[DS_FRAME_BUFFER_SIZE];
/* nibble position in the current frame */
static uint8_t ds_buffer_pos = 0;
static uint8_t ds_w_pos = 0;
static uint8_t ds_r_pos = 0;

int32_t ds_checksum_errors = 0;
int32_t ds_frames_processed = 0;

static void decoder_feed(uint8_t input) {
	if (ds_buffer_pos < 2*sizeof(*ds_buffer)) {
		struct ds_frame_t *fr = &ds_buffer[ds_w_pos];
		uint8_t *b = (uint8_t *) fr;
		/* did we just enter a new frame? wipe the struct */
		if (ds_buffer_pos == 0) {
			memset(fr, 0, sizeof(*fr));
		} else if (ds_buffer_pos >= 2*(offsetof(struct ds_frame_t, cmd)+sizeof(fr->cmd))) {
			/* so we have the command type and know how big the payload should be;
			 * if we alredy have enough nibbles/bytes, advance to the checksum
			 */
			if (2*(offsetof(struct ds_frame_t, data)+DS_CMD_PAYLOAD_SIZE(fr->cmd)) == ds_buffer_pos) {
				ds_buffer_pos = 2*(offsetof(struct ds_frame_t, chk));
			}
		}
		input ^= 1<<ds_buffer_pos%4;
		if (ds_buffer_pos%2 == 0) {
			b[ds_buffer_pos/2] = input&0x0F;
		} else {
			b[ds_buffer_pos/2] |= input<<4;
		}
		ds_buffer_pos++;
		/* finished a complete frame? */
		if (ds_buffer_pos == 2*sizeof(*ds_buffer)) {
			ds_w_pos++;
			ds_buffer_pos = 0;
			if (ds_w_pos == DS_FRAME_BUFFER_SIZE) {
				ds_w_pos = 0;
			}
		}
	}
}

static void decoder_reset(void) {
	ds_buffer_pos = 0;
}

static uint8_t decoder_complete(void) {
	return (DS_FRAME_BUFFER_SIZE+ds_w_pos-ds_r_pos) % DS_FRAME_BUFFER_SIZE;
}

static uint8_t decoder_get_frame(struct ds_frame_t *fr) {
	if (decoder_complete()) {
		cli();
		memcpy(fr, &ds_buffer[ds_r_pos], sizeof(*fr));
		ds_r_pos = (ds_r_pos+1)%DS_FRAME_BUFFER_SIZE;
		sei();
		return 1;
	} else {
		return 0;
	}
}

static uint8_t decoder_verify_frame(struct ds_frame_t *fr) {
	uint8_t sum = 0;
	for (uint8_t i=0; i<sizeof(*fr); i++) {
		sum ^= ((uint8_t*)fr)[i];
	}
	return (sum==0);
}

/* this code is used to store persistent flight assistance data,
 * e.g. whether the ACC has been enabled by Datensprung frames
 */
#define DATENSPRUNG_FA_ACC 0
#define DATENSPRUNG_FA_BARO 1
#define DATENSPRUNG_FA_MAG 2
#define DATENSPRUNG_FA_HEADFREE 3
#define DATENSPRUNG_FA_GPSHOLD 4
static struct {
	/* only settings masked with 1 are touched by Datensprung */
	uint8_t mask;
	uint8_t values;
} datenschlag_fa_settings = {0,0};

static void datenschlag_apply_setting(uint8_t bit, uint8_t *var) {
	// Do we touch the setting?
	if ( ~datenschlag_fa_settings.mask & 1<<bit) return;

	if (datenschlag_fa_settings.values & 1<<bit) {
		*var = 1;
	} else {
		*var = 0;
	}
}

void datenschlag_apply_fa_settings() {
	datenschlag_apply_setting(DATENSPRUNG_FA_ACC, &rcOptions[BOXACC]);
	datenschlag_apply_setting(DATENSPRUNG_FA_BARO, &rcOptions[BOXBARO]);
	datenschlag_apply_setting(DATENSPRUNG_FA_MAG, &rcOptions[BOXMAG]);
	datenschlag_apply_setting(DATENSPRUNG_FA_HEADFREE, &rcOptions[BOXHEADFREE]);
	datenschlag_apply_setting(DATENSPRUNG_FA_GPSHOLD, &rcOptions[BOXGPSHOLD]);
}

static struct {
	/* only channels masked with 1 are touched by Datensprung */
	uint8_t mask;
	/* is the aux channel non-neutral? */
	uint8_t active;
	/* is the switch up or down? */
	uint8_t up;
} datenschlag_aux = {0,0,0};

void datenschlag_process_aux(struct ds_frame_t *fr) {
	uint8_t a = (fr->data[0] & 0x0F) | ((fr->data[1] & 0x0F)<<4);
	uint8_t b = ((fr->data[0] & 0xF0)>>4) | ((fr->data[1] & 0xF0));
	datenschlag_aux.mask   = (a | b);
	datenschlag_aux.active = (    b);
	datenschlag_aux.up     = (a & b);
}

void datenschlag_apply_aux(void) {
#ifndef AUX_CHANNELS
#define AUX_CHANNELS 4
#define AUXN(i) (AUX1-1+(i))
#endif
	for (uint8_t i=0; i<AUX_CHANNELS && i<8; i++) {
		if (!(datenschlag_aux.mask & 1<<i)) continue;
		uint16_t v = 1500;
		if (datenschlag_aux.active & 1<<i) {
			v += ((datenschlag_aux.up & 1<<i) ? 500 : -500);
		}
		rcData[AUXN(i+1)] = v;
	}
}

static struct {
	uint16_t min;
	uint16_t max;
} datenschlag_calib = {~(uint16_t)0, 0};

void datenschlag_feed(uint16_t value) {
	static uint16_t last_token = 0;
	/* let the calibration decay over time */
	if (datenschlag_calib.min < ~((uint16_t)0)) datenschlag_calib.min++;
	if (datenschlag_calib.max >  0) datenschlag_calib.max--;
	/* calibrate */
	if (value < datenschlag_calib.min) datenschlag_calib.min = value;
	if (value > datenschlag_calib.max) datenschlag_calib.max = value;

	/* calculate the nibble value of our signal */

	/* we need to map the value in the range from min to max into an
	 * interval from 0 to 0x0F+2:
	 * 0         == minimum calibration (and nibble ack)
	 * 0x0F+2    == maximum calibration (and frame start)
	 * 1-0x0F+1  == payload nibble value (+1)
	 */
	uint32_t input = value-datenschlag_calib.min;
	uint32_t span = datenschlag_calib.max-datenschlag_calib.min;
	uint32_t o_max = 0x0F+2;
	uint16_t v = (input+span/o_max/2)*o_max/span;
	if (v == 0x0F+2) {
		decoder_reset();
	} else if (last_token != v && last_token > 0 && last_token < 0x0F+2) {
		decoder_feed(last_token-1);
	}
	last_token = v;
}

void datenschlag_reset() {
	decoder_reset();
	datenschlag_calib.min = ~0;
	datenschlag_calib.max = 0;
}

void datenschlag_process() {
	struct ds_frame_t frame;
	while (decoder_get_frame(&frame)) {
		if (! decoder_verify_frame(&frame)) {
			ds_checksum_errors++;
			debug[1] = ds_checksum_errors;
			continue;
		}
		ds_frames_processed++;
		debug[0] = ds_frames_processed;

		/* evaluate the received frames */
		switch (frame.cmd) {
			case (4<<5 | 0x0A): // 0x8A
				/* flight assistance data has 4 payload bytes:
				 * mask data (which systems do we wish to control?)
				 * mask data mask [sic] (which mask bits do we want to change?)
				 * value data (which systems do we want enabled?)
				 * value data mask (which states do we want to switch?)
				 */
				datenschlag_fa_settings.mask = (datenschlag_fa_settings.mask & ~frame.data[1]) | (frame.data[0] & frame.data[1]);
				datenschlag_fa_settings.values = (datenschlag_fa_settings.values & ~frame.data[3]) | (frame.data[2] & frame.data[3]);
				break;
#if defined(LED_FLASHER)
			case (1<<5 | 0x0F): // 0x2F
				/* set LED flash sequence, 1 byte payload */
				led_flasher_set_sequence(frame.data[0]);
				break;
#endif
			case (2<<5 | 0x0D): // 0x4D
				/* debugging data, 2 bytes payload */
				debug[0] = frame.data[0];
				debug[1] = frame.data[1];
				break;
			case (1<<5 | 0x0A): // 0x2A
			case (2<<5 | 0x0A): // 0x4A
				/* AUX channel switches, 1 or 2 byte payload */
				datenschlag_process_aux(&frame);
				break;
			case (2<<5 | 0x04): // 0x44
				/* headfree adjustment, 2 byte payload */
				headFreeModeHold = ((int16_t) frame.data[0])<<8 | frame.data[1];
				break;
#if defined(I2CAM_GIMBAL_SERVO)
			case (1<<5 | 0x0C): //0x2C
				/* camera gimbal adjustment of pitch axis */
				gimbal_base_angle[PITCH] = (1800L*frame.data[0]/255)-900;;
				break;
#endif
			default:
				break;
		}
	}
}

#endif
