#include <stdlib.h>
#include <stdint.h>

void decoder_reset(void);
void decoder_feed(int8_t state);
uint8_t decoder_complete(void);
uint8_t decoder_get_frame(struct ds_frame_t *f);
uint8_t decoder_verify_frame(struct ds_frame_t *f);
