#ifndef LEGACY_BASE_IO_H
#define LEGACY_BASE_IO_H

void legacy_load_off();
void legacy_encoder_status(bool encOnOff, float limit = 0.0);
void legacy_read_encoder();
void legacy_cursor_position();
void legacy_read_volts_current();

#endif
