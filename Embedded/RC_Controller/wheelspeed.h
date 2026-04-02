#ifndef WHEELSPEED_H_
#define WHEELSPEED_H_

void wheelspeed_init(void);
void update_speed_buffer(float period, float unused);


extern volatile bool new_pulse;

#endif
