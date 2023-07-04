#ifndef __WAVETABLE__
#define __WAVETABLE__
// Sample name: STW PIANO f D4
// Sample's base frequency: 293.8451872069176 Hz
// Sample's sample rate: 32000 Hz
#define WAVETABLE_LEN 36747
#define WAVETABLE_ATTACK_LEN 36203
#define WAVETABLE_LOOP_LEN 544
#define WAVETABLE_ACTUAL_LEN 36748

#ifndef __ASSEMBLER__
#include <stdint.h>
extern const int16_t WaveTable[WAVETABLE_ACTUAL_LEN];
extern const uint16_t WaveTable_Increment[];
#else
.extern	WaveTable
.extern WaveTable_Increment
#endif

#endif