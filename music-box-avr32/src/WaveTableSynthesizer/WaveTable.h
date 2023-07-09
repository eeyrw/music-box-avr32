#ifndef __WAVETABLE__
#define __WAVETABLE__
// Sample name: STW PIANO f D4
// Sample's base frequency: 588.2693256717 Hz
// Sample's sample rate: 32000 Hz
#define WAVETABLE_LEN 90241
#define WAVETABLE_ATTACK_LEN 70190
#define WAVETABLE_LOOP_LEN 20051
#define WAVETABLE_ACTUAL_LEN 90242

#ifndef __ASSEMBLER__
#include <stdint.h>
extern const int16_t WaveTable[WAVETABLE_ACTUAL_LEN];
extern const uint16_t WaveTable_Increment[];
#else
.extern	WaveTable
.extern WaveTable_Increment
#endif

#endif