#ifndef __SYNTH_CORE_H__
#define __SYNTH_CORE_H__

#include <stdint.h>

#define POLY_NUM 24

typedef struct _SoundUnit
{
	uint32_t wavetablePos;
	uint32_t waveTableAddress;
	uint32_t waveTableLen;
	uint32_t waveTableLoopLen;
	uint32_t waveTableAttackLen;
	uint32_t envelopePos;
	uint32_t increment;
	int32_t val;
	int32_t sampleVal;
	uint32_t envelopeLevel;
} SoundUnit;

typedef struct _Synthesizer
{
	SoundUnit SoundUnitList[POLY_NUM];
	int32_t mixOut;
	uint32_t lastSoundUnit;
    uint16_t volume;
} Synthesizer;

typedef struct _SampleInfo
{
	uint8_t sampleCoverlowerPitch;
	uint8_t sampleCoverupperPitch;
	uint8_t sampleBasePitch;
	uint8_t reserved;
	uint32_t sampleAddr;
	uint32_t sampleLen;
	uint32_t sampleLoopStart;
	uint32_t sampleLoopLen;
} SampleInfo;

typedef struct _InstrumentInfo
{
	uint8_t instrumentId;
	uint8_t sampleNum;
	uint32_t sampleBaseAddr;
	SampleInfo *samples;
} InstrumentInfo;

extern void SynthInit(Synthesizer *synth);

//#ifdef RUN_TEST
extern void NoteOnGreedy(Synthesizer *synth, uint8_t note);
extern void NoteOnC(Synthesizer *synth, uint8_t note);
extern void SynthC(Synthesizer *synth);
extern void GenDecayEnvlopeC(Synthesizer *synth);
//#endif

extern void NoteOnAsm(Synthesizer *synth, uint8_t note);
extern void GenDecayEnvlopeAsm(Synthesizer *synth);
extern void SynthAsmFrame(Synthesizer *synth,int16_t* framePtr,uint32_t frameSize);

#endif
