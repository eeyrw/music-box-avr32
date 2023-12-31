#include "SynthCore.h"
#include <stdint.h>
#include <stdio.h>
#include "WaveTable.h"
#include "EnvelopeTable.h"
#include <asf.h>

void SynthInit(Synthesizer *synth)
{
	SoundUnit *soundUnits = synth->SoundUnitList;
	for (uint8_t i = 0; i < POLY_NUM; i++)
	{
		soundUnits[i].increment = 0;
		soundUnits[i].wavetablePos = 0;
		soundUnits[i].envelopeLevel = 0;
		soundUnits[i].envelopePos = 0;
		soundUnits[i].val = 0;
		soundUnits[i].waveTableAddress = (uint32_t)WaveTable;
		soundUnits[i].waveTableLen = WAVETABLE_LEN;
		soundUnits[i].waveTableLoopLen = WAVETABLE_LOOP_LEN;
		soundUnits[i].waveTableAttackLen = WAVETABLE_ATTACK_LEN;
	}
	synth->lastSoundUnit = 0;
}
//#ifdef RUN_TEST
void NoteOnC(Synthesizer *synth, uint8_t note)
{
	uint32_t lastSoundUnit = synth->lastSoundUnit;
	SoundUnit *soundUnits = synth->SoundUnitList;

	cpu_irq_disable();
	soundUnits[lastSoundUnit].increment = WaveTable_Increment[note & 0x7F];
	soundUnits[lastSoundUnit].wavetablePos = 0;
	soundUnits[lastSoundUnit].waveTableAddress = (uint32_t)WaveTable;
	soundUnits[lastSoundUnit].waveTableLen = WAVETABLE_LEN;
	soundUnits[lastSoundUnit].waveTableLoopLen = WAVETABLE_LOOP_LEN;
	soundUnits[lastSoundUnit].waveTableAttackLen = WAVETABLE_ATTACK_LEN;
	soundUnits[lastSoundUnit].envelopeLevel = 255;
	soundUnits[lastSoundUnit].envelopePos = 0;
	cpu_irq_enable();

	lastSoundUnit++;
	if (lastSoundUnit == POLY_NUM)
		lastSoundUnit = 0;

	synth->lastSoundUnit = lastSoundUnit;
}

void NoteOnGreedy(Synthesizer *synth, uint8_t note)
{
	uint32_t lastSoundUnit = synth->lastSoundUnit;
	SoundUnit *soundUnits = synth->SoundUnitList;
	
	int minimalLevelUnitIdx = 0;
	int level = 0xFF;
	for (int i=0;i<POLY_NUM;i++)
	{
		if(soundUnits[i].envelopeLevel < level)
		{
			level = soundUnits[i].envelopeLevel;
			minimalLevelUnitIdx = i;
		}
	}

cpu_irq_disable();

			soundUnits[minimalLevelUnitIdx].increment = WaveTable_Increment[note & 0x7F];
			soundUnits[minimalLevelUnitIdx].wavetablePos = 0;
			soundUnits[minimalLevelUnitIdx].waveTableAddress = (uint32_t)WaveTable;
			soundUnits[minimalLevelUnitIdx].waveTableLen = WAVETABLE_LEN;
			soundUnits[minimalLevelUnitIdx].waveTableLoopLen = WAVETABLE_LOOP_LEN;
			soundUnits[minimalLevelUnitIdx].waveTableAttackLen = WAVETABLE_ATTACK_LEN;
			soundUnits[minimalLevelUnitIdx].envelopeLevel = 255;
			soundUnits[minimalLevelUnitIdx].envelopePos = 0;
cpu_irq_enable();
	
}

void SynthC(Synthesizer *synth)
{
	synth->mixOut = 0;
	int16_t *pWaveTable;
	uint32_t waveTablePosInt;
	SoundUnit *soundUnits = synth->SoundUnitList;

	for (uint32_t i = 0; i < POLY_NUM; i++)
	{
		if (soundUnits[i].envelopeLevel != 0)
		{
			pWaveTable = (int16_t *)soundUnits[i].waveTableAddress;
			waveTablePosInt = (soundUnits[i].wavetablePos) >> 8;
			soundUnits[i].val = ((int32_t)soundUnits[i].envelopeLevel) * pWaveTable[waveTablePosInt];
			soundUnits[i].sampleVal = pWaveTable[waveTablePosInt];
			uint32_t waveTablePos = soundUnits[i].increment +
									soundUnits[i].wavetablePos;

			if (waveTablePos >= soundUnits[i].waveTableLen << 8)
				waveTablePos -= soundUnits[i].waveTableLoopLen << 8;
			soundUnits[i].wavetablePos = waveTablePos;
			synth->mixOut += soundUnits[i].val;
		}
	}
}

void GenDecayEnvlopeC(Synthesizer *synth)
{
	SoundUnit *soundUnits = synth->SoundUnitList;
	for (uint32_t i = 0; i < POLY_NUM; i++)
	{
		if ((soundUnits[i].wavetablePos >> 8) >= soundUnits[i].waveTableAttackLen &&
			soundUnits[i].envelopePos < (sizeof(EnvelopeTable) - 1))
		{
			soundUnits[i].envelopeLevel = EnvelopeTable[soundUnits[i].envelopePos];
			soundUnits[i].envelopePos += 1;
		}
	}
}
//#endif
