#include <xc.h>
#include "WaveTable.h"


//.set noreorder
.global SynthAsm
.global GenDecayEnvlopeAsm
.global NoteOnAsm

// typedef struct _SoundUnit
// {
//     uint32_t wavetablePos;
//     uint32_t waveTableAddress;
//     uint32_t waveTableLen;
//     uint32_t waveTableLoopLen;
//     uint32_t waveTableAttackLen;
//     uint32_t envelopePos;
//     uint32_t increment;
//     int32_t val;
//     int32_t sampleVal;
//     uint32_t envelopeLevel;
// }SoundUnit;

// typedef struct _Synthesizer
// {
//     SoundUnit SoundUnitList[POLY_NUM];
//     int32_t mixOut;
//     uint32_t lastSoundUnit;
// }Synthesizer;

.equ pWavetablePos , 0
.equ pWaveTableAddress , 4
.equ pWaveTableLen , 8
.equ pWaveTableLoopLen , 12
.equ pWaveTableAttackLen , 16
.equ pEnvelopePos ,20
.equ pIncrement , 24
.equ pVal , 28
.equ pSampleVal , 32
.equ pEnvelopeLevel , 36
.equ SoundUnitSize,40


.equ ENVELOP_LEN,256
.equ POLY_NUM,20
.equ pMixOut,SoundUnitSize*POLY_NUM
.equ pLastSoundUnit,pMixOut+4
.equ pVolume,pLastSoundUnit+4

.section .text
.ent SynthAsm
SynthAsm:
#define pSoundUnit t1
#define loopIndex t2
#define mixOut t3
#define USE_MADD
#define USE_Interpolation

li loopIndex, POLY_NUM

move mixOut, zero
move pSoundUnit,a0

#ifdef USE_MADD
mtlo zero
mthi zero
#endif

loopSynth:
    lw t4,pEnvelopeLevel(pSoundUnit)
    beqz t4,loopSynthEnd
    lw t5,pWaveTableAddress(pSoundUnit)
    lw t6,pWavetablePos(pSoundUnit)
    srl  t6,t6,8 //wavetablePos /= 256
    sll  t6,t6,1 //wavetablePos *= 2
    add t6,t5,t6  // Load signed 16bit sample to t6
    #ifdef USE_Interpolation
    addi t7,t6,2    // Load 2nd signed 16bit sampleaddr
    #endif
    lh t6,(t6)
    #ifdef USE_Interpolation 
    lh t7,(t7)
    sub t7,t7,t6
    lbu t5,pWavetablePos(pSoundUnit)
    mflo mixOut
    mul t7,t7,t5
    mtlo mixOut
    sra  t7,t7,8 //wavetablePos /= 256
    add t6,t6,t7
    #endif
    sw t6,pSampleVal(pSoundUnit)
 
    #ifdef USE_MADD
    madd t4,t6
    #else
    mul t4,t6,t4 //sample*envelope
    // sra t4,t4,8
    sw t4,pVal(pSoundUnit)
    add mixOut,t4,mixOut //mixOut+=sample*envelope
    #endif
    
    lw t6,pWavetablePos(pSoundUnit)
    lw t5,pIncrement(pSoundUnit)
    add t6,t5,t6
    lw t5,pWaveTableLen(pSoundUnit)
    sll t5,t5,8 //pWaveTableLen*=256    
    bgeu t5,t6,wavePosUpdateEnd  //bgeu:">="
    lw t5,pWaveTableLoopLen(pSoundUnit)
    sll t5,t5,8 //waveTableLoopLen*=256
    sub t6,t6,t5
    wavePosUpdateEnd:
    sw t6,pWavetablePos(pSoundUnit)
loopSynthEnd:

addi loopIndex,loopIndex,-1 // set n = n-1
addi pSoundUnit,pSoundUnit,SoundUnitSize
bnez loopIndex,loopSynth

#ifdef USE_MADD
mflo mixOut
#endif
sra mixOut,mixOut,8 // mixOut/=255 ( envelope)


li t5,100
mul mixOut,t5,mixOut //pre vol
sra mixOut,mixOut,8



move pSoundUnit,a0

lh t6,pVolume(pSoundUnit)
mul mixOut,t6,mixOut //main vol
sra mixOut,mixOut,10



/*
li t5,-32766
bge mixOut,t5,saturateLowerBoundSatisfied
move mixOut,t5
saturateLowerBoundSatisfied:
li t5,32766
ble mixOut,t5,saturateEnd
move mixOut,t5
saturateEnd:
*/
sw mixOut,pMixOut(pSoundUnit)


jr ra
.end SynthAsm

.ent GenDecayEnvlopeAsm
GenDecayEnvlopeAsm:
#define pSoundUnitGenEnv t1
#define loopIndexGenEnv t2

move pSoundUnitGenEnv,a0
li loopIndexGenEnv,POLY_NUM
loopGenDecayEnvlope:
// void GenDecayEnvlopeC(Synthesizer* synth)
// {
//     SoundUnit* soundUnits = synth->SoundUnitList//
//     for (uint32_t i = 0// i < POLY_NUM// i++)
//     {
//         if((soundUnits[i].wavetablePos>>8) >=soundUnits[i].waveTableAttackLen &&
//                 soundUnits[i].envelopePos <sizeof(EnvelopeTable)-1)
//         {
//             soundUnits[i].envelopeLevel = EnvelopeTable[soundUnits[i].envelopePos]//
//             soundUnits[i].envelopePos += 1//
//         }
//     }
// }
    lw t5,pWavetablePos(pSoundUnitGenEnv)
    lw t6,pWaveTableAttackLen(pSoundUnitGenEnv)
    sll t6,t6,8
    bltu t5,t6,conditionEnd // blt:"<"
    lw t5,pEnvelopePos(pSoundUnitGenEnv)
    li t6,(ENVELOP_LEN-1)
    bgeu t5,t6,conditionEnd // bhs Higher or same (unsigned >= )
    la t6,EnvelopeTable
    add t6,t5,t6
    lbu t6,(t6)   // Load envelope to r6
    sw t6,pEnvelopeLevel(pSoundUnitGenEnv)
    addi t5,t5,1
    sw t5,pEnvelopePos(pSoundUnitGenEnv)
    conditionEnd:
addi loopIndexGenEnv,loopIndexGenEnv,-1 // set n = n-1
addi pSoundUnitGenEnv,pSoundUnitGenEnv,SoundUnitSize
bnez loopIndexGenEnv,loopGenDecayEnvlope
jr ra
.end GenDecayEnvlopeAsm

.ent NoteOnAsm
NoteOnAsm:
#define pSynth t4
#define note a1
move pSynth,a0
// void NoteOnC(Synthesizer* synth,uint8_t note)
// {
//     uint8_t lastSoundUnit = synth->lastSoundUnit//
//     SoundUnit* soundUnits = synth->SoundUnitList//

//     //disable_interrupts()//
//     soundUnits[lastSoundUnit].increment = WaveTable_Increment[note&0x7F]//
//     soundUnits[lastSoundUnit].wavetablePos = 0//
//     soundUnits[lastSoundUnit].waveTableAddress = (uint32_t)WaveTable//
//     soundUnits[lastSoundUnit].waveTableLen = WAVETABLE_LEN//
//     soundUnits[lastSoundUnit].waveTableLoopLen = WAVETABLE_LOOP_LEN//
//     soundUnits[lastSoundUnit].waveTableAttackLen = WAVETABLE_ATTACK_LEN//
//     //enable_interrupts()//

//     lastSoundUnit++//
//     if (lastSoundUnit== POLY_NUM)
//         lastSoundUnit = 0//

//     synth->lastSoundUnit=lastSoundUnit//
// }

lw t5,pLastSoundUnit(pSynth)

li t6,SoundUnitSize
mul t5,t5,t6
add pSynth,pSynth,t5
li t5,0x7F
and note,note,t5
sll note,note,1
la t5,WaveTable_Increment
add t5,t5,note
lhu t5,(t5)

// Disable all interrupt
di

sw t5,pIncrement(pSynth)
move t5,zero
sw t5,pWavetablePos(pSynth)
la t5,WaveTable
sw t5,pWaveTableAddress(pSynth)
li t5,WAVETABLE_LEN
sw t5,pWaveTableLen(pSynth)
li t5,WAVETABLE_LOOP_LEN
sw t5,pWaveTableLoopLen(pSynth)
li t5,WAVETABLE_ATTACK_LEN
sw t5,pWaveTableAttackLen(pSynth)
sw zero,pEnvelopePos(pSynth)
li t5,255
sw t5,pEnvelopeLevel(pSynth)

// Enable all interrupt
ei

move pSynth,a0

lw t5,pLastSoundUnit(pSynth)

addi t5,t5,1
addi t6,t5,-POLY_NUM
bnez t6,updateLastSoundUnitEnd
move t5,zero
updateLastSoundUnitEnd:
sw t5,pLastSoundUnit(pSynth)

jr ra
.end NoteOnAsm
