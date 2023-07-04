#include <avr32/io.h>
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
SynthAsm:
#define pSoundUnit r8
#define loopIndex r9
#define mixOut r10
#define USE_Interpolation
pushm	r4-r7,lr
mov loopIndex, POLY_NUM

mov mixOut, 0
mov pSoundUnit,r12


loopSynth:
    ld.w r5,pSoundUnit[pEnvelopeLevel]
	cp.w r5,0
    breq loopSynthEnd
    ld.w r5,pSoundUnit[pWaveTableAddress]
    ld.w r6,pSoundUnit[pWavetablePos]
    asr  r6,8 //wavetablePos /= 256
    lsl  r6,1 //wavetablePos *= 2
    add r6,r5  // Load signed 16bit sample to t6
    #ifdef USE_Interpolation
	mov r7,2
    add r6,r7    // Load 2nd signed 16bit sampleaddr
    #endif
    ld.sh r6,r6
    #ifdef USE_Interpolation 
    ld.sh r7,r7
    sub r7,r6
    ld.ub r5,pSoundUnit[pWavetablePos]
    mul t7,t7,t5
    asr r7,8 //wavetablePos /= 256
    add t6,t6,t7
    #endif
    st.w pSoundUnit[pSampleVal],r6
 
    mac mixOut,r4,r6
    //sw t4,pVal(pSoundUnit)
   
    ld.w r6,pSoundUnit[pWavetablePos]
    ld.w r5,pSoundUnit[pIncrement]
    add r6,r5
    ld.w r5,pSoundUnit[pWaveTableLen]
    asr  r5,8 //wavetablePos /= 256 //pWaveTableLen*=256    
	cp.w r5,r6
    brge wavePosUpdateEnd  //bgeu:">="
    ld.w r5,pSoundUnit[pWaveTableLoopLen]
    lsr r5,8 //waveTableLoopLen*=256
    sub r6,r5
    wavePosUpdateEnd:
    st.w pSoundUnit[pWavetablePos],r6
loopSynthEnd:

sub loopIndex,1 // set n = n-1
sub pSoundUnit,-SoundUnitSize
cp.w loopIndex,0
brne loopSynth

asr mixOut,8 // mixOut/=255 ( envelope)


mov r5,100
mul mixOut,r5 //pre vol
asr mixOut,8



mov pSoundUnit,r12

ld.uh r6,pSoundUnit[pVolume]
mul mixOut,r6 //main vol
asr mixOut,10



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
sw mixOut,pSoundUnit[pMixOut]


ldm	sp++,r4-r7,pc
.end SynthAsm

GenDecayEnvlopeAsm:
#define pSoundUnitGenEnv r8
#define loopIndexGenEnv r9

mov pSoundUnitGenEnv,r12
mov loopIndexGenEnv,POLY_NUM
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
