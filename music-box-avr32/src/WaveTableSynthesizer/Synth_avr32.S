#include <avr32/io.h>
#include "WaveTable.h"


//.set noreorder
.global SynthAsmFrame
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
.equ POLY_NUM,24
.equ pMixOut,SoundUnitSize*POLY_NUM
.equ pLastSoundUnit,pMixOut+4
.equ pVolume,pLastSoundUnit+4

.section .text
SynthAsmFrame:
#define pSoundUnit r8
#define loopIndex r9

#define framePtr r11
#define frameSize r10
#define USE_Interpolation

pushm	r0-r3,r4-r7,lr
#define mixOut r3
frameLoopSynth:
mov loopIndex, POLY_NUM
mov mixOut, 0
mov pSoundUnit,r12
loopSynth:
    ld.w r4,pSoundUnit[pEnvelopeLevel]
	cp.w r4,0
    breq loopSynthEnd
    ld.w r5,pSoundUnit[pWaveTableAddress]
    ld.w r6,pSoundUnit[pWavetablePos]
    lsr  r6,8 //wavetablePos /= 256
    lsl  r6,1 //wavetablePos *= 2
    add r6,r5  // Load signed 16bit sample to t6
    #ifdef USE_Interpolation
	mov r7,2
    add r7,r6    // Load 2nd signed 16bit sampleaddr
    #endif
    ld.sh r6,r6
    #ifdef USE_Interpolation 
    ld.sh r7,r7
    sub r7,r6
    ld.ub r5,pSoundUnit[pWavetablePos+3] //AVR32 is big endian
    mul r7,r5
    asr r7,8
    add r6,r7
    #endif
    st.w pSoundUnit[pSampleVal],r6
 
    mac mixOut,r4,r6
    //sw t4,pVal(pSoundUnit)
   
    ld.w r6,pSoundUnit[pWavetablePos]
    ld.w r5,pSoundUnit[pIncrement]
    add r6,r5
    ld.w r5,pSoundUnit[pWaveTableLen]
    lsl  r5,8 //pWaveTableLen*=256    
	cp.w r5,r6
    brhs wavePosUpdateEnd  //brhs:">="
    ld.w r5,pSoundUnit[pWaveTableLoopLen]
    lsl r5,8 //waveTableLoopLen*=256
    sub r6,r5
    wavePosUpdateEnd:
    st.w pSoundUnit[pWavetablePos],r6
loopSynthEnd:

sub loopIndex,1 // set n = n-1
sub pSoundUnit,-SoundUnitSize
cp.w loopIndex,0
brne loopSynth

asr mixOut,8 // mixOut/=255 ( envelope)

mov r5,70
mul mixOut,r5 //pre vol
asr mixOut,8



mov pSoundUnit,r12

ld.uh r6,pSoundUnit[pVolume]
mul mixOut,r6 //main vol
asr mixOut,10
sats mixOut,16 //clip into -32768 ~ 32767

st.w pSoundUnit[pMixOut],mixOut

st.h framePtr++,mixOut
st.h framePtr++,mixOut

frameLoopSynthEnd:
sub frameSize,2 // set n = n-2
cp.w frameSize,0
brne frameLoopSynth
ldm	sp++,r4-r7,r0-r3,pc



GenDecayEnvlopeAsm:
#define pSoundUnitGenEnv r8
#define loopIndexGenEnv r9
pushm	r4-r7,lr
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
    ld.w r5,pSoundUnitGenEnv[pWavetablePos]
    ld.w r6,pSoundUnitGenEnv[pWaveTableAttackLen]
    lsl r6,8
	cp.w r5,r6
    brlo conditionEnd // brlo: unsigned"<"
    ld.w r5,pSoundUnitGenEnv[pEnvelopePos]
    mov r6,(ENVELOP_LEN-1)
	cp.w r5,r6
    brhs conditionEnd // bhs Higher or same (unsigned >= )
	lda.w r6,EnvelopeTable
    add r6,r5
    ld.ub r6,r6   // Load envelope to r6
    st.w pSoundUnitGenEnv[pEnvelopeLevel],r6
    sub r5,-1
	st.w pSoundUnitGenEnv[pEnvelopePos],r5
    conditionEnd:
sub loopIndexGenEnv,1 // set n = n-1
sub pSoundUnitGenEnv,-SoundUnitSize
cp.w loopIndexGenEnv,0
brne loopGenDecayEnvlope
ldm	sp++,r4-r7,pc


NoteOnAsm:
#define pSynth r10
#define note r11
pushm	r4-r7,lr
mov pSynth,r12
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

ld.w r5,pSynth[pLastSoundUnit]

mov r6,SoundUnitSize
mul r5,r6
add pSynth,r5
mov r5,0x7F
and note,r5
lsl note,1
lda.w r5,WaveTable_Increment
add r5,note
ld.uh r5,r5

// Disable all interrupt
ssrf	0x10

st.w pSynth[pIncrement],r5
mov r5,0
st.w pSynth[pWavetablePos],r5
lda.w r5,WaveTable
st.w pSynth[pWaveTableAddress],r5
mov r5,WAVETABLE_LEN
st.w pSynth[pWaveTableLen],r5
mov r5,WAVETABLE_LOOP_LEN
st.w pSynth[pWaveTableLoopLen],r5
mov r5,WAVETABLE_ATTACK_LEN
st.w pSynth[pWaveTableAttackLen],r5
mov r5,0
st.w pSynth[pEnvelopePos],r5
mov r5,255
st.w pSynth[pEnvelopeLevel],r5

// Enable all interrupt
csrf	0x10

mov pSynth,r12

ld.w r5,pSynth[pLastSoundUnit]

sub r5,-1
cp.w r5,POLY_NUM
brne updateLastSoundUnitEnd
mov r5,0
updateLastSoundUnitEnd:
st.w pSynth[pLastSoundUnit],r5

ldm	sp++,r4-r7,pc
.end
