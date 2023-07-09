#include <avr32/io.h>
#include <stdio.h>
#include <asf.h>
#include "Player.h"
Player mPlayer;

#define AUDIO_BUFFER_SIZE 512

int16_t AudioBuffer[AUDIO_BUFFER_SIZE*2];
volatile uint32_t BufferToggle;
volatile uint32_t RequireUpdate;

void synth_wave(short *buffer_pp, int len);

#define ONE_FIX_POINT 255
int lastGain = ONE_FIX_POINT;
#define timeConstAttack 10
#define timeConstRelease 200
int lastPeak = 0;
int compressEffect(int sampleIn,int threshold)
{
	int currentGain;
	int currentPeak;
	if (abs(sampleIn) > threshold)
	{
		currentGain = threshold * ONE_FIX_POINT / abs(sampleIn);
	}
	else
	{
		currentGain = ONE_FIX_POINT;
	}
	currentPeak = currentGain;

	if (lastPeak > currentPeak)
	{
		currentPeak = (timeConstAttack*lastPeak + (ONE_FIX_POINT-timeConstAttack) *currentGain)/ ONE_FIX_POINT;
	}
	else
	{
		currentPeak = (timeConstRelease*lastPeak + (ONE_FIX_POINT - timeConstRelease) *currentGain) / ONE_FIX_POINT;
	}
	lastPeak = currentPeak;

	return sampleIn * currentPeak / ONE_FIX_POINT;
}

void compressEffect2(short* samplesIn,int len)
{
	int peak = 32767;
	for (int i=0;i<len;i++)
	{
		int ap = abs(samplesIn[i]);
		if(ap>peak)
		peak = ap;
	}
	int gain = 32767*255/peak;
	
	if(gain<255)
	{
		for (int i=0;i<len;i++)
		{
			samplesIn[i] = (samplesIn[i]*gain)>>8;
		}
	}else
	{
	}


}

static void wp_start_ssc_tx(void)
{
	/* The PDCA is not able to synchronize its start of transfer with the
	* SSC start of period, so this has to be done by polling the TF pin.
	* Not doing so may result in channels being swapped randomly.
	*/
	Bool is_global_interrupt_enabled = Is_global_interrupt_enabled();
	Disable_global_interrupt();
	while(gpio_get_pin_value(AVR32_SSC_TX_FRAME_SYNC_0_0_PIN));
	while(!gpio_get_pin_value(AVR32_SSC_TX_FRAME_SYNC_0_0_PIN));
	pdca_enable(0);
	pdca_enable_interrupt_reload_counter_zero(0);
	if (is_global_interrupt_enabled)
	Enable_global_interrupt();
}



/** \brief Main application entry point - init and loop to display ADC values */
void ADCInit(void)
{
	/** GPIO pin/adc-function map. */
	const gpio_map_t ADC_GPIO_MAP = {
		{AVR32_ADC_AD_0_PIN, AVR32_ADC_AD_0_FUNCTION}
	};

	/* Assign and enable GPIO pins to the ADC function. */
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) /
	sizeof(ADC_GPIO_MAP[0]));

	/* Configure the ADC peripheral module.
	* Lower the ADC clock to match the ADC characteristics (because we
	* configured the CPU clock to 12MHz, and the ADC clock characteristics are
	*  usually lower; cf. the ADC Characteristic section in the datasheet). */
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(&AVR32_ADC);
}

int16_t ReadADC(uint16_t chn)
{
	signed short adc_value_light = -1;
	
	if (adc_get_status(&AVR32_ADC, chn) == false)
	adc_enable(&AVR32_ADC,chn);
	/* Start conversions on all enabled channels */
	adc_start(&AVR32_ADC);


	/* Get value for the light adc channel */
	adc_value_light = adc_get_value(&AVR32_ADC,chn);
	return adc_value_light;
}


void PwmLedInit(void)
{
	/* PWM controller configuration. */
	pwm_opt_t pwm_opt =
	{
		.diva = AVR32_PWM_DIVA_CLK_OFF,
		.divb = AVR32_PWM_DIVB_CLK_OFF,
		.prea = AVR32_PWM_PREA_MCK,
		.preb = AVR32_PWM_PREB_MCK
	};

	/* PWM channel configuration structure. */
	avr32_pwm_channel_t pwm_channel = { .ccnt = 0 };
	
	/* Enable the alternative mode of the output pin to connect it to the PWM
	* module within the device. */
	gpio_enable_module_pin(AVR32_PWM_0_0_PIN, AVR32_PWM_0_0_FUNCTION);
	gpio_enable_module_pin(AVR32_PWM_1_0_PIN, AVR32_PWM_1_0_FUNCTION);

	/* Initialize the PWM module. */
	pwm_init(&pwm_opt);


	/* With these settings, the output waveform period will be:
	* (115200/256)/20 == 22.5Hz == (MCK/prescaler)/period, with
	* MCK == 115200Hz, prescaler == 256, period == 20. */
	pwm_channel.cdty = 0; /* Channel duty cycle, should be < CPRD. */
	pwm_channel.cprd = 1023; /* Channel period. */
	pwm_channel.cupd = 0; /* Channel update is not used here. */
	pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
	pwm_channel.CMR.cpol = PWM_POLARITY_HIGH;      /* Channel polarity. */
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
	pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_4; /* Channel prescaler. */
	
	/* Set channel configuration to channel 0,1. */
	pwm_channel_init(0, &pwm_channel);
	pwm_channel.cprd = 255; /* Channel period. */
	pwm_channel.CMR.cpol = PWM_POLARITY_HIGH;      /* Channel polarity. */
	pwm_channel_init(1, &pwm_channel);

	/* Start channel 0. */
	pwm_start_channels(1 << 0 | 1 << 1);
}


#if (UC3C || UC3L)
ISR(pdca_int_handler, AVR32_PDCA_IRQ_GROUP0, 0)
#else
ISR(pdca_int_handler, AVR32_PDCA_IRQ_GROUP, 0)
#endif
{
	if(RequireUpdate)
		RequireUpdate = 0;
	if(BufferToggle)
	{
		pdca_reload_channel(0,
		(void *)AudioBuffer, AUDIO_BUFFER_SIZE);
		BufferToggle = 0;
	}else
	{
		pdca_reload_channel(0,
		(void *)(AudioBuffer+AUDIO_BUFFER_SIZE), AUDIO_BUFFER_SIZE);
		BufferToggle = 1;
	}
	RequireUpdate = 1;
}

void CodecInit(void)
{
	gpio_set_gpio_pin(CODEC_SD_PIN);
}

void SSCInit(void)
{
	static const gpio_map_t SSC_GPIO_MAP =
	{
		{AVR32_SSC_TX_CLOCK_0_0_PIN,      AVR32_SSC_TX_CLOCK_0_0_FUNCTION     },
		{AVR32_SSC_TX_DATA_0_0_PIN,       AVR32_SSC_TX_DATA_0_0_FUNCTION      },
		{AVR32_SSC_TX_FRAME_SYNC_0_0_PIN, AVR32_SSC_TX_FRAME_SYNC_0_0_FUNCTION}
	};
	// Assign GPIO to SSC.
	gpio_enable_module(SSC_GPIO_MAP, sizeof(SSC_GPIO_MAP) / sizeof(SSC_GPIO_MAP[0]));

	// SSC init in I2S mode.
	ssc_i2s_init(&AVR32_SSC, 32000, 16, 16, SSC_I2S_MODE_STEREO_OUT, sysclk_get_pba_hz());
	
	const pdca_channel_options_t PDCA_OPTIONS = {
		/* Select peripheral - data is transmitted on USART TX line */
		.pid = AVR32_PDCA_PID_SSC_TX,
		/* Select size of the transfer */
		.transfer_size = PDCA_TRANSFER_SIZE_HALF_WORD,

		/* Memory address */
		.addr = (void *)AudioBuffer,
		/* Transfer counter */
		.size = AUDIO_BUFFER_SIZE,

		/* Next memory address */
		.r_addr = NULL,
		/* Next transfer counter */
		.r_size = 0,
	};
	
	BufferToggle = 0;
	RequireUpdate = 0;

	/* Initialize the PDCA channel with the requested options. */
	pdca_init_channel(0, &PDCA_OPTIONS);
	
	/* Register PDCA IRQ interrupt. */
	/* Enable PDCA interrupt each time the reload counter reaches zero, i.e.
	* each time half of the ASCII animation (either anim1 or anim2) is
	* transferred. */
	pdca_enable_interrupt_reload_counter_zero(0);

	/* Enable now the transfer. */
	pdca_enable(0);
	
	
}

void synth_wave(short *buffer_pp, int len)
{


	for (int i = 0; i < len; i += 2)
	{
		Player32kProc(&mPlayer);


		int32_t rawSynthOutput = mPlayer.mainSynthesizer.mixOut;
		//rawSynthOutput = compressEffect(rawSynthOutput, 20000);
		//if (rawSynthOutput < -32768)
		//{
		//rawSynthOutput = -32768;
		//}
		//else if (rawSynthOutput > 32767)
		//{
		//rawSynthOutput = 32767;
		//}
		
		buffer_pp[i] = rawSynthOutput;
		buffer_pp[i + 1] = rawSynthOutput;
	}
	//compressEffect2(buffer_pp,len);
}

/*! \brief Main function, application starts executing here after
*         initializing the stack pointer.
*/
int main(void)
{
	int prevAmp=0;
	sysclk_init();
	PwmLedInit();
	ADCInit();
	SSCInit();
	init_dbg_rs232(sysclk_get_pba_hz());
	CodecInit();

	cpu_irq_disable();		// Initialize interrupt vectors.
	INTC_init_interrupts();
	INTC_register_interrupt(&pdca_int_handler, AVR32_PDCA_IRQ_0,
	AVR32_INTC_INT0);

	cpu_irq_enable();	//udc_start();	//udc_attach();

	PlayerInit(&mPlayer);
	PlayerPlay(&mPlayer);
	
	printf("Debug uart is working!");

	while (1)
	{

		if(RequireUpdate)
		{
			if(BufferToggle)
			{
				synth_wave((void *)(AudioBuffer+AUDIO_BUFFER_SIZE), AUDIO_BUFFER_SIZE);
			}else
			{
				synth_wave((void *)(AudioBuffer), AUDIO_BUFFER_SIZE);
			}
			RequireUpdate = 0;
		}
		PlayerProcess(&mPlayer);
		mPlayer.mainSynthesizer.volume = ReadADC(0);
		int currentAmp = abs(mPlayer.mainSynthesizer.mixOut)>>2;
		int a = 20;
		AVR32_PWM.channel[0].cupd = (a*currentAmp+(255-a)*prevAmp)>>8;
		prevAmp = AVR32_PWM.channel[0].cupd;
	}
}
