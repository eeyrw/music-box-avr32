#include <avr32/io.h>
#include <stdio.h>
#include <asf.h>



/* Connection of the light sensor */
#define EXAMPLE_ADC_LIGHT_CHANNEL           0
#define EXAMPLE_ADC_LIGHT_PIN               AVR32_ADC_AD_0_PIN
#define EXAMPLE_ADC_LIGHT_FUNCTION          AVR32_ADC_AD_0_FUNCTION

uint8_t AudioBuffer[1024];


/** \brief Main application entry point - init and loop to display ADC values */
void ADCInit(void)
{
	/** GPIO pin/adc-function map. */
	const gpio_map_t ADC_GPIO_MAP = {
		{EXAMPLE_ADC_LIGHT_PIN, EXAMPLE_ADC_LIGHT_FUNCTION}
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

	/* Enable the ADC channels. */
	adc_enable(&AVR32_ADC, EXAMPLE_ADC_LIGHT_CHANNEL);

}

int16_t ReadADC(void)
{
	signed short adc_value_light = -1;
	/* Start conversions on all enabled channels */
	adc_start(&AVR32_ADC);


	/* Get value for the light adc channel */
	adc_value_light = adc_get_value(&AVR32_ADC,
	EXAMPLE_ADC_LIGHT_CHANNEL);
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
	pwm_channel.cdty = 90; /* Channel duty cycle, should be < CPRD. */
	pwm_channel.cprd = 1<<12-1; /* Channel period. */
	pwm_channel.cupd = 0; /* Channel update is not used here. */
	pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
	pwm_channel.CMR.cpol = PWM_POLARITY_HIGH;      /* Channel polarity. */
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
	pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_4; /* Channel prescaler. */
	
	/* Set channel configuration to channel 0,1. */
	pwm_channel_init(0, &pwm_channel);
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

	pdca_reload_channel(0,
	(void *)AudioBuffer, sizeof(AudioBuffer));

}

void SSCInit(void)
{
	volatile avr32_ssc_t *ssc = &AVR32_SSC;
	static const gpio_map_t SSC_GPIO_MAP =
	{
		{AVR32_SSC_TX_CLOCK_0_0_PIN,      AVR32_SSC_TX_CLOCK_0_0_FUNCTION     },
		{AVR32_SSC_TX_DATA_0_0_PIN,       AVR32_SSC_TX_DATA_0_0_FUNCTION      },
		{AVR32_SSC_TX_FRAME_SYNC_0_0_PIN, AVR32_SSC_TX_FRAME_SYNC_0_0_FUNCTION}
	};
	// Assign GPIO to SSC.
	gpio_enable_module(SSC_GPIO_MAP, sizeof(SSC_GPIO_MAP) / sizeof(SSC_GPIO_MAP[0]));

	// SSC init in I2S mode.
	ssc_i2s_init(ssc, 32000, 16, 32, SSC_I2S_MODE_STEREO_OUT, sysclk_get_pba_hz());
	// ssc_i2s_enable_interrupts(ssc,AVR32_SSC_IER_TXEMPTY);
	
	const pdca_channel_options_t PDCA_OPTIONS = {
		/* Select peripheral - data is transmitted on USART TX line */
		.pid = AVR32_PDCA_PID_SSC_TX,
		/* Select size of the transfer */
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE,

		/* Memory address */
		.addr = (void *)AudioBuffer,
		/* Transfer counter */
		.size = sizeof(AudioBuffer),

		/* Next memory address */
		.r_addr = NULL,
		/* Next transfer counter */
		.r_size = 0,
	};

	/* Initialize the PDCA channel with the requested options. */
	pdca_init_channel(0, &PDCA_OPTIONS);
	
	/* Register PDCA IRQ interrupt. */
	pdca_enable_interrupt_transfer_complete(0);
	/* Enable PDCA interrupt each time the reload counter reaches zero, i.e.
	* each time half of the ASCII animation (either anim1 or anim2) is
	* transferred. */
	pdca_enable_interrupt_reload_counter_zero(0);

	/* Enable now the transfer. */
	pdca_enable(0);
	
	
}

/*! \brief Main function, application starts executing here after
*         initializing the stack pointer.
*/
int main(void)
{
	sysclk_init();
	PwmLedInit();
	ADCInit();
	SSCInit();
	init_dbg_rs232(sysclk_get_pba_hz());

	cpu_irq_disable();		// Initialize interrupt vectors.
	INTC_init_interrupts();
	INTC_register_interrupt(&pdca_int_handler, AVR32_PDCA_IRQ_0,
	AVR32_INTC_INT0);

	cpu_irq_enable();	udc_start();	udc_attach();


	
	printf("Debug uart is working!");

	while (1)
	{
		int16_t adcV = ReadADC();
		AVR32_PWM.channel[0].cupd = adcV;
	}
}
