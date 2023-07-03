#include <avr32/io.h>
#include <stdio.h>
#include <asf.h>


#define EXAMPLE_PWM_PIN             AVR32_PWM_0_0_PIN
#define EXAMPLE_PWM_FUNCTION        AVR32_PWM_0_0_FUNCTION
#define EXAMPLE_PWM_CHANNEL_ID      0

/* Connection of the light sensor */
#define EXAMPLE_ADC_LIGHT_CHANNEL           0
#define EXAMPLE_ADC_LIGHT_PIN               AVR32_ADC_AD_0_PIN
#define EXAMPLE_ADC_LIGHT_FUNCTION          AVR32_ADC_AD_0_FUNCTION




/** \brief Main application entry point - init and loop to display ADC values */
void ADCInit(void)
{
	/** GPIO pin/adc-function map. */
	const gpio_map_t ADC_GPIO_MAP = {
		{EXAMPLE_ADC_LIGHT_PIN, EXAMPLE_ADC_LIGHT_FUNCTION}
	};
	signed short adc_value_light = -1;

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

	while (true) {
		/* Start conversions on all enabled channels */
		adc_start(&AVR32_ADC);



		/* Get value for the light adc channel */
		adc_value_light = adc_get_value(&AVR32_ADC,
		EXAMPLE_ADC_LIGHT_CHANNEL);
		
		/* Display value to user */
		print_dbg("HEX Value for Channel light : 0x");
		print_dbg_hex(adc_value_light);
		print_dbg("\r\n");

		/* Slow down the display of converted values */
		delay_ms(500);
	}

}


void PWM2Init(void)
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
	
	/* With these settings, the output waveform period will be:
	* (115200/256)/20 == 22.5Hz == (MCK/prescaler)/period, with
	* MCK == 115200Hz, prescaler == 256, period == 20. */
	pwm_channel.cdty = 5; /* Channel duty cycle, should be < CPRD. */
	pwm_channel.cprd = 20; /* Channel period. */
	pwm_channel.cupd = 0; /* Channel update is not used here. */
	pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; /* Channel mode. */
	pwm_channel.CMR.cpol = PWM_POLARITY_LOW;      /* Channel polarity. */
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY;        /* Not used the first time. */
	pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_256; /* Channel prescaler. */
	
	/* Enable the alternative mode of the output pin to connect it to the PWM
	* module within the device. */
	gpio_enable_module_pin(EXAMPLE_PWM_PIN, EXAMPLE_PWM_FUNCTION);

	/* Initialize the PWM module. */
	pwm_init(&pwm_opt);

	/* Set channel configuration to channel 0. */
	pwm_channel_init(EXAMPLE_PWM_CHANNEL_ID, &pwm_channel);

	/* Start channel 0. */
	pwm_start_channels(1 << EXAMPLE_PWM_CHANNEL_ID);
}

/*! \brief Main function, application starts executing here after
*         initializing the stack pointer.
*/
int main(void)
{
	static const gpio_map_t SSC_GPIO_MAP =
	{
		{AVR32_SSC_TX_CLOCK_0_0_PIN,      AVR32_SSC_TX_CLOCK_0_0_FUNCTION     },
		{AVR32_SSC_TX_DATA_0_0_PIN,       AVR32_SSC_TX_DATA_0_0_FUNCTION      },
		{AVR32_SSC_TX_FRAME_SYNC_0_0_PIN, AVR32_SSC_TX_FRAME_SYNC_0_0_FUNCTION}
	};

	volatile avr32_ssc_t *ssc = &AVR32_SSC;

	// Switch main clock to external oscillator 0 (crystal).
	//pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);
	
	/*
	* The call to sysclk_init() will disable all non-vital
	* peripheral clocks, except for the peripheral clocks explicitly
	* enabled in conf_clock.h.
	*/
	sysclk_init();
	udc_start();
	PWM2Init();

	/* Initialize the USART module to print trace messages */
	init_dbg_rs232(sysclk_get_pba_hz());
	
	printf("Debug uart is working!");

	// Assign GPIO to SSC.
	gpio_enable_module(SSC_GPIO_MAP, sizeof(SSC_GPIO_MAP) / sizeof(SSC_GPIO_MAP[0]));

	// SSC init in I2S mode.
	ssc_i2s_init(ssc, 11025, 8, 8, SSC_I2S_MODE_STEREO_OUT, FOSC0);


	while (1)
	{

		while (!(ssc_i2s_get_status(ssc) & AVR32_SSC_SR_TXRDY_MASK));
		ssc_i2s_transfer(ssc, 0xA3);
	}
}
