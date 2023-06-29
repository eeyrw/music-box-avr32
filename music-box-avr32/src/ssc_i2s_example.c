#include <avr32/io.h>
#include "pm.h"
#include "gpio.h"
#include "ssc_i2s.h"
#include "user_board.h"


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
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

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
