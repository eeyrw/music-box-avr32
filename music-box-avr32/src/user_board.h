/*****************************************************************************
 *
 * \file
 *
 * \brief AT32UC3B MUSIC_BOX board header file.

 */


#ifndef _MUSIC_BOX_H_
#define _MUSIC_BOX_H_

#include "compiler.h"



/*! \name Oscillator Definitions
 */
//! @{

#define FOSC32          32768                                 //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   AVR32_PM_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.

#define FOSC0           8000000                              //!< Osc0 frequency: Hz.
#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.


// Osc1 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc1 crystal is mounted on your board.
//#define FOSC1           12000000                              //!< Osc1 frequency: Hz.
//#define OSC1_STARTUP    AVR32_PM_OSCCTRL1_STARTUP_2048_RCOSC  //!< Osc1 startup time: RCOsc periods.

//! @}

/* These are documented in services/basic/clock/uc3b0_b1/osc.h */
#define BOARD_OSC0_HZ           8000000
#define BOARD_OSC0_STARTUP_US   17000
#define BOARD_OSC0_IS_XTAL      true
#define BOARD_OSC32_HZ          32768
#define BOARD_OSC32_STARTUP_US  71000
#define BOARD_OSC32_IS_XTAL     true

/*! \name USB Definitions
 */
//! @{

//! Multiplexed pin used for USB_ID: AVR32_USBB_USB_ID_x_x.
//! To be selected according to the AVR32_USBB_USB_ID_x_x_PIN and
//! AVR32_USBB_USB_ID_x_x_FUNCTION definitions from <avr32/uc3bxxxx.h>.
#define USB_ID                      AVR32_USBB_USB_ID_0_0

//! Multiplexed pin used for USB_VBOF: AVR32_USBB_USB_VBOF_x_x.
//! To be selected according to the AVR32_USBB_USB_VBOF_x_x_PIN and
//! AVR32_USBB_USB_VBOF_x_x_FUNCTION definitions from <avr32/uc3bxxxx.h>.
#define USB_VBOF                    AVR32_USBB_USB_VBOF_0_0

//! Active level of the USB_VBOF output pin.
#define USB_VBOF_ACTIVE_LEVEL       LOW

//! USB overcurrent detection pin.
#define USB_OVERCURRENT_DETECT_PIN  AVR32_PIN_PA20

//! @}


#define DBG_USART               (&AVR32_USART1)
#define DBG_USART_RX_PIN        AVR32_USART1_RXD_0_0_PIN
#define DBG_USART_RX_FUNCTION   AVR32_USART1_RXD_0_0_FUNCTION
#define DBG_USART_TX_PIN        AVR32_USART1_TXD_0_0_PIN
#define DBG_USART_TX_FUNCTION   AVR32_USART1_TXD_0_0_FUNCTION
#define DBG_USART_IRQ           AVR32_USART1_IRQ
#define DBG_USART_BAUDRATE      57600




/*! \name GPIO Connections of Push Buttons
 */
//! @{
#define GPIO_PUSH_BUTTON_0            AVR32_PIN_PB02
#define GPIO_PUSH_BUTTON_0_PRESSED    0
#define GPIO_PUSH_BUTTON_1            AVR32_PIN_PB03
#define GPIO_PUSH_BUTTON_1_PRESSED    0
//! @}





/*! \name PWM Connections of Audio
 */
//! @{
#define AUDIO_LOW_PWM_CHANNEL       5
#define AUDIO_LOW_PWM_PIN           AVR32_PWM_5_0_PIN
#define AUDIO_LOW_PWM_FUNCTION      AVR32_PWM_5_0_FUNCTION
#define AUDIO_HIGH_PWM_CHANNEL      6
#define AUDIO_HIGH_PWM_PIN          AVR32_PWM_6_1_PIN
#define AUDIO_HIGH_PWM_FUNCTION     AVR32_PWM_6_1_FUNCTION
//! @}



#define CODEC_SD_PIN				AVR32_PIN_PB04

/*! \name SPI Connections of the AT45DBX Data Flash Memory
 */
//! @{
#define AT45DBX_SPI                 (&AVR32_SPI)
#define AT45DBX_SPI_NPCS            0
#define AT45DBX_SPI_SCK_PIN         AVR32_SPI_SCK_0_0_PIN
#define AT45DBX_SPI_SCK_FUNCTION    AVR32_SPI_SCK_0_0_FUNCTION
#define AT45DBX_SPI_MISO_PIN        AVR32_SPI_MISO_0_0_PIN
#define AT45DBX_SPI_MISO_FUNCTION   AVR32_SPI_MISO_0_0_FUNCTION
#define AT45DBX_SPI_MOSI_PIN        AVR32_SPI_MOSI_0_0_PIN
#define AT45DBX_SPI_MOSI_FUNCTION   AVR32_SPI_MOSI_0_0_FUNCTION
#define AT45DBX_SPI_NPCS0_PIN       AVR32_SPI_NPCS_0_0_PIN
#define AT45DBX_SPI_NPCS0_FUNCTION  AVR32_SPI_NPCS_0_0_FUNCTION
//! @}


/*! \name GPIO and SPI Connections of the SD/MMC Connector
 */
//! @{
#define SD_MMC_SPI_MEM_CNT          1
#define SD_MMC_0_CD_GPIO            AVR32_PIN_PB00
#define SD_MMC_0_CD_DETECT_VALUE    1
#define SD_MMC_0_WP_GPIO            AVR32_PIN_PB01
#define SD_MMC_0_WP_DETECT_VALUE    0
#define SD_MMC_SPI_0_CS             1

// Keep it for SD MMC stack ASF V1.7
#define SD_MMC_CARD_DETECT_PIN      SD_MMC_0_CD_GPIO
#define SD_MMC_WRITE_PROTECT_PIN    SD_MMC_0_WP_GPIO
#define SD_MMC_SPI_NPCS             SD_MMC_SPI_0_CS

#define SD_MMC_SPI                  (&AVR32_SPI)
#define SD_MMC_SPI_SCK_PIN          AVR32_SPI_SCK_0_0_PIN
#define SD_MMC_SPI_SCK_FUNCTION     AVR32_SPI_SCK_0_0_FUNCTION
#define SD_MMC_SPI_MISO_PIN         AVR32_SPI_MISO_0_0_PIN
#define SD_MMC_SPI_MISO_FUNCTION    AVR32_SPI_MISO_0_0_FUNCTION
#define SD_MMC_SPI_MOSI_PIN         AVR32_SPI_MOSI_0_0_PIN
#define SD_MMC_SPI_MOSI_FUNCTION    AVR32_SPI_MOSI_0_0_FUNCTION
#define SD_MMC_SPI_NPCS_PIN         AVR32_SPI_NPCS_1_0_PIN
#define SD_MMC_SPI_NPCS_FUNCTION    AVR32_SPI_NPCS_1_0_FUNCTION
//! @}

/*! \name USART connection to the UC3B board controller
 */
//! @{
#define USART                       (&AVR32_USART1)
#define USART_RXD_PIN               AVR32_USART1_RXD_0_0_PIN
#define USART_RXD_FUNCTION          AVR32_USART1_RXD_0_0_FUNCTION
#define USART_TXD_PIN               AVR32_USART1_TXD_0_0_PIN
#define USART_TXD_FUNCTION          AVR32_USART1_TXD_0_0_FUNCTION
#define USART_IRQ                   AVR32_USART1_IRQ
#define USART_IRQ_GROUP             AVR32_USART1_IRQ_GROUP
#define USART_SYSCLK                SYSCLK_USART1
//! @}

/*! \name TWI Connections of the Spare TWI Connector
 */
//! @{
#define SPARE_TWI                   (&AVR32_TWI)
#define SPARE_TWI_SCL_PIN           AVR32_TWI_SCL_0_0_PIN
#define SPARE_TWI_SCL_FUNCTION      AVR32_TWI_SCL_0_0_FUNCTION
#define SPARE_TWI_SDA_PIN           AVR32_TWI_SDA_0_0_PIN
#define SPARE_TWI_SDA_FUNCTION      AVR32_TWI_SDA_0_0_FUNCTION
//! @}


/*! \name SPI Connections of the Spare SPI Connector
 */
//! @{
#define SPARE_SPI                   (&AVR32_SPI)
#define SPARE_SPI_NPCS              2
#define SPARE_SPI_SCK_PIN           AVR32_SPI_SCK_0_0_PIN
#define SPARE_SPI_SCK_FUNCTION      AVR32_SPI_SCK_0_0_FUNCTION
#define SPARE_SPI_MISO_PIN          AVR32_SPI_MISO_0_0_PIN
#define SPARE_SPI_MISO_FUNCTION     AVR32_SPI_MISO_0_0_FUNCTION
#define SPARE_SPI_MOSI_PIN          AVR32_SPI_MOSI_0_0_PIN
#define SPARE_SPI_MOSI_FUNCTION     AVR32_SPI_MOSI_0_0_FUNCTION
#define SPARE_SPI_NPCS_PIN          AVR32_SPI_NPCS_2_0_PIN
#define SPARE_SPI_NPCS_FUNCTION     AVR32_SPI_NPCS_2_0_FUNCTION
//! @}


#endif  // _MUSIC_BOX_H_
