/*
	Knobs12.h - Library for Midimods Knobs12 human interface device running on SAM3X8C microcontroller.
	Created by Midimods LLC, 8 October 2015.
	
	Contributions by 	
		Phil Manofsky
		Doug Manofsky
		Matthew Sherwood
*/

#ifndef KNOBS12_H
#define KNOBS12_H

#include "Arduino.h"

//Pin Definitions for KNOBS12 Board (and other device definitions)

//LED Driver Pins
#define GSCLK_PIN       	PIO_PB25
#define LED_POWER_PIN   	PIO_PB18
#define XERR_PIN        	PIO_PA21
#define XERR_RGB_PIN    	PIO_PB6
#define OEN1_PIN        	PIO_PB0
#define OEN2_PIN        	PIO_PB1
#define OEN3_PIN        	PIO_PB2
#define OEN_RGB_PIN     	PIO_PB4
#define LED_LATCH_PIN   	PIO_PB5

//Button Pins
#define BUTTON1_PIN			PIO_PB15
#define BUTTON2_PIN			PIO_PB16
#define BUTTON3_PIN			PIO_PA16
#define BUTTON4_PIN			PIO_PA24

//SPI pins
#define MISO_PIN        	PIO_PA25
#define MOSI_PIN			PIO_PA26
#define SPI_CLK_PIN			PIO_PA27

//TWI pins and constants
#define MUX_RESET_PIN				PIO_PB17
#define TWI_CLK_PIN					PIO_PB13
#define TWI_DATA_PIN				PIO_PB12
#define MUX0_ADR					0x70		//i2c address for first multiplexer
#define MUX1_ADR					0x71		// "     "     "  second     " 
#define ENCDR_ADR					0x36		//i2c address for all AS5600 encoder ICs
#define ANG_MSB_IADR				0x0C		//internal register address for Raw Angle (bits 8:11)
#define ANG_LSB_IADR				0x0D		//internal register address for Raw Angle (bits 0:7)
#define MUX_CH_SELECT(value)		value + 0x08		//data byte sent to MUX0/1 to select Channel #value
#define MUX_CH_SELECT_NONE			0x00		//data byte sent to MUX0/1 to deselect ALL channels
#define TWI_CLK_FREQ				100000		//TWI clock set to 100KHz

//More Constants
#define MCLK_FREQ 				84000000		//Master Clock frequency is 84MHz
#define TC_WAVEFORM_DIVISOR 	2 				//because we are using Timer Clock Source 1 (Master Clock/2)
#define TC_WAVEFORM_FREQUENCY 	245760    		//Greyscale Clock Frequency for LED drivers (4096 * 60Hz)
#define GSCLK_CYCLE_LENGTH 		4096			//used for TC1 channel 0 compare value to toggle OEN lines every single greyscale counter cycle
#define TC0_RC0					MCLK_FREQ/TC_WAVEFORM_DIVISOR/TC_WAVEFORM_FREQUENCY
#define TC0_RA0					TC0_RC0/2
#define TC0_RC1					TC0_RC0*GSCLK_CYCLE_LENGTH

//Color definitions
#define AQUA		0x0647, 0x0B1B, 0x061A 		//0b011001000111, 0b101100011011, 0b011000011010
#define LILAC		0x0E85,	0x04BD, 0x0B88		//0b111010000101, 0b010010111101, 0b101110001000
#define PEACH		0x0F73,	0x05F4, 0x024C 		//0b111101110011, 0b010111110100, 0b001001001100	
#define	MINT		0x04DF, 0x0E95,	0x01EB		//0b010011011111, 0b111010010101, 0b000111101011
// #define
// #define
// #define
// #define
// #define
// #define


class Knobs12 
{
	public:
		Knobs12();
		void systemInit();
		void readAllKnobs();
		uint16_t getKnobPosition(uint16_t knobNum);
		void resetMuxes();
		void changeLedBrightness(uint16_t knobNum, uint16_t ledNum, uint16_t ledBrightness);
		void changeButtonColor(uint8_t buttonNum, uint16_t red, uint16_t grn, uint16_t blu);
		void pushLedData();
		void resetLedValues();
		void setAllLedsOn();
		void readAllButtons();
		uint8_t checkButtonState(uint16_t buttonNumber);
		uint16_t ledBrightnessValues[13][16];
		uint16_t knobPositionValues[12];

	private:
		void InitSPI();
		void InitPIO();
		void InitTC();
		void InitTWI();
		
};

extern Knobs12 knobs12; //create Knobs12 instance of Knobs12 class

#endif