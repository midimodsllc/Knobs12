#include "Knobs12.h"

Knobs12::Knobs12() {/*no constructer needed*/}

//turns all of the Knobs12's chip's peripherals ON
void Knobs12::systemInit() {
	InitPIO(); 		//configures all pins on the board to be controlled by the correct peripherals, sets input/output, etc 
	InitTC(); 		//configures Timer Counter to produce 4096*60Hz Clock signal on TIOA0 for LED Driver Greyscale Clock and toggle OEN lines at 60Hz	
	InitSPI();		//configures SPI peripheral to send data to LED drivers
	InitTWI();		//configures TWI to read/write to Encoder ICs
	resetLedValues();
	pushLedData();
	
}

//configures Input/Output for all pins on the SAM3X8C used by Knobs12
void Knobs12::InitPIO() {

	//PMC turns on clock for PIOA and PIOB
	REG_PMC_PCER0 |= (1 << ID_PIOA) | (1 << ID_PIOB);

	//PIO initialization for GSCLK

		//disable PIO control of GSCLK_PIN (enable's peripheral control of pin, aka TC0)
		REG_PIOB_PDR |= GSCLK_PIN;
		//AB peripheral select register (each pin has two peripherals it can interface with. PB25's "B" periph is TIOA0. 0 bit means use A periph, 1 bit means use B periph) 
		REG_PIOB_ABSR |= GSCLK_PIN;
		

	//PIO initialization for LED lines

		//enable PIO control on LED_POWER, OEN, XERR, and LATCH pins,
		REG_PIOA_PER |=  XERR_PIN;
		REG_PIOB_PER |=  LED_POWER_PIN
		| OEN1_PIN 
		| OEN2_PIN 
		| OEN3_PIN 
		| OEN_RGB_PIN
		| XERR_RGB_PIN 
		| LED_LATCH_PIN
		;

		//enable output mode on OEN and LD pins
		REG_PIOB_OER |= LED_POWER_PIN
		| OEN1_PIN 
		| OEN2_PIN 
		| OEN3_PIN 
		| OEN_RGB_PIN 
		| LED_LATCH_PIN
		;

		//enable input on XERR pins
		REG_PIOA_ODR |= XERR_PIN;
		REG_PIOB_ODR |= XERR_RGB_PIN;

		//XERR and XERR_RGB are inputs by default because they arent enabled as outputs above.

		//disable pull up resistors on OEN and LD outputs
		REG_PIOB_PUDR |= LED_POWER_PIN
		| OEN1_PIN 
		| OEN2_PIN 
		| OEN3_PIN 
		| OEN_RGB_PIN 
		| LED_LATCH_PIN
		;

		//power up LEDs and set LD pin to high (set output data register on PB18, PB5)
		REG_PIOB_SODR = LED_POWER_PIN;

	//PIO initialization for SPI lines (MISO,MOSI,SCLK)

		REG_PIOA_PDR |= MISO_PIN;		//MISO disable PIO control (enable SPI control)
		REG_PIOA_ODR |= MISO_PIN;    	//config MISO as Input
		REG_PIOA_ABSR &= ~MISO_PIN; 	//Peripheral A

		REG_PIOA_PDR |= MOSI_PIN;    	//MOSI disable PIO control (enable SPI control)
		REG_PIOA_OER |= MOSI_PIN;    	//MOSI config as Output
		REG_PIOA_ABSR &= ~MOSI_PIN; 	//Peripheral A

		REG_PIOA_PDR |= SPI_CLK_PIN;   	//SPCK disable PIO control (enable SPI control)
		REG_PIOA_OER |= SPI_CLK_PIN;   	//SPCK  as Output
		REG_PIOA_ABSR &= ~SPI_CLK_PIN; 	//Peripheral A

	//PIO config for button inputs

		REG_PIOB_PER |= BUTTON1_PIN | BUTTON2_PIN;
		REG_PIOA_PER |= BUTTON3_PIN | BUTTON4_PIN;

		REG_PIOB_ODR |= BUTTON1_PIN | BUTTON2_PIN;
		REG_PIOA_ODR |= BUTTON3_PIN | BUTTON4_PIN;

		REG_PIOB_PUDR |= BUTTON1_PIN | BUTTON2_PIN;
		REG_PIOB_PUDR |= BUTTON1_PIN | BUTTON2_PIN;

	//PIO initilization for TWI lines

		//config multiplexer reset pin (PROOFREAD)
		REG_PIOB_PER |= MUX_RESET_PIN;
		REG_PIOB_OER |= MUX_RESET_PIN;
		REG_PIOB_PUDR |= MUX_RESET_PIN;
		REG_PIOB_SODR |= MUX_RESET_PIN;

		REG_PIOB_PDR |= TWI_CLK_PIN | TWI_DATA_PIN;
		REG_PIOB_OER |= TWI_CLK_PIN | TWI_DATA_PIN;
		REG_PIOB_PUDR |= TWI_CLK_PIN | TWI_DATA_PIN;
		REG_PIOB_ABSR &= ~(TWI_CLK_PIN | TWI_DATA_PIN); //Peripheral A (TWI controls pins instead of PIO)

}

//Configures Timer Counter to produce GSCLK on TIOA0 and flip the OEN lines every greyscale counter cycle using TC1 and TC1's interrupt
void Knobs12::InitTC() {
        
	// Configure the PMC to enable the TC0 and TC1 modules.
	REG_PMC_PCER0 |= (1 << ID_TC0) | (1 << ID_TC1);

	// Disable TC clock
	REG_TC0_CCR0 = TC_CCR_CLKDIS;
	REG_TC0_CCR1 = TC_CCR_CLKDIS;
    
        //Disable interrupts on TC0 on channels 0/1
	REG_TC0_IDR0 = 0b11111111;
	REG_TC0_IDR1 = 0b11111111;

	// Clear status register
	REG_TC0_SR0;
	REG_TC0_SR1;
	
	REG_TC0_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1   //Counter uses TC Clock Source 1. It is Master Clock / 2
		| TC_CMR_WAVE                                          //enable waveform mode
		| TC_CMR_WAVSEL_UP_RC                                  //waveform selection: UP mode with automatic trigger on RC Compare
		| TC_CMR_ACPA_SET                                      //when counter hits compare register A, TIOA0 is set HIGH
		| TC_CMR_ACPC_CLEAR                                   //when counter hits compare register C, TIOA0 is set LOW
	;

	REG_TC0_CMR1 = TC_CMR_TCCLKS_TIMER_CLOCK1   	        //Counter uses TC External Clock Source 1. Use Block Mode Register (TC_BMR) to assign TIOA0 to XC1. 
		| TC_CMR_WAVE                                           //enable waveform mode
		| TC_CMR_WAVSEL_UP_RC                                   //waveform selection: UP mode with automatic trigger on RC Compare
	;

	//set the values to their respective registers
	REG_TC0_RC0 = TC0_RC0;
	REG_TC0_RA0 = TC0_RA0;
	REG_TC0_RC1 = TC0_RC1;

	// enable and restart the counters.
	REG_TC0_CCR0 = TC_CCR_CLKEN;
	REG_TC0_CCR1 = TC_CCR_CLKEN;
	REG_TC0_BCR = TC_BCR_SYNC; // resets all TC0 counters

	// Enable RC Compare interrupt on TC0 channel 1 
	REG_TC0_IDR1 = ~TC_IDR_CPCS;
	REG_TC0_IER1 = TC_IER_CPCS;
	NVIC_EnableIRQ(TC1_IRQn); 

}

void Knobs12::InitSPI() {

	//Enable clock for the SPI0 peripheral
	REG_PMC_PCER0 |= 1 << ID_SPI0;
	
	//Disable the SPI0 peripheral so we can configure it.
	REG_SPI0_CR = SPI_CR_SPIDIS;
	
	//Set as Master, Fixed Peripheral Select, Mode Fault Detection disabled and
	//Peripheral Chip Select is PCS = xxx0 NPCS[3:0] = 1110
	REG_SPI0_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;
	
	//config Chip Select Register
	REG_SPI0_CSR |= SPI_CSR_NCPHA 	//config clock phase
		| 0x00008000					//SPCK baudrate = MCK / SCBR = 84MHz / 128 = 656250Hz
		| SPI_CSR_BITS_12_BIT;					//BITS configs number of bits per transfer
	REG_SPI0_CSR &= ~SPI_CSR_CPOL;	//config clock polarity
	
	//disable SPI interrupts
	REG_SPI0_IDR = 0xFFFFFFFF;

	//Enable the SPI0 unit
	REG_SPI0_CR = SPI_CR_SPIEN;

}

/*
the TWI functions inside of this method can be found
in hardware/arduino/sam/system/libsam/source/twi.c
there is plenty of comments/documentation in that file
*/
void Knobs12::InitTWI() {

	//enable PMC to power TWI1
	REG_PMC_PCER0 |= 1 << ID_TWI1;

	TWI_ConfigureMaster(TWI1, TWI_CLK_FREQ, MCLK_FREQ);

	resetMuxes();

}

void Knobs12::resetMuxes() {
	//reset MUXs
	REG_PIOB_CODR |= MUX_RESET_PIN;
	delay(1);
	REG_PIOB_SODR |= MUX_RESET_PIN;
}

/*
the TWI functions inside of this method can be found
in hardware/arduino/sam/system/libsam/source/twi.c
there is plenty of comments/documentation in that file
*/
void Knobs12::readAllKnobs() {

	//deselect MUX1 channels
	REG_TWI1_MMR = TWI_MMR_DADR(MUX1_ADR) & ~(TWI_MMR_MREAD);
	REG_TWI1_THR = MUX_CH_SELECT_NONE;
	REG_TWI1_CR = TWI_CR_STOP;

	while( !TWI_ByteSent(TWI1) );
	while( !TWI_TransferComplete(TWI1) );

	//cycle through MUX0 channels and read/store knob angle values
	for (uint8_t i = 0; i < 8; i++) {

		uint16_t tempKnobMSB, tempKnobLSB;

		REG_TWI1_MMR = TWI_MMR_DADR(MUX0_ADR) & ~(TWI_MMR_MREAD);
		REG_TWI1_THR = MUX_CH_SELECT(i);
		REG_TWI1_CR = TWI_CR_STOP;

		while( !TWI_ByteSent(TWI1) );
		while( !TWI_TransferComplete(TWI1) );

		REG_TWI1_MMR = TWI_MMR_DADR(ENCDR_ADR) | TWI_MMR_MREAD | TWI_MMR_IADRSZ_1_BYTE;
		REG_TWI1_IADR = ANG_MSB_IADR;
		REG_TWI1_CR = TWI_CR_START | TWI_CR_STOP;

		while( !TWI_ByteReceived(TWI1) );

		tempKnobMSB = TWI_ReadByte(TWI1);

		while( !TWI_TransferComplete(TWI1) );

		REG_TWI1_MMR = TWI_MMR_DADR(ENCDR_ADR) | TWI_MMR_MREAD | TWI_MMR_IADRSZ_1_BYTE;
		REG_TWI1_IADR = ANG_LSB_IADR;
		REG_TWI1_CR = TWI_CR_START | TWI_CR_STOP;

		while( !TWI_ByteReceived(TWI1) );

		tempKnobLSB = TWI_ReadByte(TWI1);

		while( !TWI_TransferComplete(TWI1) );

		knobs12.knobPositionValues[i] = 0x0FFF & ((tempKnobMSB << 8) | tempKnobLSB);

	}

	//deselect MUX0 channels
	REG_TWI1_MMR = TWI_MMR_DADR(MUX0_ADR) & ~(TWI_MMR_MREAD);
	REG_TWI1_THR = MUX_CH_SELECT_NONE;
	REG_TWI1_CR = TWI_CR_STOP;

	while( !TWI_ByteSent(TWI1) );
	while( !TWI_TransferComplete(TWI1) );

	//cycle through MUX1 channels and read/store knob angle values
	for (uint8_t i = 0; i < 4; i++) {

		uint16_t tempKnobMSB, tempKnobLSB;

		REG_TWI1_MMR = TWI_MMR_DADR(MUX1_ADR) & ~(TWI_MMR_MREAD);
		REG_TWI1_THR = MUX_CH_SELECT(i);
		REG_TWI1_CR = TWI_CR_STOP;

		while( !TWI_ByteSent(TWI1) );
		while( !TWI_TransferComplete(TWI1) );

		REG_TWI1_MMR = TWI_MMR_DADR(ENCDR_ADR) | TWI_MMR_MREAD | TWI_MMR_IADRSZ_1_BYTE;
		REG_TWI1_IADR = ANG_MSB_IADR;
		REG_TWI1_CR = TWI_CR_START | TWI_CR_STOP;

		while( !TWI_ByteReceived(TWI1) );

		tempKnobMSB = TWI_ReadByte(TWI1);

		while( !TWI_TransferComplete(TWI1) );

		REG_TWI1_MMR = TWI_MMR_DADR(ENCDR_ADR) | TWI_MMR_MREAD | TWI_MMR_IADRSZ_1_BYTE;
		REG_TWI1_IADR = ANG_LSB_IADR;
		REG_TWI1_CR = TWI_CR_START | TWI_CR_STOP;

		while( !TWI_ByteReceived(TWI1) );

		tempKnobLSB = TWI_ReadByte(TWI1);

		while( !TWI_TransferComplete(TWI1) );

		knobs12.knobPositionValues[i + 8] = 0x0FFF & ((tempKnobMSB << 8) | tempKnobLSB);

	}

}

uint16_t Knobs12::getKnobPosition(uint16_t knobNum) {

	return knobPositionValues[knobNum];

}

// uint16_t Knobs12::readKnob(uint8_t knobNum) {

// }

uint8_t Knobs12::checkButtonState(uint16_t buttonNumber) {

	if (buttonNumber == 1) {
		if ( (REG_PIOB_PDSR & BUTTON1_PIN) != 0) {
			return 1;
		} else {
			return 0;
		}
	}
	if (buttonNumber == 2) {
		if ( (REG_PIOB_PDSR & BUTTON2_PIN) != 0) {
			return 1;
		} else {
			return 0;
		}
	}
	if (buttonNumber == 3) {
		if ( (REG_PIOA_PDSR & BUTTON3_PIN) != 0 ) {
			return 1;
		} else {
			return 0;
		}
	}
	if (buttonNumber == 4) {
		if ((REG_PIOA_PDSR & BUTTON4_PIN) != 0) {
			return 1;
		} else {
			return 0;
		}
	}

}

//Transfers new data to the LED Drivers using the SPI0 peripheral
void Knobs12::pushLedData() {

	//latch pin goes low
	REG_PIOB_CODR = LED_LATCH_PIN;

	//send the ledBrightnessValues array in backwards

	for ( int i = 12; i >= 0; i--) {

		for ( int j = 15; j >= 0; j--) {

			//Wait for previous transfer to complete
			while ((REG_SPI0_SR & SPI_SR_TXEMPTY) == 0);

			//load the Transmit Data Register with the value to transmit
			REG_SPI0_TDR = ledBrightnessValues[i][j]; 

			//Wait for data to be transferred to serializer
			while ((REG_SPI0_SR & SPI_SR_TDRE) == 0);

		}

	}
	
	delayMicroseconds(20);

	//latch data
	REG_PIOB_SODR = LED_LATCH_PIN;

}

void Knobs12::changeLedBrightness(uint16_t knobNum, uint16_t ledNum, uint16_t ledBrightness) {
	
	ledBrightnessValues[knobNum][ledNum] = ledBrightness;
	
}

void Knobs12::changeButtonColor(uint8_t buttonNum, uint16_t red, uint16_t grn, uint16_t blu) {

	if (buttonNum == 0) {

		ledBrightnessValues[12][0] = red;
		ledBrightnessValues[12][1] = blu;
		ledBrightnessValues[12][2] = grn;
		
	} else if (buttonNum == 1) {

		ledBrightnessValues[12][3] = red;
		ledBrightnessValues[12][4] = blu;
		ledBrightnessValues[12][5] = grn;
		
	} else if (buttonNum == 2) {

		ledBrightnessValues[12][6] = red;
		ledBrightnessValues[12][7] = blu;
		ledBrightnessValues[12][8] = grn;
		
	} else if (buttonNum == 3) {

		ledBrightnessValues[12][9]  = red;
		ledBrightnessValues[12][10] = blu;
		ledBrightnessValues[12][11] = grn;

	}

}

void Knobs12::resetLedValues() {
	
	//fill LED array with zeros
	for (uint8_t i = 0; i < 13; i++) {
		for (uint8_t j = 0; j < 16; j++) {
			ledBrightnessValues[i][j] = 0x0000;
		}
	}
	
}

void Knobs12::setAllLedsOn() {

	//fill LED array with max brightness for all LEDs
	for (uint8_t i = 0; i < 13; i++) {
		for (uint8_t j = 0; j < 16; j++) {
			ledBrightnessValues[i][j] = 0x0FFF;
		}
	}

}

//runs every time TC0 channel 1 reaches 4096 which should be about 60 times per second
void TC1_Handler() {
	
	//reading TC status is necessary here or else you get infinite interrupts (or so I read)
	//TC_GetStatus(TC0, 1);
	REG_TC0_SR1;

	//toggle the OEN lines (they only need to be high for >20ns)

	//set OENs high
		REG_PIOB_SODR = OEN1_PIN 
		| OEN2_PIN 
		| OEN3_PIN 
		| OEN_RGB_PIN;

	delayMicroseconds(1);

		//Set OENs low
		REG_PIOB_CODR = OEN1_PIN 
		| OEN2_PIN 
		| OEN3_PIN 
		| OEN_RGB_PIN;

}

Knobs12 knobs12;