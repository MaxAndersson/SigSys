/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>

static usart_options_t usart_opt = {
	.baudrate = 57000,
	.channelmode = USART_NORMAL_CHMODE,
	.charlength = 8,
	.paritytype = USART_NO_PARITY,
	.stopbits = USART_1_STOPBIT,
	
};

void module_start(){
	sysclk_init();
	pm_switch_to_osc0(&AVR32_PM,FOSCO,OSC_STARTUP_0);
	gpio_enable_module_pin(USART_TXD_PIN,USART_TXD_FUNCTION);
	gpio_enable_module_pin(USART_RXD_PIN,USART_RXD_FUNCTION);
	usart_init_rs232(USART,&usart_opt,FOSCO);
}

void usart_message(char* message){
	usart_write_line(USART,message);
}




int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).

	board_init();
	module_start();
	
	while (1)
	{
		if (counter >= ADC_FREQ) // control the time when the next Ext_Trigger is valid
		{                // counter = the number of cycles; "counter = ADC_FREQ" corresponds to the time of 1 second
			// "counter = ADC_FREQ/1000" corresponds to 1 ms
			gpio_enable_pin_interrupt(Ext_Trigger_Pin,GPIO_RISING_EDGE);


		}
	

	// Insert application code here, after the board has been initialized.
}
