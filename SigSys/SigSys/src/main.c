
/*********************************************************************************
 * This project is designed for the course Embedded Signal Processing Systems.
 * The program provides a software framework that has implemented 2xADC, 2xDAC,  
 * an external trigger and an USART. Therefore, one can 
 *   - access the digital data from ADCs,  
 *   - develop algorithms, or build digital systems to process signals/data in real time 
 *   - send processed signals/data out to DACs and get analog signals
 *   - send data from the MCU to PCs via USART
 * 
 * The configurations for the above functions are the following
 *   - J2's ADC1 and ADC5 are configured as two ADC channels: ADC0 and ADC1
 *   - J4's SCK and J2's ADC2 are defined as two DAC channels: DAC0 and DAC1
 *   - J4's SCL is configured as external trigger
 *   - J4's TX is for USART
 * 
 * When this project is running, the output can be obtained ONLY when the external 
 * trigger is activated (which can be made just by touching your finger on J4's SCL).
 * A controller (using "counter") is available to control the time instant 
 * when the next external trigger is enabled
 *
 * When programming for DSP applications, the starting point is "DO DSP HERE" in main(void).
 * You get to it via Ctrl+F (Find) 
 *
 * Copyright (c) 2014 Ping Wu. All rights reserved.
 ********************************************************************************/

/* Include .h files here for all the drivers imported from ASF (Atmel Software Framework) */
#include <asf.h>

#define FALSE               0
#define U_CPU_F             60000000        // CPU Freq
#define U_OSC0_F            12000000        // OSC0's Freq
#define U_PBA_F             30000000        // PBA Freq
#define U_OSC0_START        3               // OSC0 start up time
#define PDCA_CHANNEL_ADC    0               // DMA Channel 0 is for ADC
#define PDCA_CHANNEL_DAC    1               // DAC
#define Buffer_Size         4096            // DSP Buffer Size which must be a number that is 4^n 
#define DATA_SIZE			1024			// DSP FFT DATA SIZE = 2^N_of_bits
#define N_of_bits			10				// FFT orders
#define TC                  (&AVR32_TC0)	// Timer/Counter
#define TC_CHANNEL          0				// Timer Channel 0
#define PROCESSING          0				// If DSP_STATE == PROCESSING, the board is doing DSP 
#define LOADING				1				// If DSP_STATE == LAODING, the board is loading data from ADC
#define STANDSTILL          2				// If DSP_STATE == STANDSTILL, the board is standing still
#define ADC_FREQUENCY       FOSC0/256
#define Ext_Trigger_Pin  AVR32_PIN_PA15		// Pin J4's SCL is defined as Ext trigger

static volatile uint32_t trigger=0;
static volatile uint16_t DSP_STATE = LOADING;
static void User_Usart_int2str(uint32_t User_int32);
static avr32_tc_t *tc=TC;
/************************************************************************/
/* Declare PDCA buffers for ADC and DAC, and buffers for DSP                                                    */
/************************************************************************/
static volatile uint16_t ADC_PDCA_BUFFER1[Buffer_Size*2];
static volatile uint16_t DAC_PDCA_BUFFER1[Buffer_Size*2];
//DSP BUFFERS
static volatile uint16_t  DSP_REAL_BUFFER[Buffer_Size];
static volatile dsp16_complex_t DSP_COMP_BUFFER[Buffer_Size];

/************************************************************************/

//Declare pointers
static uint16_t *DAC_BUFFER_PTR   =&DAC_PDCA_BUFFER1;
static uint16_t *DSP_BUFFER_PTR   =&DSP_REAL_BUFFER;
//Timer Tick
static volatile uint32_t tc_tick=0;
static volatile uint32_t counter = 0 ;
/************************************************************************/
/*                            New Things For This                       */
/************************************************************************/


/************************************************************************/

/************************************************************************/
/*                             PDCA Options                             */
/************************************************************************/

static const pdca_channel_options_t ADC_PDCA_OPT = {
    /** Memory address. */
    .addr = (void *)ADC_PDCA_BUFFER1,
    /** Select peripheral ID. */
    .pid = AVR32_PDCA_PID_ADC_RX,
    /** Transfer counter. */
    .size = Buffer_Size*2,
    /** Next memory address. */
    .r_addr = NULL,
    /** Next transfer counter. */
    .r_size = NULL,
    /** Select the size of the transfer (byte, half-word or word). */
    .transfer_size = PDCA_TRANSFER_SIZE_HALF_WORD,
};
//DMA DAC options
static const pdca_channel_options_t DAC_PDCA_OPT = {
    .addr =(void *)DAC_PDCA_BUFFER1,
    .pid = AVR32_PDCA_PID_ABDAC_TX,
    .size = Buffer_Size,
    .r_addr = NULL,
    .r_size = NULL,
    .transfer_size = PDCA_TRANSFER_SIZE_WORD,
};


/************************************************************************/
/*                              INTERRUPTS                              */
/************************************************************************/

/*----------------------------------------------------------------------*/
/* PDCA Reload Interrupt setting
 *  \brief Set PDCA channel reload values
 *
 * \param pdca_ch_number PDCA channel
 * \param addr           address where data to load are stored
 * \param size           size of the data block to load
*/
/*----------------------------------------------------------------------*/
__attribute__((__interrupt__))
ISR(pdca_adc_handler, AVR32_PDCA_IRQ_0, 0)
{
        pdca_reload_channel(PDCA_CHANNEL_ADC,
                (void *)ADC_PDCA_BUFFER1, Buffer_Size*2);
}
ISR(pdca_dac_handler, AVR32_PDCA_IRQ_1, 1)
{
        pdca_reload_channel(PDCA_CHANNEL_DAC,
                DAC_BUFFER_PTR, Buffer_Size);
}

/*----------------------------------------------------------------------*/
/*   Timer interrupt                                                    */
/*----------------------------------------------------------------------*/
ISR(tc_irq,AVR32_TC0_IRQ0,AVR32_INTC_INT3)
{
    tc_read_sr(TC, TC_CHANNEL);
    //convert 10-bit ADC data to 16 bits & then transfer to DAC & DSP buffers
    DAC_BUFFER_PTR[2*tc_tick]  = ADC_PDCA_BUFFER1[2*tc_tick+1]<<6^0x8000; //ADC CH1 to DAC CH1
    DAC_BUFFER_PTR[2*tc_tick+1]= ADC_PDCA_BUFFER1[2*tc_tick+1]<<6^0x8000; //ADC CH5 to DAC CH0
    tc_tick++;
    if (tc_tick >= Buffer_Size) tc_tick =0;
}
/*----------------------------------------------------------------------*/
/*   EXTERNAL TRIGGER INTERRUPT
     Use for One Shot DSP 
     Reset DMAs and set DSP state to LOADING and timer tick
*/
/*----------------------------------------------------------------------*/
ISR(User_Ext_Trigger_Isr,AVR32_GPIO_IRQ_1,AVR32_INTC_INT0)
{
	gpio_toggle_pin(LED1_GPIO);
	DSP_STATE = LOADING;
	pdca_enable(PDCA_CHANNEL_ADC);	//reset DMA
	pdca_enable(PDCA_CHANNEL_DAC);	//reset DMA
	tc_tick=0;						//reset index
	counter =0;						//reset counter
	gpio_clear_pin_interrupt_flag(Ext_Trigger_Pin);
	gpio_disable_pin_interrupt(Ext_Trigger_Pin);
}

/*----------------------------------------------------------------------*/
/*   ADC TO DSP TO DAC INTERRUPT
        Both Continuously and One shot 
*/
/*---------------------------------------------------------------------*/
ISR(tc_irq_dsp,AVR32_TC0_IRQ0,AVR32_INTC_INT3)
{
	tc_read_sr(TC, TC_CHANNEL);
	//put the ADC data to DAC and DSP buffers
	switch (DSP_STATE)
	{
		case LOADING :		// load the data of "Buffer_Size" to the DSP buffer
			DSP_BUFFER_PTR[tc_tick]=(ADC_PDCA_BUFFER1[2*tc_tick]<<6)/2;
			break;
		case PROCESSING:
			gpio_toggle_pin(LED0_GPIO);
			break;
		case STANDSTILL:
			gpio_toggle_pin(LED3_GPIO);
		break;
	}
	tc_tick++;
	counter++;
	if (tc_tick >= Buffer_Size)
	{	
		tc_tick =0;
		if ( DSP_STATE == LOADING)  DSP_STATE = PROCESSING;	
		gpio_toggle_pin(LED2_GPIO);
	}
}
/************************INTERRUPTS**************************************/


/************************************************************************/
/*                            SUB FUNCTIONS                             */
/************************************************************************/

/*----------------------------------------------------------------------*/
/*  convert int to ascii,  e.g., 1 => '0x31'                            */
/*----------------------------------------------------------------------*/
static uint16_t User_Binbcd(uint16_t User_Bin)
{
    User_Bin=User_Bin&0x0f;
    User_Bin=User_Bin+0x30;
    return User_Bin;
}

/*----------------------------------------------------------------------*/
/*  Convert a number to ascii array and put to the usart buffer end with ','
    e.g., 123 => {0x30,0x30,0x31,0x32,0x33,','}              */
/*----------------------------------------------------------------------*/
static void User_Usart_int2str(uint32_t User_int32)
{
		usart_putchar(USART,User_Binbcd( User_int32/10000)) ;
		usart_putchar(USART,User_Binbcd( User_int32%10000/1000)) ;
		usart_putchar(USART,User_Binbcd( User_int32%1000/100)) ;
		usart_putchar(USART,User_Binbcd( User_int32%100/10)) ;
		usart_putchar(USART,User_Binbcd( User_int32%10)) ;
		
}

/*----------------------------------------------------------------------*/
/*    Timer inits and interrupt options                                 */
/*----------------------------------------------------------------------*/
static void tc_init(volatile avr32_tc_t *tc,uint32_t ADC_FREQ)
{
    // Options for waveform generation.
    static const tc_waveform_opt_t waveform_opt = {
        // Channel selection.
        .channel  = TC_CHANNEL,
        // Software trigger effect on TIOB.
        .bswtrg   = TC_EVT_EFFECT_NOOP,
        // External event effect on TIOB.
        .beevt    = TC_EVT_EFFECT_NOOP,
        // RC compare effect on TIOB.
        .bcpc     = TC_EVT_EFFECT_SET,
        // RB compare effect on TIOB.
        .bcpb     = TC_EVT_EFFECT_CLEAR,
        // Software trigger effect on TIOA.
        .aswtrg   = TC_EVT_EFFECT_NOOP,
        // External event effect on TIOA.
        .aeevt    = TC_EVT_EFFECT_NOOP,
        // RC compare effect on TIOA.
        .acpc     = TC_EVT_EFFECT_SET ,
        /* RA compare effect on TIOA. */
        .acpa     = TC_EVT_EFFECT_CLEAR,
        /* Waveform selection: Up mode with automatic trigger(reset)
         * on RC compare.*/
        .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
        // External event trigger enable.
        .enetrg   = false,
        // External event selection.
        .eevt     = 0,
        // External event edge selection.
        .eevtedg  = TC_SEL_NO_EDGE,
        // Counter disable when RC compare.
        .cpcdis   = false,
        // Counter clock stopped with RC compare.
        .cpcstop  = false,
        // Burst signal selection.
        .burst    = false,
        // Clock inversion.
        .clki     = false,
        // Internal source clock 3, connected to fPBA / 8.
        .tcclks   = TC_CLOCK_SOURCE_TC3
    };

    // Options for enabling TC interrupts
    static const tc_interrupt_t tc_interrupt = {
        .etrgs = 0,
        .ldrbs = 0,
        .ldras = 0,
        .cpcs  = 1, // Enable interrupt on RC compare alone
        .cpbs  = 0,
        .cpas  = 0,
        .lovrs = 0,
        .covfs = 0
    };
    // Initialize the timer/counter (TC).
    tc_init_waveform(tc, &waveform_opt);
    /*set freq for both Rc and Ra. RC freq = ADC freq            */
    tc_write_rc(tc, TC_CHANNEL, (U_PBA_F / 8 /ADC_FREQ));
    tc_write_ra(tc, TC_CHANNEL, (U_PBA_F/ 8 / 90000));
    // configure the timer interrupt
    tc_configure_interrupts(tc,TC_CHANNEL, &tc_interrupt);
    // Start the timer/counter.
    tc_start(tc, TC_CHANNEL);
}

/*----------------------------------------------------------------------*/
/*               Interrupt initiate, register and enable.               */
/*----------------------------------------------------------------------*/
static void pdca_set_irq(void)
{
	#if __GNUC__
	    INTC_init_interrupts();

    //All PDCA interrupts need to be registered here
	    INTC_register_interrupt(&pdca_adc_handler, AVR32_PDCA_IRQ_0,
		        AVR32_INTC_INT0);
		INTC_register_interrupt(&pdca_dac_handler, AVR32_PDCA_IRQ_1,
			    AVR32_INTC_INT0);
		INTC_register_interrupt(&User_Ext_Trigger_Isr,AVR32_GPIO_IRQ_1,AVR32_INTC_INT3);
		INTC_register_interrupt(&tc_irq_dsp,AVR32_TC0_IRQ0,AVR32_INTC_INT2);
	#endif

    /* Enable all interrupt/exception. */
    Enable_global_interrupt();
}
static int max_L(dsp16_t *vect1,int start, int end){
	int a = 0;
	int b = 0;
	for(int i=start; i<end; i++){
		if(vect1[i] >= a){
		a = vect1[i];
		//User_Usart_int2str(a);
		//	usart_write_line(USART,"m ");
		//	User_Usart_int2str(i);
		//		usart_write_line(USART,"i ");
		b = i;
		}
	}
	return b;
}

static void convertFreq(int low, int high) {
	char *symbol = "123456789*0#";
	int lfg[4] = {abs(697-low),abs(770-low),abs(852-low),abs(941-low)};
	int hfg[3] = {abs(1209-high),abs(1336-high),abs(1447-high)};
	int lowNum = 0;
	int highNum = 0;

	for (int a=1; a<4; a++)
	{
		if(lfg[a] < lfg[lowNum]){
			lowNum = a;
		}
	}
	for (int a=1; a<3; a++)
	{
		if(hfg[a] < hfg[highNum]){
			highNum = a;
		}
	}
	int symbolValue = highNum + (3 * lowNum);
	usart_putchar(USART,symbol[symbolValue]);
}
/************************* End of SUB FUNCTIONS ***************************/


/************************************************************************/
/*                               MAIN                                   */
/************************************************************************/
int main (void)
{
    board_init();
    sysclk_init();
    //The pins and functions to be used
    const gpio_map_t usart_gpio_map = {
        {AVR32_ABDAC_DATA_0_0_PIN,AVR32_ABDAC_DATA_0_0_FUNCTION},   // Configure pin J4's SCK as DAC0
        {AVR32_ABDAC_DATA_1_1_PIN,AVR32_ABDAC_DATA_1_1_FUNCTION},   // Configure pin J2's ADC2 as DAC1
        {AVR32_ADC_AD_1_PIN,AVR32_ADC_AD_1_FUNCTION},				// Configure pin J2's ADC1 as ADC
        {AVR32_ADC_AD_5_PIN,AVR32_ADC_AD_5_FUNCTION},				// Configure pin J2's ADC5 as ADC
        {USART_TXD_PIN,USART_TXD_FUNCTION},
        {USART_RXD_PIN,USART_RXD_FUNCTION},
        {65,2},
    };

    //Usart options
    //no parity, no flow control, one stop bit
    static usart_options_t User_Usart_OPT = {
        .baudrate    = 57600,                    //due to LM358's speed, Baudrate can't fast than 57600.
        .channelmode = USART_NORMAL_CHMODE,
        .charlength  = 8,
        .paritytype  = USART_NO_PARITY,
        .stopbits    = USART_1_STOPBIT,
    };

    //Clock options
    static pm_freq_param_t clockSettings;
        clockSettings.cpu_f = U_CPU_F;
        clockSettings.osc0_f = U_OSC0_F;
        clockSettings.osc0_startup = U_OSC0_START;
        clockSettings.pba_f = U_PBA_F;

    //set clock with the options
    if (pm_configure_clocks(&clockSettings) != PM_FREQ_STATUS_OK)
    {
        while(1){};
    }
	/*  Set ADC Sampling Frequency */ 
    uint32_t ADC_FREQ = ADC_FREQUENCY;

    //enable pins
    gpio_enable_module(usart_gpio_map,
        sizeof(usart_gpio_map) / sizeof(usart_gpio_map[0]));

    //ADC prescal setting
    //ADC clock = clock / ((prescal+1)*2)
    AVR32_ADC.mr |= 0x01 << AVR32_ADC_MR_PRESCAL_OFFSET;
    /*ADC Trigger setting. Trigger is Timer 0 channel 0 RA */
    AVR32_ADC.mr |= 0x00 << AVR32_ADC_MR_TRGSEL_OFFSET;
    AVR32_ADC.mr |= 0x01 << AVR32_ADC_MR_TRGEN_OFFSET;
    adc_configure(&AVR32_ADC);
    adc_enable(&AVR32_ADC,AVR32_ADC_CH1);        // enable ADC1
    adc_enable(&AVR32_ADC,AVR32_ADC_CH5);        // enable ADC5

    //DAC init, sampling frequency = FOSC0/256;
    abdac_set_dac_hz(&AVR32_ABDAC,FOSC0,FOSC0/256);
    abdac_enable(&AVR32_ABDAC);

	// External trigger 
	gpio_enable_gpio_pin(Ext_Trigger_Pin);
	gpio_configure_pin(Ext_Trigger_Pin,GPIO_DIR_INPUT);
	gpio_enable_pin_interrupt(Ext_Trigger_Pin,GPIO_RISING_EDGE);
    //Usart init wait until success
    while(usart_init_rs232(USART,&User_Usart_OPT,U_PBA_F) != USART_SUCCESS);
    //test if communication is ok
    usart_write_line(USART,"USART OK!! \r\n");

    // Initialize all PDCA channels to be used 
    pdca_init_channel(PDCA_CHANNEL_ADC,&ADC_PDCA_OPT);
    pdca_init_channel(PDCA_CHANNEL_DAC,&DAC_PDCA_OPT);

    //All PDCA channels need to enable corresponding interrupts
    pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_ADC);
    pdca_enable_interrupt_reload_counter_zero(PDCA_CHANNEL_DAC);


    //call interrupt settings
    pdca_set_irq();

    //Enable all channels before the data begin to be read or written 
    pdca_enable(PDCA_CHANNEL_ADC);
    pdca_enable(PDCA_CHANNEL_DAC);
    //timer init
    tc_init(tc,ADC_FREQ);
	uint32_t extTriggerCount =175*(ADC_FREQ/1000);
    while (1)
    {	
	
		if (counter >= extTriggerCount)
								// control the time when the next Ext_Trigger is valid
		{						 // counter = the number of cycles; "counter = ADC_FREQ" corresponds to the time of 1 second
			gpio_enable_pin_interrupt(Ext_Trigger_Pin,GPIO_RISING_EDGE);  // "counter = ADC_FREQ/1000" corresponds to 1 ms
		}
		if (DSP_STATE==PROCESSING)
		{
			for (int ind=0;ind< Buffer_Size;ind++)	
			{
				DAC_BUFFER_PTR[2*ind] = (DSP_REAL_BUFFER[ind]*2^0x8000);	
				DAC_BUFFER_PTR[2*ind+1]=(DSP_REAL_BUFFER[ind]*2^0x8000);	
			}
/*......................................................................*/
/*            Do DSP HERE                                               */
/*......................................................................*/

	dsp16_trans_realcomplexfft(&DSP_COMP_BUFFER,&DSP_REAL_BUFFER,N_of_bits);
	dsp16_vect_complex_abs(&DSP_REAL_BUFFER,&DSP_COMP_BUFFER,DATA_SIZE);
	
	uint16_t df = ADC_FREQ/DATA_SIZE;
	uint16_t posCut = 1075/df;
	
	/*usart_write_line(USART,"posCut:");
	User_Usart_int2str(posCut);
	usart_write_line(USART,"df: ");
	User_Usart_int2str(df);
	*/
	

    int a = max_L(&DSP_REAL_BUFFER,10,22);
	//usart_write_line(USART," v1: ");
  //    User_Usart_int2str(a*df);
	
	int b = max_L(&DSP_REAL_BUFFER,23,35);
	//usart_write_line(USART," v2: ");
//	User_Usart_int2str(b*df);
	
	
//	usart_write_line(USART,"Num:");
	
	convertFreq(a*df,b*df);
	
	/*dsp16_t v1 = dsp16_vect_max(&DSP_REAL_BUFFER,(posCut));
	dsp16_t v2 = dsp16_vect_max(&DSP_REAL_BUFFER,DATA_SIZE);
	
	
	for(int i =0; i < posCut+400; i++){
	if(v1 == DSP_REAL_BUFFER[i]){
		usart_write_line(USART," v1: ");
		User_Usart_int2str(i+1*df);
	}
	}
	for(int i = 1; i < DATA_SIZE; i++){
	if(v2 == DSP_REAL_BUFFER[i]){
		
		usart_write_line(USART," v2: ");
		User_Usart_int2str(i*df);
		break;
	}
	
	}*/
		
			/*for (int ind=0;ind<= DATA_SIZE;ind =ind++){

				User_Usart_int2str(DSP_REAL_BUFFER[ind]);
				usart_write_line(USART,",");
			}*/



/*......................................................................*/
			DSP_STATE=STANDSTILL;  // 
		}
    }
}
/***************************** End of MAIN *************************************/
// batchisp.exe -device at32uc3a3256 -hardware usb -operation erase f memory flash blankcheck loadbuffer 'C:\Users\user\Documents\Atmel Studio\6.2\SigSys\SigSys\SigSys\Debug\SigSys.hex' program verify start reset 0