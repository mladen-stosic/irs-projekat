/*
 * Mladen Stosic
 *
 * Pollution measurement station
 *
 * Take in data over UART from PSMA003 sensor
 * Display average value from last 4 measurements
 * SW3 and SW4 select which particle concentration is to be displayed
 * If concentration is below 50ug/m3, turn on green LED (LED1) to signal safe levels
 * If concentration exceeds 50ug/m3 turn on red LED (LED2) to signal potentially harmful levels
 *
 */
#include <msp430.h> 
#include <stdint.h>
#include <function.h>

#define S3                  ( 1 )           /* S3 used in main switch case */
#define S4                  ( 2 )           /* S4 used in main switch case */
#define BAUD_RATE_9600      ( 3 )           /* UART BAUD RATE from lookup table*/
#define ARRAY_LENGTH        ( 4 )           /* Length of array of measurements */
#define LAST_BYTE           ( 31 )          /* Position of last received byte in RX array*/
#define RX_LENGTH           ( 32 )          /* Length of RX array */
#define SAFE_LEVELS         ( 50 )          /* Used to check if measured values exceed 50ug/m3 */
#define PM1_0_BYTE_H        ( 10 )          /* Position of PM1_0 measurement HIGH byte in RX array */
#define PM1_0_BYTE_L        ( 11 )          /* Position of PM1_0 measurement LOW byte in RX array */
#define PM2_5_BYTE_H        ( 12 )          /* Position of PM2_5 measurement HIGH byte in RX array */
#define PM2_5_BYTE_L        ( 13 )          /* Position of PM2_5 measurement LOW byte in RX array */
#define PM10_BYTE_H         ( 14 )          /* Position of PM10 measurement HIGH byte in RX array */
#define PM10_BYTE_L         ( 15 )          /* Position of PM10 measurement LOW byte in RX array */
#define MAX_DISPLAY_VALUE   ( 99 )          /* Max possible value to display*/
#define START_CHAR          ( 0x42 )        /* PMSA003 Start character*/
#define TIMER_FREQ          ( 32768 )       /* ACLK Freq */
#define PERIOD_MS           ( 0.005 )       /* 5ms */
#define MULTIPLEX_PERIOD    ( TIMER_FREQ * PERIOD_MS )  /* 7seg multiplexing period */
#define DEBOUNCING_PERIOD   ( TIMER_FREQ * 0.1 )        /* 100ms debouncing period */

// Declare global variables
volatile int8_t   i;
volatile uint8_t  received[RX_LENGTH] = {0};
volatile uint8_t  current_digit = 0;
volatile uint8_t  button = 0;
volatile uint8_t  cnt = 0;
volatile uint8_t  flag = 0;
volatile uint8_t  disp1 = 0;
volatile uint8_t  disp2 = 0;
volatile uint16_t check = 0;
volatile uint16_t checker;

// Arrays holding measurements of particle concentrations of different sizes
uint16_t    PM1_0[ARRAY_LENGTH] = {0};
uint16_t    PM2_5[ARRAY_LENGTH] = {0};
uint16_t    PM10[ARRAY_LENGTH] = {0};
uint8_t     memCnt = 0;
// Variables holding average of 4 different measurements
uint16_t    PM1_0avg = 0;
uint16_t    PM2_5avg = 0;
uint16_t    PM10avg = 0;


/*
 * Function that takes measured average level of pollution
 * and alerts the user if levels are potentially harmful
 * by turning on red LED, when levels are safe green LED
 * is turned on
 */
void check_levels(uint16_t level)
{
    // If harmful levels are detected turn on red LED
    if (level > SAFE_LEVELS)
    {
        P4OUT &= ~BIT7; // Turn off Green LED
        P1OUT |= BIT0;  // Turn on Red LED
    }
    // If safe, turn on green LED
    else
    {
        P1OUT &= ~BIT0; // Turn off Red LED
        P4OUT |= BIT7;  // Turn on Green LED
    }
}

/*
 *  Function for initialization of used resources
 */
void hardware_setup()
{
    // Init buttons S3 & S4
    P1DIR &= ~(BIT4 + BIT5);    // Set direction as input for P1.4 and P1.5
    P1REN |= BIT4 + BIT5;       // Enable pullup-pulldown on P1.4 and P1.5
    P1OUT |= BIT4 + BIT5;       // Set pull up
    P1IES |= BIT4 + BIT5;       // Set Interrupt on falling edge
    P1IE  |= BIT4 + BIT5;       // Enable Interrupts on P1.4 and P1.5

    // Init LEDS
    P1DIR |= BIT0;              // Set Direction for LED1
    P4DIR |= BIT7;              // Set Direction for LED2
    P1OUT &= ~BIT0;             // Turn off LED1
    P4OUT &= ~BIT7;             // Turn off LED2

    // Init 7seg
    // Init 7seg outputs
    P8DIR |= BIT1 | BIT2;       // SET P8.1 & P8.2 as outputs (d & g)
    P4DIR |= BIT3 | BIT0;       // SET P4.3 & P4.0 as outputs (b & f)
    P2DIR |= BIT3 | BIT6;       // SET P2.3 & P2.6 as outputs (e & c)
    P3DIR |= BIT7;              // SET P3.7 as output         (a)

    // Init 7seg SEL signals
    P7DIR |= BIT0;              // SET P7.0 as output ( SEL1 )
    P6DIR |= BIT4;              // SET P6.4 as output ( SEL2 )
    P7OUT |= BIT0;              // Disable display 1
    P6OUT |= BIT4;              // Disable display 2

    // Init UART USCI0 (RX on P3.4 TX on 3.3)
    UCA0CTL1 |= UCSWRST;        // Enter sw reset
    P3SEL |= BIT3 + BIT4;       // Enable alternative function on P3.3 & P3.4
    UCA0CTL0 = 0;               // Set baud rate 9600 with ACLK
    UCA0CTL1 = UCSSEL__ACLK;    // Parity disabled, 1 stop bit
    UCA0BRW = BAUD_RATE_9600;   // 8 bit data
    UCA0MCTL = UCBRS_3 + UCBRF_0;
    UCA0IE = UCRXIE;            // Enable RX interrupt
    UCA0CTL1 &= ~UCSWRST;       // Leave sw reset

    // Init TB0 as compare in up mode
    TB0CCR0 = MULTIPLEX_PERIOD;     // set timer period in CCR0 reg
    TB0CCTL0 = CCIE;                // enable interrupt for TA1CCR0
    TB0CTL = TBSSEL__ACLK | MC__UP; // clock select and up mode

    // Init TA0 as compare
    TA0CCR0 = DEBOUNCING_PERIOD;    // set timer period in CCR0 reg
    TA0CCTL0 = CCIE;                // enable interrupt for TA0CCR0
    TA0CTL = TASSEL__ACLK;          // clock select and up mode
}
/*
 *  Function that takes an unsigned integer and fills
 *  two display variables with BCD code
 */
void intToBCD(const uint16_t number)
{
    uint8_t tmp;
    uint16_t converted = 0;
    uint16_t nr = number;

    for (tmp = 8; tmp > 0; tmp--) {
        converted = __bcd_add_short(converted, converted);
        if ((nr & BIT7) != 0) {
            converted = __bcd_add_short(converted, 1);
        }
        nr <<= 1;
    }
    disp2 = converted & (0xf);
    disp1 = (converted & (0xf << 4)) >> 4;
}

/**
 * main.c
 */
void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	hardware_setup();
	__enable_interrupt();

	while(1)
	{
	    // Do when full data block is received
	    if (flag == 1)
	    {
	        // Extract high and low byte and merge them into 1 variable
	        PM1_0[memCnt] = (received[PM1_0_BYTE_H] << 8) | received[PM1_0_BYTE_L];
	        PM2_5[memCnt] = (received[PM2_5_BYTE_H] << 8) | received[PM2_5_BYTE_L];
	        PM10[memCnt]  = (received[PM10_BYTE_H] << 8) | received[PM10_BYTE_L];

	        memCnt++;
	        if (memCnt > ARRAY_LENGTH - 1){
	            memCnt = 0;
	        }
	        // Calculate average of last 3 measurements
            PM1_0avg = 0;
            PM2_5avg = 0;
            PM10avg = 0;
	        for (i = ARRAY_LENGTH - 1; i >= 0; i--)
	        {
	            // Sum up all the values
	            PM1_0avg += PM1_0[i];
                PM2_5avg += PM2_5[i];
                PM10avg += PM10[i];
	        }
	        // Divide by 4 to get average values
	        PM1_0avg >>= 2;
	        PM2_5avg >>= 2;
	        PM10avg >>= 2;

	        // Constrain values to max display value
	        if (PM2_5avg > MAX_DISPLAY_VALUE){
	            PM2_5avg = MAX_DISPLAY_VALUE;
	        }
	        if (PM10avg > MAX_DISPLAY_VALUE){
	            PM10avg = MAX_DISPLAY_VALUE;
	        }

	        // Reset flag
	        flag = 0;
	    }
	    // When button is pressed enter one of the cases then
	    // return button value to 0 until button is pressed again
	    __disable_interrupt();
	    switch(button){
            case 0:
                break;
            case S3:
                // Check if levels are safe and turn on LEDs accordingly
                check_levels(PM2_5avg);
                // Reset button value
                button = 0;
                break;
            case S4:
                // Extract BCD
                intToBCD(PM10avg);
                // Check if levels are safe and turn on LEDs accordingly
                check_levels(PM10avg);
                // Reset button value
                button = 0;
                break;
	    }
	    __enable_interrupt();
	}
}
/*
 * Interrupt routine for UART
 */
void __attribute__ ((interrupt(USCI_A0_VECTOR))) UARTISR (void)
{
    received[cnt++] = UCA0RXBUF;    // Fill received array with data from UART buffer
    if (received[0] != START_CHAR)  // If first element isn't equal to start character
    {                               // reset counter and reset array values
        for (i = RX_LENGTH - 1; i >= 0; i--)
        {
            received[i] = 0;
            cnt = 0;
            check = 0;
        }
    }
    if (cnt > RX_LENGTH - 1){               // If counter reached max value, reset counter
        cnt = 0;
    }
    if (received[RX_LENGTH - 1] != 0)       // When last byte of data is received
    {                                       // sum all first 30 bytes of data and
        for (i = RX_LENGTH - 3; i >= 0; i--)// compare it to last 2 received bytes
        {
            check += received[i];
        }
        checker = (received[LAST_BYTE - 1] << 8) | received[LAST_BYTE];
        if (check = checker)
        {
            // Set flag             // If received data sum matches checker
            flag = 1;               // set flag that indicates that
        }                           // full packet is received
        received[0] = 0;
        received[RX_LENGTH - 1] = 0;           // Reset bytes that are checked in the routine
        cnt = 0;                    // Reset counter
    }
}
