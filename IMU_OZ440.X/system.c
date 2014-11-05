/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>    /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#include "system.h"
#include "define.h"

#endif

/******************************************************************************/
/* Extern global variables declarations                                       */
/******************************************************************************/

extern struct USARTRxFifo USART1RxFifo;
extern struct USARTRxFifo USART2RxFifo;


/* Refer to the device datasheet for information about available
oscillator configurations. */

void ConfigureOscillator(void)
{

    OSCCONbits.IDLEN=1; /* Iddle mode when SLEEP instruction*/
    OSCCONbits.IRCF=0;  /* don't care*/
    OSCCONbits.SCS=0;   /*Primary oscillator selected*/

}

/* ---------------   GPIO configurations ---------------*/

void ConfigureGPIO(void)
{

    ADCON0bits.CHS = 0;     // AN0 selection
    ADCON0bits.ADON=1;       // A/D is operating
    ADCON1 = 0;             // reference and negative channel selection

    ADCON2bits.ADFM = 1;    // result right justified
    ADCON2bits.ACQT = 0x07; // 20TAD acisition time
    ADCON2bits.ADCS = 0x06; // Fosc/64 clock source for ADC

    ANCON0 = 0b0000001;     // only AN0 configured as analog
    ANCON1 = 0;             // all digital (AN8--AN14)

    TRISA = 0b00000101;    // inputs for ADDRCAN, and CTS lines
    LATA = 0b00001000;     // fix RTS line to 1 (standby)

    TRISB = 0b11111011;    // output for TX CAN
    LATBbits.LATB2=1;      // fix output TXCAN

    TRISC = 0b10010000;    // outputs for IMU SPI CS, EUSART1 TiiX and SPI signals
    ACCEL_SPI_CS = 1;      // IMU CS set to 1     
    GYRO_SPI_CS = 1;
    MAGNET_SPI_CS = 1;
    
    TRISD = 0b10111000;     /* output for leds and USART2 TX line*/
    LED1=0;
    LED2=0;
    ISM_RESET=1;            /*ISM reset set active*/

    TRISE = 0b00000000;     /* output ISM SPI CS */
    ISM_CON_CS = 1;
    ISM_DAT_CS = 1;


}

/* ---------------   UART1 configuration ---------------*/
void ConfigureUSART1(void)
{
    /* USART used for GSM module: default configuration:
        - 9600 bd
        - BRG LOW*/

    TXSTA1 = 0;             // Reset USART registers to POR state
    RCSTA1 = 0;

    TXSTA1bits.SYNC = 0;    // asynchronous mode

    TXSTA1bits.TX9 = 0;     // TX in 8 bits mode
    RCSTA1bits.RX9 = 0;     // RX in 8 bits mode

    TXSTA1bits.CSRC = 0;    // don't care

    RCSTA1bits.CREN = 1;   // enable receiver

    TXSTA1bits.BRGH = 0;    // low speed baud rate generator
    BAUDCON1bits.BRG16 = 0; // 16 Bits Baud rate generator disabled

    PIE1bits.RC1IE = 1;     // interrupt on RX
    PIE1bits.TX1IE = 0;     // no interrupt on TX

    SPBRG1 = 103;           // Write baudrate to SPBRG1 
    
    TXSTA1bits.TXEN = 1;    // Enable transmitter
    RCSTA1bits.SPEN = 1;    // Enable receiver

    
}

/* ---------------   UART2 configuration ---------------*/
void ConfigureUSART2(void)
{
/* USART used for GPS module: default configuration:
        - 9600 bd
        - BRG LOW*/

    TXSTA2 = 0;             // Reset USART registers to POR state
    RCSTA2 = 0;

    TXSTA2bits.SYNC = 0;    // asynchronous mode

    TXSTA2bits.TX9 = 0;     // TX in 8 bits mode
    RCSTA2bits.RX9 = 0;     // RX in 8 bits mode

    TXSTA2bits.CSRC = 0;    // don't care

    RCSTA2bits.CREN = 1;   // enable receiver

    TXSTA2bits.BRGH = 0;    // Low speed baud rate generator
    BAUDCON2bits.BRG16 = 0; // 16 Bits Baud rate generator disabled

    PIE1bits.RC1IE = 1;     // interrupt on RX
    PIE1bits.TX1IE = 0;     // no interrupt on TX

    SPBRG2 = 103;         // Write baudrate to SPBRG2

    TXSTA2bits.TXEN = 1;  // Enable transmitter
    RCSTA2bits.SPEN = 1;  // Enable receiver

    
}

/* ---------------   Analog converter configuration ---------------*/
void ConfigureAnalog(void)
{
 /* Fclk/64 for ADC clock source
    Right justified
    2TA ACK timing
    ADC0 channel selected, for CAN adress sensing
    Interrupt disabled
    VDD and VSS for voltages references*/
    
    CloseADC();
    OpenADC(ADC_FOSC_64 | ADC_RIGHT_JUST | ADC_2_TAD , ADC_CH0 | ADC_INT_OFF, ADC_REF_VDD_VSS);

}

void ConfigureSPI(void)
{

  SSPSTAT &= 0x3F;               // power on state

  SSPCON1bits.SSPM = 0b0010;    // 1Mhz clock for SPI

  SSPCON1bits.CKP = 0;          // clock idle is a Low state
  SSPSTATbits.CKE = 1;          // data transmitted on falling edge

  SSPCON1bits.SSPEN = 1;        // enable synchronous serial port

}

void ConfigureInterrupts(void)
{
    //all interrupts are low priority, except CAN module

    RCONbits.IPEN = 1;           // enables priority logic for Interrupts.
            
    INTCON = 0b00100000;        // GIE interrupts disabled for the moment, TMRO OVF activated
    INTCON2 = 0b11111000;       // low priority for TMR0 overflow
    INTCON3 = 0b00000000;       // no external interrupt

    PIR1 = 0;                   //clear IT flags
    PIR2 = 0;
    PIR3 = 0;
    PIR4 = 0;
    PIR5 = 0;

    PIE1 = 0b00100000;          // enable USART1  RX interrupts.
    PIE2 = 0;
    PIE3 = 0b00100000;          // enable USART2  RX interrupts.
    PIE4 = 0;
    PIE5 = 0b00000011;          // enable CAN RX interrupts.

    IPR1= 0 ;                   // low priority IT's
    IPR2= 0 ;                   // low priority IT's
    IPR3= 0 ;                   // low priority IT's
    IPR4= 0 ;                   // low priority IT's
    IPR5= 0b00000011 ;          // High priority for CAN RX.
   
    INTCONbits.GIE=1;           // general IT enable
    INTCONbits.PEIE=1;          // peripheral IT enable
}

void ConfigureTimers(void)
{
    T0CONbits.T08BIT = 0;       //TMR0 in 16 bits mode
    T0CONbits.T0CS = 0;         // TMRO source is internal clock
    T0CONbits.PSA = 0;          // PSA assigned to TIMER0
    T0CONbits.T0PS=6;           // 1:128 prescaler

    TMR0H = TMR0H_INIT;
    TMR0L = TMR0L_INIT;         // preload for 10ms overflow (TMR0 = 1250)

    T0CONbits.TMR0ON = 1;       // enable TIMER0

}

void USART1Write(char data)
{
    while(TXSTA1bits.TRMT==0);  // wait for last transmission to be completed
    TXREG1=data;                // send data to TX buffer
}

void USART2Write(char data)
{
    while(TXSTA2bits.TRMT==0);  // wait for last transmission to be completed
    TXREG2=data;                // send data to TX buffer
}

signed char SPIWrite(char data)
{
  unsigned char TempVar;
  INTCONbits.GIE=0;                   // disable interrupts during SPI access

  TempVar = SSPBUF;             // Clears BF
  PIR1bits.SSPIF = 0;           // Clear interrupt flag
  SSPCON1bits.WCOL = 0;         //Clear any previous write collision
  SSPBUF = data;                // write byte to SSPBUF register
  if ( SSPCON1 & 0x80 )         // test if write collision occurred
  {
      INTCONbits.GIE=1;           // enable interrupts during SPI access
      return ( -1 );              // if WCOL bit is set return negative #
  }

  else
    while( !SSPSTATbits.BF );   // wait until bus cycle complete
   //while( !PIR1bits.SSPIF );  // wait until bus cycle complete

    INTCONbits.GIE=1;           // enable interrupts after SPI access
    return ( 0 );               // if WCOL bit is not set return non-negative#
 
}

char SPIRead(char DummyData)
{
   unsigned char TempVar;

    INTCONbits.GIE=0;                   // disable interrupts during SPI access
    TempVar = SSPBUF;             // Clears BF
    PIR1bits.SSPIF = 0;           // Clear interrupt flag
    SSPCON1bits.WCOL = 0;         //Clear any previous write collision
    SSPBUF = DummyData;           // write byte to SSPBUF register

   while( !SSPSTATbits.BF );    // wait until bus cycle complete
  
   TempVar = SSPBUF;
    
   INTCONbits.GIE=1;           // enable interrupts after SPI access
   return ( TempVar );           // return the SSPBUF value
}

void USARTFifoInit (void)
{
    USART1RxFifo.FifoEmpty=1;
    USART1RxFifo.Fifofull=0;
    USART1RxFifo.HighIndex=0;
    USART1RxFifo.LowIndex=0;

    USART2RxFifo.FifoEmpty=1;
    USART2RxFifo.Fifofull=0;
    USART2RxFifo.HighIndex=0;
    USART2RxFifo.LowIndex=0;

}

void PutUSART1RxFifo(char ByteToWrite)
{
    INTCONbits.GIE=0;                   // disable interrupts during Fifo filling

    if(!USART1RxFifo.Fifofull)                                         // if the fifo is not full...
    {
        USART1RxFifo.USARTMsg[USART1RxFifo.HighIndex] = ByteToWrite;  // write the message in parameters
        USART1RxFifo.FifoEmpty=0;                                      // so the fifo is not empty
        USART1RxFifo.HighIndex++;                                      // incr High index

        if(USART1RxFifo.HighIndex==(USART_RX_FIFO_SIZE))               // if the high index match to fifo size, go back to 0
            USART1RxFifo.HighIndex=0;

        if(USART1RxFifo.HighIndex==USART1RxFifo.LowIndex)             //after last message been written, is the buffer full?
            USART1RxFifo.Fifofull=1;
    }
    INTCONbits.GIE=1;           // enable interrupts after fifo filling

}

char GetUSART1RxFifo(void)
{
    char CharToRead;


    if(!USART1RxFifo.FifoEmpty)            // if the fifo is not empty....
    {

        INTCONbits.GIE=0;                   // disable interrupts during Fifo reading

        CharToRead=USART1RxFifo.USARTMsg[USART1RxFifo.LowIndex];
        USART1RxFifo.Fifofull=0;                              // so the fifo is not full
        USART1RxFifo.LowIndex++;

        if(USART1RxFifo.LowIndex==(CANTX_FIFO_SIZE))          // if the low index match to fifo size, go back to 0
            USART1RxFifo.LowIndex=0;

        if(USART1RxFifo.HighIndex==USART1RxFifo.LowIndex)     //after last message been read, is the buffer empty?
            USART1RxFifo.FifoEmpty=1;

        INTCONbits.GIE=1;           // enable interrupts after fifo reading
        return CharToRead;
    }
    else
    {
         return 0 ;       // fifo empty, return 0
    }

}

void PutUSART2RxFifo(char ByteToWrite)
{

    INTCONbits.GIE=0;                   // disable interrupts during Fifo filling

    if(!USART2RxFifo.Fifofull)                                         // if the fifo is not full...
    {
        USART2RxFifo.USARTMsg[USART2RxFifo.HighIndex] = ByteToWrite;  // write the message in parameters
        USART2RxFifo.FifoEmpty=0;                                      // so the fifo is not empty
        USART2RxFifo.HighIndex++;                                      // incr High index

        if(USART2RxFifo.HighIndex==(USART_RX_FIFO_SIZE))               // if the high index match to fifo size, go back to 0
            USART2RxFifo.HighIndex=0;

        if(USART2RxFifo.HighIndex==USART2RxFifo.LowIndex)             //after last message been written, is the buffer full?
            USART2RxFifo.Fifofull=1;
    }

    INTCONbits.GIE=1;           // enable interrupts after fifo filling

}

char GetUSART2RxFifo(void)
{
    char CharToRead;

    if(!USART2RxFifo.FifoEmpty)            // if the fifo is not empty....
    {

        INTCONbits.GIE=0;                   // disable interrupts during Fifo reading

        CharToRead=USART2RxFifo.USARTMsg[USART2RxFifo.LowIndex];    // store the byte into fifo
        USART2RxFifo.Fifofull=0;                                    // so the fifo is not full
        USART2RxFifo.LowIndex++;                    

        if(USART2RxFifo.LowIndex==(CANTX_FIFO_SIZE))          // if the low index match to fifo size, go back to 0
            USART2RxFifo.LowIndex=0;

        if(USART2RxFifo.HighIndex==USART2RxFifo.LowIndex)     //after last message been read, is the buffer empty?
            USART2RxFifo.FifoEmpty=1;

        INTCONbits.GIE=1;           // enable interrupts after fifo reading
        return CharToRead;
    }
    else
    {
        return 0 ;       // fifo empty, return 0
    }

}