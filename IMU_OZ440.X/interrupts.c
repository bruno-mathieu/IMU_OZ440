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
#include "Can_HL.h"

#endif

extern struct RTC_counter TickCounter;
extern struct USARTRxFifo USART1RxFifo;
extern struct USARTRxFifo USART2RxFifo;
extern struct CANRxFifo CANRxFifo;

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* High-priority service */

#if defined(__XC) || defined(HI_TECH_C)
void interrupt high_isr(void)
#elif defined (__18CXX)
#pragma code high_isr=0x08
#pragma interrupt high_isr
void high_isr(void)
#else
#error "Invalid compiler selection for implemented ISR routines"
#endif

{
    struct CANRxMsg CANRxNewMessage;

      /* This code stub shows general interrupt handling.  Note that these
      conditional statements are not handled within 3 seperate if blocks.
      Do not use a seperate if block for each interrupt flag to avoid run
      time errors. */

    if(PIR5bits.RXB0IF==1 | PIR5bits.RXB1IF==1)          // new CAN message received in buffer0 or buffer1
        {

            ECANReceiveMessage(&CANRxNewMessage.id, CANRxNewMessage.data_RX, &CANRxNewMessage.dataLen, &CANRxNewMessage.flags);
            PutCANRxFifo(CANRxNewMessage);

            PIR5bits.RXB0IF=0;      // clear interrupt flags
            PIR5bits.RXB1IF=0;
        }

   


}


/* Low-priority interrupt routine */
#if defined(__XC) || defined(HI_TECH_C)
void low_priority interrupt low_isr(void)
#elif defined (__18CXX)
#pragma code low_isr=0x18
#pragma interruptlow low_isr
void low_isr(void)
#else
#error "Invalid compiler selection for implemented ISR routines"
#endif
{

    unsigned char temp;
     
    if(INTCONbits.TMR0IF)           // TIMER0 overflow interrupt
    {
        INTCONbits.TMR0IF=0;        // Clear interrupt flag

        TickCounter.AccelTick_ms ++;  //increment accelerometer IMU tick counter
        TickCounter.GyroTick_ms ++;   //increment Gyro IMU tick counter
        TickCounter.MagnetTick_ms ++;  //increment magnetometer IMU tick counter

        TMR0H = TMR0H_INIT;
        TMR0L = TMR0L_INIT;         // preload for 10ms overflow

    }
    
    if(PIR1bits.RCIF)          //USART1 receive interrupt
    {
        PIR1bits.RCIF=0;            // Clear interrupt flag
        temp= RCREG;
        PutUSART1RxFifo(temp);
        if(RCSTA1bits.OERR)         //if overrun error, disable and enable receiver again (occurs only during startup)
        {
            RCSTA1bits.CREN=0;
            RCSTA1bits.CREN=1;
        }
     }

    if(PIR3bits.RC2IF)          //USART2 receive interrupt
    {
        PIR3bits.RC2IF=0;            // Clear interrupt flag
        temp= RCREG2;
        PutUSART2RxFifo(temp);
        if(RCSTA2bits.OERR)         //if overrun error, disable and enable receiver again (occurs only during startup)
        {
            RCSTA2bits.CREN=0;
            RCSTA2bits.CREN=1;
        }
    }

}
