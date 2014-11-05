
#include "ECANPoll.h"
#include "Define.h"

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Define system operating frequency */

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        8000000L
#define FCY             SYS_FREQ/4


/******************************************************************************/
/* Variable types declaration                                                      */
/******************************************************************************/

struct IMUData {

    unsigned int XAccelerationData;
    unsigned int YAccelerationData;
    unsigned int ZAccelerationData;

    unsigned int XGyroscopeData;
    unsigned int YGyroscopeData;
    unsigned int ZGyroscopeData;

    unsigned int XMagnetData;
    unsigned int YMagnetData;
    unsigned int ZMagnetData;

    unsigned int MagnetResistance;
    
    unsigned char Temperature;

};

struct RTC_counter {
    unsigned char AccelTick_ms;
    unsigned char GyroTick_ms;
    unsigned char MagnetTick_ms;

};

struct CANTxMsg{

    unsigned long id;
    BYTE data_TX[8];
    BYTE dataLen;
    ECAN_TX_MSG_FLAGS flags;

};

struct CANRxMsg{

    unsigned long id;
    BYTE data_RX[8];
    BYTE dataLen;
    ECAN_RX_MSG_FLAGS flags;

};

struct CANTxFifo{

    struct CANTxMsg CANMsg[CANTX_FIFO_SIZE];

    unsigned char LowIndex;
    unsigned char HighIndex;
    unsigned char Fifofull;
    unsigned char FifoEmpty;

};

struct CANRxFifo{

    struct CANRxMsg CANMsg[CANRX_FIFO_SIZE];

    unsigned char LowIndex;
    unsigned char HighIndex;
    unsigned char Fifofull;
    unsigned char FifoEmpty;

};

struct USARTRxFifo {

    unsigned char USARTMsg [USART_RX_FIFO_SIZE];
    unsigned char LowIndex;
    unsigned char HighIndex;
    unsigned char Fifofull;
    unsigned char FifoEmpty;

};




 
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/


/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */

void ConfigureOscillator(void);     /* Handles clock switching/osc initialization */
void ConfigureGPIO(void);           /* Handles GPIO configuration*/
void ConfigureUSART1(void);         /* Handles USART1 configuration for GSM module*/
void ConfigureUSART2(void);         /* Handles USART2 configuration for GPS module*/
void ConfigureAnalog(void);         /* Handles ADC configuration for CAN adress read on startup*/
void ConfigureSPI(void);            /* Handles SPI configuration for ISM wireless module and MEMS sensor*/
void ConfigureInterrupts(void);     /* Handles Interrupts configuration*/

void ConfigureTimers(void);         /* handles timers configuration*/

void USART1Write(char data);        /* Handles write routine for USART1*/
void USART2Write(char data);        /* Handles write routine for USART2*/

signed char SPIWrite(char data);    /* handles SPI byte write*/
char SPIRead(char DummyData);       /*Handles spi read byte*/

void USARTFifoInit (void);          //initialize USART FIFO pointers

char GetUSART1RxFifo(void); // get byte from RX1 Fifo
void PutUSART1RxFifo(char ByteToWrite); // put byte into Rx1 Fifo

char GetUSART2RxFifo(void); // get byte from RX2 Fifo
void PutUSART2RxFifo(char ByteToWrite); // put byte into Rx2 Fifo