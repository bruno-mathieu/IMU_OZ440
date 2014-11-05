/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>        /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>       /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>   /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <stdio.h>

#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "define.h"        /* board level definitions*/
#include "ecanpoll.h"      /* CAN library header file*/
#include "ISMModule.h"     /* ISM Module routines*/
#include "IMUModule.h"     /* IMU Module routines*/
#include "Can_HL.h"

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

// structure used to count 10ms tick
struct RTC_counter TickCounter;

//  variable for CAN  FIFO buffer
struct CANTxFifo CANTxFifo;
struct CANRxFifo CANRxFifo;

 // Variable for USART RX FIFO buffer
struct USARTRxFifo USART1RxFifo;
struct USARTRxFifo USART2RxFifo;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

void main(void)
{
    char TempVar;

    // variable used for IMU chip Autotest
    unsigned char IMUAutotestResult;

    // structure used to store IMU data
    struct IMUData CurrentIMUData;

    //  variable for CAN TX FIFO buffer
    struct CANTxMsg TempCANTxMsg;

    //  variable for CAN RX FIFO buffer
    struct CANRxMsg TempCANRxMsg;

    
    


    //----------------------------------------------------
    //----------  CPU internal configurations: -----------
    //----------------------------------------------------

    /* Configure the oscillator for the CPU */
    ConfigureOscillator();
    __delay_ms(10);             // wait for Oscillator to be stabilized


    // configure CPU GPIO for IMU board
    ConfigureGPIO();

    //USART Initialize();
    ConfigureUSART1();
    ConfigureUSART2();

    // SPI initialize
    ConfigureSPI();

    //CAN controller Initialize
    ECANInitialize();
    //Set MASK and Filters for CAN
    ECANFiltersInit();

    // Timers configuration
    ConfigureTimers();

    //----------------------------------------------------
    //----------  Global variables initialisation --------
    //----------------------------------------------------

    // tick counter initialisation
    TickCounter.AccelTick_ms=0;         
    TickCounter.GyroTick_ms=1;
    TickCounter.MagnetTick_ms=2;

    // initialize CAN tx FIFO
    CANTxFifoInit();
    CANRxFifoInit();

    // initialise USART RX FIFO's
    USARTFifoInit ();


    //----------------------------------------------------
    //------  external peripheral configurations: --------
    //----------------------------------------------------

    __delay_ms(10);              // wait for reset to be released on external peripherals
    
    ISM_RESET = 0;               // release reset of ISM module

    IMUInitRegisters();         // init of BMX055 chip
    IMUAutotestResult=IMUAutotest();              // launch IMU autotest

    
    //----------------------------------------------------
    //----------      GSM startup delay        -----------
    //----------------------------------------------------

    GSM_RTS=1;
    for(char i=0;i<200;i++)
    {
        __delay_ms(10);
    }
    GSM_RTS=0;

    __delay_ms(10);
    __delay_ms(10);


    //----------------------------------------------------
    //----------    Ready to go in main loop:  -----------
    //----------    interrupts activation      -----------
    //----------------------------------------------------

    ConfigureInterrupts();
    LED1=1;                     // everything is initialized: enable the PWR/booted LED

    //----------------------------------------------------
    //----------     GSM dummy AT command      -----------
    //----------------------------------------------------

    USART1Write('A');
    USART1Write('T');
    USART1Write(0x0D);

    for(char i=0;i<10;i++)
    {
        __delay_ms(10);
    }

    //-----------------------------------------------------
    //-------------  infinite main loop ----------
    //----------------------------------------------------


    while(1)
    {

        //--------------------------------------------------------------------------------
        //-------------  periodic tasks occures according to TickCounter variable----------
        //--------------------------------------------------------------------------------

        if(TickCounter.AccelTick_ms>IMU_TICK_PERIOD)
        {

            LED2=1;
            TickCounter.AccelTick_ms=0;                // reset IMU tick counter to 0
            CurrentIMUData = IMUUpdateData();        // update IMU data from sensor

            // send Accelerometer data to CAN Fifo
        
            TempCANTxMsg.data_TX[0]=(char)(CurrentIMUData.XAccelerationData>>8);      //fill data buffer
            TempCANTxMsg.data_TX[1]=(char)(CurrentIMUData.XAccelerationData);
            TempCANTxMsg.data_TX[2]=(char)(CurrentIMUData.YAccelerationData>>8);
            TempCANTxMsg.data_TX[3]=(char)(CurrentIMUData.YAccelerationData);
            TempCANTxMsg.data_TX[4]=(char)(CurrentIMUData.ZAccelerationData>>8);
            TempCANTxMsg.data_TX[5]=(char)(CurrentIMUData.ZAccelerationData);
            TempCANTxMsg.data_TX[6]=0;
            TempCANTxMsg.data_TX[7]=0;

            TempCANTxMsg.dataLen= ACCEL_DATA_MESSAGE_LEN;
            TempCANTxMsg.id = (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | ACCEL_DATA_MESSAGE_ADRESS );
            TempCANTxMsg.flags = ECAN_TX_STD_FRAME;

            if(!CANTxFifo.Fifofull)
                 PutCANTxFifo(TempCANTxMsg);

            LED2=0;

        }

        if(TickCounter.GyroTick_ms>IMU_TICK_PERIOD)
        {
            //LED2=1;
            TickCounter.GyroTick_ms=0;                // reset IMU tick counter to 0

            // send Gyro data to CAN Fifo
        
            TempCANTxMsg.data_TX[0]=(char)(CurrentIMUData.XGyroscopeData>>8);
            TempCANTxMsg.data_TX[1]=(char)(CurrentIMUData.XGyroscopeData);
            TempCANTxMsg.data_TX[2]=(char)(CurrentIMUData.YGyroscopeData>>8);
            TempCANTxMsg.data_TX[3]=(char)(CurrentIMUData.YGyroscopeData);
            TempCANTxMsg.data_TX[4]=(char)(CurrentIMUData.ZGyroscopeData>>8);
            TempCANTxMsg.data_TX[5]=(char)(CurrentIMUData.ZGyroscopeData);

            TempCANTxMsg.dataLen= GYRO_DATA_MESSAGE_LEN;
            TempCANTxMsg.id = (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | GYRO_DATA_MESSAGE_ADRESS );
            TempCANTxMsg.flags = ECAN_TX_STD_FRAME;

            if(!CANTxFifo.Fifofull)
                 PutCANTxFifo(TempCANTxMsg);
            //LED2=0;

        }

        if(TickCounter.MagnetTick_ms>IMU_TICK_PERIOD)
        {
            //LED2=1;
            TickCounter.MagnetTick_ms=0;                // reset IMU tick counter to 0

        // send MAGNETO data to CAN Fifo
        
            TempCANTxMsg.data_TX[0]=(char)(CurrentIMUData.XMagnetData>>8);
            TempCANTxMsg.data_TX[1]=(char)(CurrentIMUData.XMagnetData);
            TempCANTxMsg.data_TX[2]=(char)(CurrentIMUData.YMagnetData>>8);
            TempCANTxMsg.data_TX[3]=(char)(CurrentIMUData.YMagnetData);
            TempCANTxMsg.data_TX[4]=(char)(CurrentIMUData.ZMagnetData>>8);
            TempCANTxMsg.data_TX[5]=(char)(CurrentIMUData.ZMagnetData);
            TempCANTxMsg.data_TX[6]=(char)(CurrentIMUData.MagnetResistance >>8);
            TempCANTxMsg.data_TX[7]=(char)(CurrentIMUData.MagnetResistance);

            TempCANTxMsg.dataLen= MAGN_DATA_MESSAGE_LEN;
            TempCANTxMsg.id = (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | MAGN_DATA_MESSAGE_ADRESS );
            TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
         
            if(!CANTxFifo.Fifofull)
                 PutCANTxFifo(TempCANTxMsg);

            //LED2=0;
        }

        //--------------------------------------------------------------------------------
        //-------------   Permanent tasks: executed as fast as possible  ----------------
        //--------------------------------------------------------------------------------

        //--------------------------------------------------------------------------------
        // --------  Check if CAN RX fifo is empty or not and perform treatment  ---------
        //--------------------------------------------------------------------------------
        
        if(!CANRxFifo.FifoEmpty && !CANTxFifo.Fifofull)
        {
            TempCANRxMsg = GetCANRxFifo();

            // ------------------  Return software version to CAN if RTR message detected  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | SOFT_VERSION_MESSAGE_ADRESS ) && TempCANRxMsg.flags== ECAN_RX_RTR_FRAME )
            {
                TempCANTxMsg.data_TX[0]=MAJOR_SW_VERSION;
                TempCANTxMsg.data_TX[1]=MINOR_SW_VERSION;
                TempCANTxMsg.dataLen= SOFT_VERSION_MESSAGE_LEN;
                TempCANTxMsg.id = (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | SOFT_VERSION_MESSAGE_ADRESS );
                TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
                PutCANTxFifo(TempCANTxMsg);
            }
            
            // ------------------  Return Board revision and number to CAN bus if RTR message detected  ------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | BOARD_VERSION_MESSAGE_ADRESS ) && TempCANRxMsg.flags== ECAN_RX_RTR_FRAME )
            {
                TempCANTxMsg.data_TX[0]=BOARD_NUMBER;
                TempCANTxMsg.data_TX[1]=BOARD_REVISION;
                TempCANTxMsg.dataLen= BOARD_VERSION_MESSAGE_LEN;
                TempCANTxMsg.id = (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | BOARD_VERSION_MESSAGE_ADRESS );
                TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
                PutCANTxFifo(TempCANTxMsg);
            }

            // ------------------  Return Temperature of IMU to CAN bus  if RTR message detected  ------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | TEMPERATURE_MESSAGE_ADRESS ) && TempCANRxMsg.flags== ECAN_RX_RTR_FRAME )
            {
                TempCANTxMsg.data_TX[0]=CurrentIMUData.Temperature;
                TempCANTxMsg.dataLen= TEMPERATURE_MESSAGE_LEN;
                TempCANTxMsg.id = (CAN_MESSAGE_IMU_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | TEMPERATURE_MESSAGE_ADRESS );
                TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
                PutCANTxFifo(TempCANTxMsg);
            }

             // ------------------   Send data to GSM if requested by canBUS     ------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_GSM_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | GSM_DATATX_MESSAGE_ADRESS ) && TempCANRxMsg.dataLen==GSM_DATATX_MESSAGE_LEN )
            {
                USART1Write(TempCANRxMsg.data_RX[0]);
            }

        }


        //--------------------------------------------------------------------------------
        // --------  Check if GPS RX fifo is empty and send to CAN Fifo ------------------
        //--------------------------------------------------------------------------------
        
        if(!USART2RxFifo.FifoEmpty && !CANTxFifo.Fifofull)       // if USART2 is not empty, and CANTX Fifo is not full...
        {
            TempVar = GetUSART2RxFifo();                        // read from USART FIFO

            TempCANTxMsg.data_TX[0]=TempVar;
            TempCANTxMsg.dataLen= GPS_DATA_MESSAGE_LEN;
            TempCANTxMsg.id = (CAN_MESSAGE_GPS_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | GPS_DATA_MESSAGE_ADRESS );
            TempCANTxMsg.flags = ECAN_TX_STD_FRAME;

            PutCANTxFifo(TempCANTxMsg);

         }

         //--------------------------------------------------------------------------------
        // --------  Check if GSM RX fifo is empty and send to CAN Fifo ------------------
        //--------------------------------------------------------------------------------

        if(!USART1RxFifo.FifoEmpty && !CANTxFifo.Fifofull)       // if USART is not empty, and CANTX Fifo is not full...
        {
            TempVar = GetUSART1RxFifo();                        // read from USART FIFO
            TempCANTxMsg.data_TX[0]=TempVar;
            TempCANTxMsg.dataLen= GSM_DATARX_MESSAGE_LEN;
            TempCANTxMsg.id = (CAN_MESSAGE_GSM_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | GSM_DATARX_MESSAGE_ADRESS );
            TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
            PutCANTxFifo(TempCANTxMsg);
         }

        //--------------------------------------------------------------------------------
        // ---  Send can message if TXB0 buffer free, and data available in CAN TX FIFO --
        //--------------------------------------------------------------------------------
        
        if(!CANTxFifo.FifoEmpty && !TXB0CONbits.TXREQ)          // if fifo is not empty and buffer0 empty
        {
            LED2=1;
            TempCANTxMsg=GetCANTxFifo();
            ECANSendMessage(TempCANTxMsg.id,TempCANTxMsg.data_TX,TempCANTxMsg.dataLen,TempCANTxMsg.flags);  // fill tx buffer with Fifo data
            LED2=0;
        }


    }

}

