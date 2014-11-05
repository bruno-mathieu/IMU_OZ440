/* 
 * File:   Define.h
 * Author: Bruno
 *
 * Created on 13 juin 2014, 16:13
 */

#ifndef DEFINE_H
#define	DEFINE_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* DEFINE_H */
#define MAJOR_SW_VERSION 1
#define MINOR_SW_VERSION 1

#define BOARD_NUMBER 1
#define BOARD_REVISION 1

#define _XTAL_FREQ 64000000

#define TMR0H_INIT  0xFB              // TMR0 value for 10ms tick
#define TMR0L_INIT  0x1E              // TMR0 value for 10ms tick

#define IMU_TICK_PERIOD 0             // IMU data tick each 10ms

#define ACCEL_SPI_CS    LATCbits.LATC2
#define GYRO_SPI_CS     LATCbits.LATC1
#define MAGNET_SPI_CS   LATCbits.LATC0

#define ACCEL_ID    0
#define GYRO_ID     1
#define MAGNET_ID   2

#define ACCEL_MANUFACTURING_CODE 0xFA
#define GYRO_MANUFACTURING_CODE 0x0F
#define MAGNET_MANUFACTURING_CODE 0x32

#define USART_RX_FIFO_SIZE 10

#define CAN_MESSAGE_IMU_TYPE 3
#define CAN_MESSAGE_GPS_TYPE 4
#define CAN_MESSAGE_GSM_TYPE 5
#define CAN_MESSAGE_ISM_TYPE 6

#define CAN_DEVICE_ADRESS 0

#define ACCEL_DATA_MESSAGE_ADRESS 0x0
#define ACCEL_DATA_MESSAGE_LEN 6

#define GYRO_DATA_MESSAGE_ADRESS 0x1
#define GYRO_DATA_MESSAGE_LEN 6

#define MAGN_DATA_MESSAGE_ADRESS 0x2
#define MAGN_DATA_MESSAGE_LEN 8

#define SOFT_VERSION_MESSAGE_ADRESS 0x4
#define SOFT_VERSION_MESSAGE_LEN 2

#define BOARD_VERSION_MESSAGE_ADRESS 0xF
#define BOARD_VERSION_MESSAGE_LEN 2

#define TEMPERATURE_MESSAGE_ADRESS 0x3
#define TEMPERATURE_MESSAGE_LEN 1

#define GPS_DATA_MESSAGE_ADRESS 0x0
#define GPS_DATA_MESSAGE_LEN 1

#define GSM_DATARX_MESSAGE_ADRESS 0x0
#define GSM_DATARX_MESSAGE_LEN 1

#define GSM_DATATX_MESSAGE_ADRESS 0x1
#define GSM_DATATX_MESSAGE_LEN 1

#define CANTX_FIFO_SIZE 10
#define CANRX_FIFO_SIZE 10

#define ISM_CON_CS  LATEbits.LATE0
#define ISM_DAT_CS  LATEbits.LATE1

#define ISM_RESET   LATDbits.LATD2

#define LED1    LATDbits.LATD0
#define LED2    LATDbits.LATD1

#define GSM_RTS LATAbits.LATA3

#define TRUE    1
#define FALSE   0

