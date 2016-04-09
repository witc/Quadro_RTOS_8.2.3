/*
 * SX1276.h
 *
 * Created: 12.4.2014 23:21:53
 *  Author: JR
 */ 

#ifndef SX1276_def
#define SX1276_def

#include <string.h>
#include <stdint.h>
#include <stdbool.h>


void SX1276Init( void );
void SX1276Reset( void );
void SX1276SetLoRaOn(void);
void SX1276SetLoRaOFF(void);
void SX1276SetOpMode( uint8_t opMode );
uint8_t SX1276GetOpMode( void );
double SX1276ReadRssi( void );
uint8_t SX1276ReadRxGain( void );
uint8_t SX1276GetPacketRxGain( void );
int8_t SX1276GetPacketSnr( void );
double SX1276GetPacketRssi( void );
uint32_t SX1276GetPacketAfc( void );
void SX1276StartRx( void );
void SX1276GetRxPacket( void *buffer, uint16_t *size );
void SX1276SetTxPacket( const void *buffer, uint16_t size );
uint8_t SX1276GetRFState( void );
void SX1276SetRFState( uint8_t state );
uint32_t SX1276Process( void );


















#endif