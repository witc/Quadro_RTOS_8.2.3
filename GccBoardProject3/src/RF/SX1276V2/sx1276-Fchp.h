/*
 * sx1276_Fchp.h
 *
 * Created: 13.4.2014 0:36:35
 *  Author: JR
 */ 


#ifndef SX1276_FCHP_H_
#define SX1276_FCHP_H_

#include "sx1276-LoRa.h"


void SX1276LoRaSetRFPower( int8_t power,tSX1276LR *SX1276LR  );
void SX1276LoRaSetRFPower_2( int8_t power,tSX1276LR *SX1276LR  );
void SX1276LoRaSetPAOutput( uint8_t outputPin,  tSX1276LR *SX1276LR );
void SX1276LoRaSetPa20dBm( bool enale,  tSX1276LR *SX1276LR );
void SX1276LoRaSetRFFrequency( uint32_t freq,  tSX1276LR *SX1276LR );
void SX1276LoRaSetSpreadingFactor(uint8_t factor,tSX1276LR *SX1276LR );
void SX1276LoRaSetNbTrigPeaks( uint8_t value,tSX1276LR *SX1276LR );
void SX1276LoRaSetErrorCoding(  uint8_t value,tSX1276LR *SX1276LR ) ;
void SX1276LoRaSetPacketCrcOn(  bool enable,tSX1276LR *SX1276LR );
void SX1276LoRaSetSignalBandwidth( uint8_t bw,tSX1276LR *SX1276LR);
void SX1276LoRaSetImplicitHeaderOn(bool enable,tSX1276LR *SX1276LR );
void SX1276LoRaSetSymbTimeout( uint16_t value,tSX1276LR *SX1276LR );
void SX1276LoRaSetPayloadLength( uint8_t value,tSX1276LR *SX1276LR);
void SX1276LoRaSetLowDatarateOptimize( bool enable,tSX1276LR *SX1276LR );







#endif /* SX1276-FCHP_H_ */