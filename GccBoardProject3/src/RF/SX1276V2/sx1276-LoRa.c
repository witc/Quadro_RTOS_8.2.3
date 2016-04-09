
#include <asf.h>
#include <string.h>
//#define  F_CPU 32000000UL
#include "sx1276-Hal.h"
#include "sx1276.h"
#include "sx1276-LoRa.h"
#include "sx1276-Fchp.h"
#include "Main.h"
#include "FreeRTOS_V_8_Header.h"

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET                                 -137.0
#define NOISE_ABSOLUTE_ZERO                         -174.0
#define NOISE_FIGURE                                6.0


#define MODULE_SX1276RF1IAS							0
#define MODULE_SX1276RF1KAS							0
#define MODULE_SX1276RF1JAS							1


/*!
 * Precomputed signal bandwidth log values
 * Used to compute the Packet RSSI value.
 */
const double SignalBwLog[] =
{
	3.8927900303521316335038277369285,  // 7.8 kHz
	4.0177301567005500940384239336392,  // 10.4 kHz
	4.193820026016112828717566631653,   // 15.6 kHz
	4.31875866931372901183597627752391, // 20.8 kHz
	4.4948500216800940239313055263775,  // 31.2 kHz
	4.6197891057238405255051280399961,  // 41.6 kHz
	4.795880017344075219145044421102,   // 62.5 kHz
	5.0969100130080564143587833158265,  // 125 kHz
	5.397940008672037609572522210551,   // 250 kHz
	5.6989700043360188047862611052755   // 500 kHz
};

const double RssiOffsetLF[] =
{   // These values need to be specify in the Lab
	-155.0,
	-155.0,
	-155.0,
	-155.0,
	-155.0,
	-155.0,
	-155.0,
	-155.0,
	-155.0,
	-155.0,
};

const double RssiOffset[] =
{   // These values need to be specify in the Lab
	-150.0,
	-150.0,
	-150.0,
	-150.0,
	-150.0,
	-150.0,
	-150.0,
	-150.0,
	-150.0,
	-150.0,
};

// Default settings
tLoRaSettings LoRaSettings =
{
	869525000,        // RFFrequency
	0,               // Power
	9,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
	// 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
	//pro mensi nez 62,5 je dulezite pouzit TCXO as f reference
	8,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
	1,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
	1,				  // CrcOn [0: OFF, 1: ON]
	1,				 // ImplicitHeaderOn [0: OFF, 1: ON]	//pokud znám CRC, delku paketu na RX strane - pouziji implicit - zkratka tam neni
	1,                // RxSingleOn [0: Continuous, 1 Single]
	0,                // FreqHopOn [0: OFF, 1: ON]
	4,                // HopPeriod Hops every frequency hopping period symbols
	200,              // TxPacketTimeout
	700,              // RxPacketTimeout
	8,              // PayloadLength (used for implicit header mode)
};


 tSX1276LR  SX1276LR;


void SX1276LoRaInit( void )
{
   
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	
	SX1276LoRaSetDefaults( );
		
	SX1276ReadBuffer( 0x1,(uint8_t*)&SX1276LR, 0x70 - 1 );
       
    // set the RF settings 
    SX1276LoRaSetRFFrequency( LoRaSettings.RFFrequency,&SX1276LR );
    SX1276LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor,&SX1276LR ); // SF6 only operates in implicit header mode.
    SX1276LoRaSetErrorCoding( LoRaSettings.ErrorCoding,&SX1276LR );
    SX1276LoRaSetPacketCrcOn( LoRaSettings.CrcOn,&SX1276LR );
    SX1276LoRaSetSignalBandwidth( LoRaSettings.SignalBw,&SX1276LR );
    
    SX1276LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn,&SX1276LR );
    SX1276LoRaSetSymbTimeout(LoRaSettings.RxPacketTimeout,&SX1276LR );	// 
    SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength,&SX1276LR );	//
	 
	 
	  if (LoRaSettings.SpreadingFactor>=11)
	  {
		  SX1276LoRaSetLowDatarateOptimize( true,&SX1276LR );	//pouze pro SF11 a SF12
	  }else
	  {
		  
		  SX1276LoRaSetLowDatarateOptimize( false,&SX1276LR );	//pouze pro SF11 a SF12
	  }
    
	
	
	/*AGC AUTO*/
 	SX1276Read( REG_LR_MODEMCONFIG3,&SX1276LR.RegModemConfig3 );
 	SX1276LR.RegModemConfig3|=0x4;
 	SX1276Write( REG_LR_MODEMCONFIG3,SX1276LR.RegModemConfig3 );
	
#if( ( MODULE_SX1276RF1IAS == 1 ) || ( MODULE_SX1276RF1KAS == 1 ) )
	
	if( LoRaSettings.RFFrequency > 860000000 )
	{
		SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO,&SX1276LR );
		SX1276LoRaSetPa20dBm( false,&SX1276LR );
		//LoRaSettings.Power = 14;
		SX1276LoRaSetRFPower( LoRaSettings.Power,&SX1276LR );
	}
	else
	{
		SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST,&SX1276LR );
		SX1276LoRaSetPa20dBm( true,&SX1276LR );
		LoRaSettings.Power = 20;
		SX1276LoRaSetRFPower( LoRaSettings.Power,&SX1276LR );
	}
	
#elif( MODULE_SX1276RF1JAS == 1 )

	if( LoRaSettings.RFFrequency > 860000000 )
	{
		SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST,&SX1276LR );
		SX1276LoRaSetPa20dBm( true,&SX1276LR );
		LoRaSettings.Power = 20;
		SX1276LoRaSetRFPower( LoRaSettings.Power,&SX1276LR );
	}
	else
	{
		SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO,&SX1276LR );
		SX1276LoRaSetPa20dBm( false,&SX1276LR );
		LoRaSettings.Power = 14;
		SX1276LoRaSetRFPower( LoRaSettings.Power,&SX1276LR );
	}
	
#else
	#error "Not defined Module for SX1276"
#endif

    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );


}

void SX1276LoRaSetDefaults( void )
{	
	//See app note
	if (LoRaSettings.SignalBw<9)	//mensi nez 500
	{
		SX1276Read( 0x31, &SX1276LR.RegTestReserved31 );
		// Sets IF frequency selection manual
		SX1276LR.RegTestReserved31 &= 0x7F; // pro pasmo krome 5000 khz musime vycistiti bit 7
		SX1276LR.RegTestReserved31 &=~ 0x80; //7 bit vznulovat
		SX1276Write( 0x31, SX1276LR.RegTestReserved31 );
		
	}else
	{
		SX1276Read( 0x31, &SX1276LR.RegTestReserved31 );
		// Sets IF frequency selection manual
		SX1276LR.RegTestReserved31 &= 0x7F; // pro pasmo krome 5000 khz musime vycistiti bit 7
		SX1276LR.RegTestReserved31 |= 0x80; //7 bit set
		SX1276Write( 0x31, SX1276LR.RegTestReserved31 );
		SX1276Write( 0x36, 0x2);	//See Erata
		SX1276Write( 0x3a, 0x64);	//See Erata
	}
	
	if(LoRaSettings.SignalBw==0)   SX1276Write( 0x2F, 0x48 );
	if((LoRaSettings.SignalBw>=0)&&(LoRaSettings.SignalBw<=5))   SX1276Write( 0x2F, 0x44 );
	if((LoRaSettings.SignalBw>=6)&&(LoRaSettings.SignalBw<=8))   SX1276Write( 0x2F, 0x40 );
	
	
	//	SX1276Write( 0x2F, 0x40 );
	SX1276Read( 0x30, &SX1276LR.RegTestReserved31 );
	SX1276LR.RegTestReserved31&=0xFE;
	SX1276Write( 0x30, SX1276LR.RegTestReserved31 );
}

void SX1276LoRaReset( void )
{
        
    SX1276SetReset( RADIO_RESET_ON );
    
    // Wait 1ms
    vTaskDelay(10/portTICK_RATE_MS)    ;

    SX1276SetReset( RADIO_RESET_OFF );
    
    // Wait 6ms
    vTaskDelay(10/portTICK_RATE_MS)    ;

}

void SX1276LoRaSetOpMode( uint8_t opMode )
{	
		//static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
		bool antennaSwitchTxOn = false;

		//opModePrev = SX1276LR.RegOpMode & ~RFLR_OPMODE_MASK;


		if( opMode == RFLR_OPMODE_TRANSMITTER )
		{
			antennaSwitchTxOn = true;
		}
		else
		{
			antennaSwitchTxOn = false;
		}

		RXTX( antennaSwitchTxOn ); // Antenna switch control

		SX1276LR.RegOpMode = ( SX1276LR.RegOpMode & RFLR_OPMODE_MASK ) | opMode;
		
		SX1276Read( REG_LR_OPMODE, &SX1276LR.RegOpMode );
		
		SX1276LR.RegOpMode = ( SX1276LR.RegOpMode & RFLR_OPMODE_MASK ) | opMode;
		SX1276Write( REG_LR_OPMODE, SX1276LR.RegOpMode );
}

// uint8_t SX1276LoRaGetOpMode( void )
// {
//     SX1276Read( REG_LR_OPMODE, &SX1276LR->RegOpMode );
//     
//     return SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
// }

// uint8_t SX1276LoRaReadRxGain( void )
// {
//     SX1276Read( REG_LR_LNA, &SX1276LR->RegLna );
//     return( SX1276LR->RegLna >> 5 ) & 0x07;
// }

double SX1276LoRaReadRssi( void )
{
    // Reads the RSSI value
    SX1276Read( REG_LR_RSSIVALUE, &SX1276LR.RegRssiValue );

    return RssiOffset[LoRaSettings.SignalBw] + ( double )SX1276LR.RegRssiValue;
}

uint8_t SX1276LoRaGetPacketRxGain( void )
{
    //return RxGain;
    return 0;
	
}

int8_t SX1276LoRaGetPacketSnr( void )
{
   // return RxPacketSnrEstimate;
    return 0;
   
}

double SX1276LoRaGetPacketRssi( void )
{
  //  return RxPacketRssiValue;
    return 0;
  
}

