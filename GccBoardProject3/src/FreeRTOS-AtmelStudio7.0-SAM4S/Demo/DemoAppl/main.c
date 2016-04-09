/*******************************************************************************
 * Tracealyzer v3.0.1 Demo Application
 * Percepio AB, www.percepio.com
 *
 * main.c
 *
 * Main function for Tracealyzer demo project.
 *
 * Terms of Use
 * This software is copyright Percepio AB. The recorder library is free for
 * use together with Percepio products. You may distribute the recorder library
 * in its original form, including modifications in trcHardwarePort.c/.h
 * given that these modification are clearly marked as your own modifications
 * and documented in the initial comment section of these source files.
 * This software is the intellectual property of Percepio AB and may not be
 * sold or in other ways commercially redistributed without explicit written
 * permission by Percepio AB.
 *
 * Disclaimer
 * The trace tool and recorder library is being delivered to you AS IS and
 * Percepio AB makes no warranty as to its use or performance. Percepio AB does
 * not and cannot warrant the performance or results you may obtain by using the
 * software or documentation. Percepio AB make no warranties, express or
 * implied, as to noninfringement of third party rights, merchantability, or
 * fitness for any particular purpose. In no event will Percepio AB, its
 * technology partners, or distributors be liable to you for any consequential,
 * incidental or special damages, including any lost profits or lost savings,
 * even if a representative of Percepio AB has been advised of the possibility
 * of such damages, or for any claim by any third party. Some jurisdictions do
 * not allow the exclusion or limitation of incidental, consequential or special
 * damages, or the exclusion of implied warranties or limitations on how long an
 * implied warranty may last, so the above limitations may not apply to you.
 *
 * Tabs are used for indent in this file (1 tab = 4 spaces)
 *
 * Copyright Percepio AB, 2014.
 * www.percepio.com
 ******************************************************************************/

/* Standard includes. */

#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "traceDemoApp.h"
#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#define TaskHandle_t xTaskHandle
#define Timer_t xTimerHandle

// The hardware init code may redefine this. If so, it is called after the trace is started.
void (*startPeriodicInterrupt)(void);

#if (SELECTED_PORT == PORT_ARM_CortexM)

#define _PORT_INIT_EXISTS

void port_init(void);

extern void prvSetupHardware(void);

void port_init(void)
{
	prvSetupHardware();
}
#endif

#if (SELECTED_PORT == PORT_Atmel_AT91SAM7)

#define _PORT_INIT_EXISTS

void port_init(void);

/* Port specific includes */
#include "console.h"

extern void prvSetupHardware(void);

void port_init(void)
{
	prvSetupHardware();
	vStartConsoleTasks(2);
}
#endif

#if (SELECTED_PORT == PORT_Renesas_RX600)

#define _PORT_INIT_EXISTS
/* Port specific includes */
#include "iodefine.h"

void port_init(void);

void port_init(void)
{
	extern void HardwareSetup( void );
	
	/* Renesas provided CPU configuration routine. The clocks are configured in
	here. */
	HardwareSetup();
}

/* The RX port uses this callback function to configure its tick interrupt.
This allows the application to choose the tick interrupt source. */
void vApplicationSetupTimerInterrupt( void )
{
	/* Enable compare match timer 0. */
	MSTP( CMT0 ) = 0;
	
	/* Interrupt on compare match. */
	CMT0.CMCR.BIT.CMIE = 1;
	
	/* Set the compare match value. */
	CMT0.CMCOR = ( unsigned short ) ( ( ( configPERIPHERAL_CLOCK_HZ / configTICK_RATE_HZ ) -1 ) / 8 );
	
	/* Divide the PCLK by 8. */
	CMT0.CMCR.BIT.CKS = 0;
	
	/* Enable the interrupt... */
	_IEN( _CMT0_CMI0 ) = 1;
	
	/* ...and set its priority to the application defined kernel priority. */
	_IPR( _CMT0_CMI0 ) = configKERNEL_INTERRUPT_PRIORITY;
	
	/* Start the timer. */
	CMT.CMSTR0.BIT.STR0 = 1;
}

#elif (SELECTED_PORT == PORT_MICROCHIP_PIC32MX)

/* Hardware specific includes. */
#include "ConfigPerformance.h"

/* Core configuratin fuse settings */
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_2
#pragma config CP = OFF, BWP = OFF, PWP = OFF

/* Additional config fuse settings for other supported processors */
#if defined(__32MX460F512L__)
	#pragma config UPLLEN = OFF
#elif defined(__32MX795F512L__)
	#pragma config UPLLEN = OFF
	#pragma config FSRSSEL = PRIORITY_7
#endif

#define _PORT_INIT_EXISTS

/* Port specific includes */

void port_init(void);

void port_init(void)
{
	/* Configure the hardware for maximum performance. */
	vHardwareConfigurePerformance();

	/* Setup to use the external interrupt controller. */
	vHardwareUseMultiVectoredInterrupts();

	portDISABLE_INTERRUPTS();
}

#elif (SELECTED_PORT == PORT_MICROCHIP_PIC32MZ)

/* Core configuration fuse settings */
#pragma config FMIIEN = OFF, FETHIO = OFF, PGL1WAY = OFF, PMDL1WAY = OFF, IOL1WAY = OFF, FUSBIDIO = OFF
#pragma config FNOSC = SPLL, FSOSCEN = OFF, IESO = OFF, POSCMOD = EC
#pragma config OSCIOFNC = OFF, FCKSM = CSECMD, FWDTEN = OFF, FDMTEN = OFF
#pragma config DMTINTV = WIN_127_128, WDTSPGM = STOP, WINDIS= NORMAL
#pragma config WDTPS = PS1048576, FWDTWINSZ = WINSZ_25, DMTCNT = DMT31
#pragma config FPLLIDIV = DIV_3, FPLLRNG = RANGE_13_26_MHZ, FPLLICLK = PLL_POSC
#pragma config FPLLMULT = MUL_50, FPLLODIV = DIV_2, UPLLFSEL = FREQ_12MHZ, UPLLEN = OFF
#pragma config EJTAGBEN = NORMAL, DBGPER = PG_ALL, FSLEEP = OFF, FECCCON = OFF_UNLOCKED
#pragma config BOOTISA = MIPS32, TRCEN = ON, ICESEL = ICS_PGx2, JTAGEN = OFF, DEBUG = ON
#pragma config CP = OFF
#pragma config_alt FWDTEN=OFF
#pragma config_alt USERID = 0x1234u

#define _PORT_INIT_EXISTS

/* Port specific includes */

void port_init(void);

void port_init(void)
{
	/* Configure the hardware for maximum performance. */
	vHardwareConfigurePerformance();

	/* Setup to use the external interrupt controller. */
	vHardwareUseMultiVectoredInterrupts();

	portDISABLE_INTERRUPTS();
}

#elif (SELECTED_PORT == PORT_Win32)

#define _PORT_INIT_EXISTS

/* Port specific includes */

void stopHook(void);

void stopHook(void)
{
	FILE * f = fopen("out.bin", "wb");	
	fwrite(vTraceGetTraceBuffer(), uiTraceGetTraceBufferSize(), 1, f);
	printf("Stopped! Written to out.bin.\n");
	fclose(f);
	
	system("pause");
	exit(0);
}

void port_init(void);

void port_init(void)
{
	printf("Tracealyzer Demo running...\n");
	vConfigureTimerForRunTimeStats();
	vTraceSetStopHook(stopHook);
}

#endif


void vApplicationMallocFailedHook( void );

void vApplicationMallocFailedHook( void )
{
	vTraceConsoleMessage("\n\rMalloc failed!\n\r");
}

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName );

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName )
{
	(void)pxTask;
	(void)pcTaskName;
	vTraceConsoleMessage("\n\rStack overflow!\n\r");
}


/*
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void )
{
	/* Put the recorder data structure on the heap (malloc), if no 
	static allocation. (See TRACE_DATA_ALLOCATION in trcConfig.h). */

#ifdef _PORT_INIT_EXISTS
	port_init();
#endif	

	vTraceInitTraceData();
	
	if (! uiTraceStart() )
	{
			vTraceConsoleMessage("Could not start recorder!");
	}
	
	vStartDemoApplication();
	
	if (startPeriodicInterrupt != NULL) // Defined in port_init
	{
		startPeriodicInterrupt();
	}
	
	vTaskStartScheduler();
	
	/* Start the scheduler.
	
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called. The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called. If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	

	/* We should never get here as control is now taken by the scheduler. */
	return 0;
}


void vApplicationTickHook( void );
void vApplicationIdleHook( void );

void vApplicationIdleHook( void )
{
#ifdef WIN32
	/* Sleep to reduce CPU load, but don't sleep indefinitely in case there are
	tasks waiting to be terminated by the idle task. */
	Sleep( 5 );
#endif
}

void vApplicationTickHook( void )
{
}

#if (SELECTED_PORT == PORT_MICROCHIP_PIC32MX)

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel. Other exceptions
	should be handled here. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}

#elif (SELECTED_PORT == PORT_MICROCHIP_PIC32MZ)

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile char *pcFileName;
volatile unsigned long ulLineNumber;

	/* Prevent things that are useful to view in the debugger from being
	optimised away. */
	pcFileName = ( char * ) pcFile;
	( void ) pcFileName;
	ulLineNumber = ulLine;

	/* Set ulLineNumber to 0 in the debugger to break out of this loop and
	return to the line that triggered the assert. */
	while( ulLineNumber != 0 )
	{
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
		__asm volatile( "NOP" );
	}
}
/*-----------------------------------------------------------*/

/* This function overrides the normal _weak_ generic handler. */
void _general_exception_handler(void)
{
static enum {
	EXCEP_IRQ = 0, 	/* interrupt */
	EXCEP_AdEL = 4, /* address error exception (load or ifetch) */
	EXCEP_AdES, 	/* address error exception (store) */
	EXCEP_IBE, 		/* bus error (ifetch) */
	EXCEP_DBE, 		/* bus error (load/store) */
	EXCEP_Sys, 		/* syscall */
	EXCEP_Bp, 		/* breakpoint */
	EXCEP_RI, 		/* reserved instruction */
	EXCEP_CpU, 		/* coprocessor unusable */
	EXCEP_Overflow,	/* arithmetic overflow */
	EXCEP_Trap, 	/* trap (possible divide by zero) */
	EXCEP_IS1 = 16,	/* implementation specfic 1 */
	EXCEP_CEU, 		/* CorExtend Unuseable */
	EXCEP_C2E 		/* coprocessor 2 */
} _excep_code;

static unsigned long _epc_code;
static unsigned long _excep_addr;

	asm volatile( "mfc0 %0,$13" : "=r" (_epc_code) );
	asm volatile( "mfc0 %0,$14" : "=r" (_excep_addr) );

	_excep_code = ( _epc_code & 0x0000007C ) >> 2;

	for( ;; )
	{
		/* Examine _excep_code to identify the type of exception. Examine
		_excep_addr to find the address that caused the exception */
		LATHSET = 0x0007;
		Nop();
		Nop();
		Nop();
	}
}
#else

void vAssertCalled( unsigned long ulLine, const char * const pcFileName );

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{	
	(void)ulLine;
	(void)pcFileName;

	taskDISABLE_INTERRUPTS();
	for( ;; );
}

#endif
