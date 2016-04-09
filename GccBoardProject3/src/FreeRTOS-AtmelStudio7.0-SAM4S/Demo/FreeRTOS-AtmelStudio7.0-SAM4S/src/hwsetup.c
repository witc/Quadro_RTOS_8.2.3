#include <asf.h>

#include "gpio.h"
#include "asf/sam/drivers/tc/tc.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "trcUser.h"


void prvSetupHardware( void );



#define USE_TC_INT 0

/******************************************************************************
 * ISR stuff for Atmel SAM4S
 *****************************************************************************/

#if (USE_TC_INT == 1)

#define setup_tc0(freq, prio) setup_tc(freq, ID_TC0, TC0, prio, 0)

void setup_tc(int tc_freq, IRQn_Type id_tc, void* tc, int prio, int chn);

volatile uint32_t tc0_counter = 0;

volatile uint32_t dwtcyc;
volatile uint32_t dwtcyc_last;
volatile uint32_t dwt_diff;

extern void (*startPeriodicInterrupt)(void);

#define TC_ISR_ID 4
#define TC_ISR_PRIO configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY

volatile int dummy;
void TC0_Handler (void)
{	
	vTraceStoreISRBegin(TC_ISR_ID);
	
	tc_stop(TC0, 0);	            // Must be stopped to clear the status reg
	dummy = tc_get_status(TC0, 0);  // Clear the status register
	tc_start(TC0, 0);
	vTraceStoreISREnd(0);	
}

void setup_tc(int tc_freq, IRQn_Type id_tc, void* tc, int prio, int chn)
{
	
	uint32_t ul_div;
	uint32_t ul_tcclks;
	static uint32_t ul_sysclk;

	/* Get system clock. */
	ul_sysclk = sysclk_get_cpu_hz();


	/* Configure PMC. */
	pmc_enable_periph_clk(id_tc);

	tc_find_mck_divisor(tc_freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);

	tc_init(tc, chn, ul_tcclks | TC_CMR_CPCTRG);

	tc_write_rc(tc, chn, (ul_sysclk / ul_div) / tc_freq);

	/* Configure and enable interrupt on RC compare. */
	NVIC_EnableIRQ(id_tc);
	NVIC_SetPriority(id_tc, prio);
	
	tc_enable_interrupt(tc, chn, TC_IER_CPCS);
}

void prvStartInt(void);

void prvStartInt(void)
{
	vTraceSetISRProperties(TC_ISR_ID, "TC ISR", TC_ISR_PRIO);
	tc_start(TC0, 0);
}

#endif

void prvSetupHardware( void )
{	
	sysclk_init();
	board_init();
		
	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping( 0 );
		
	pmc_enable_periph_clk(ID_PIOA); // Needed for PIO input

#if (USE_TC_INT == 1)
	setup_tc0(40, TC_ISR_PRIO);	
	startPeriodicInterrupt = prvStartInt;
#endif	
	
}
