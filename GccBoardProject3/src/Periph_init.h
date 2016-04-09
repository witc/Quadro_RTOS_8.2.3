/*
 * Periph_init.h
 *
 * Created: 28.03.2016 21:52:51
 *  Author: J
 */ 


#ifndef PERIPH_INIT_H_
#define PERIPH_INIT_H_

void interrupt_init(void);
void bus_init(void);
void dma_twi_init(void);
void configure_console(void);


#define CONF_ILI9341_CLOCK_SPEED   1000000UL



#endif /* PERIPH_INIT_H_ */