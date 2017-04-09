/*
 * myeint.hpp
 *
 *  Created on: Mar 12, 2017
 *      Author: Bhupe
 */

#ifndef L5_APPLICATION_MYEINT_HPP_
#define L5_APPLICATION_MYEINT_HPP_


#include "LPC17xx.h"
#include "tasks.hpp"
#include "printf_lib.h"
#include "io.hpp"
#include "lpc_timers.h"

struct irq_struct
{
	int pin;				  //Port pin configured as interrupt
	void (*funct_ptr)(void);  //function pointer for callback function
}irq[7];   //7 as 7 pins of port 2 available for interrupt on eint3

int i=0, cnt=0;
void sw1_isr_task (void)
{
	LE.toggle(1);// ISR task
	cnt++;
	u0_dbg_printf("\nCount %d",cnt);
	LPC_GPIOINT->IO2IntClr |= (1<<7);
}
void sw2_isr_task (void)
{
	LE.toggle(2);// ISR task
	u0_dbg_put("\nsw2");
	LPC_GPIOINT->IO2IntClr |= (1<<3);
}
void my_IRQHandler (void)
{
//set below if to prevent debounce and adjust the value eg 600 in this function
#if 0
	int old_time=0;
	int new_time =lpc_timer_get_value(lpc_timer0);
    if((new_time-old_time)>600)
#endif
	{
	for(int j=0;j<2;j++) //change to 7 if using all pins
	{
		if ((LPC_GPIOINT->IO2IntStatR & (1 << irq[j].pin)))
		{
			irq[j].funct_ptr();
		}
	}
	}
#if 0
//	old_time=new_time;
#endif
}


class irq_task : public scheduler_task
{

public:
	irq_task(uint8_t priority) : scheduler_task("task", 2000, priority)
{
		/* Nothing to init */
}

	void irq_enable(int pin, void (*callBck_funct_ptr)(void))
	{
		irq[i].pin = pin;
		irq[i].funct_ptr = callBck_funct_ptr;
		LPC_GPIO2->FIODIR &= ~(1<<pin);
		LPC_GPIOINT->IO2IntEnR |= (1<<pin);
		NVIC_EnableIRQ(EINT3_IRQn);
		LPC_GPIOINT->IO2IntClr |= (1<<pin);
		isr_register(EINT3_IRQn,my_IRQHandler);
		i++;
	}
	bool init(void)
	{
		irq_enable(7, sw1_isr_task);
		//irq_enable(3, sw2_isr_task);
		return true;
	}
	bool run(void *p)
	{
		return true;
	}
};


#endif /* L5_APPLICATION_MYEINT_HPP_ */
