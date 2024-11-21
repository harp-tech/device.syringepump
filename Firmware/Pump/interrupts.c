#include "cpu.h"
#include "hwbp_core_types.h"
#include "app_ios_and_regs.h"
#include "app_funcs.h"
#include "hwbp_core.h"

/************************************************************************/
/* Declare application registers                                        */
/************************************************************************/
extern AppRegs app_regs;
extern bool but_reset_pressed;
extern bool but_reset_dir_change;
extern bool running_protocol;
extern bool switch_f_active;
extern bool switch_r_active;
extern void switch_pressed(uint8_t direction);

/************************************************************************/
/* Interrupts from Timers                                               */
/************************************************************************/
// ISR(TCC0_OVF_vect, ISR_NAKED)
// ISR(TCD0_OVF_vect, ISR_NAKED)
// ISR(TCE0_OVF_vect, ISR_NAKED)
// ISR(TCF0_OVF_vect, ISR_NAKED)
// 
// ISR(TCC0_CCA_vect, ISR_NAKED)
// ISR(TCD0_CCA_vect, ISR_NAKED)
// ISR(TCE0_CCA_vect, ISR_NAKED)
// ISR(TCF0_CCA_vect, ISR_NAKED)
// 
// ISR(TCD1_OVF_vect, ISR_NAKED)
// 
// ISR(TCD1_CCA_vect, ISR_NAKED)

/************************************************************************/ 
/* IN00                                                                 */
/************************************************************************/
uint8_t previous_in0;

ISR(PORTB_INT0_vect, ISR_NAKED)
{
	uint8_t aux = read_IN00;

	if((app_regs.REG_DI0_CONFIG & MSK_DI0_CONF) == GM_DI0_SYNC )
	{
		app_regs.REG_INPUT_STATE = aux;
		app_write_REG_INPUT_STATE(&app_regs.REG_INPUT_STATE);
		core_func_send_event(ADD_REG_INPUT_STATE, true);
	}
	
	if((app_regs.REG_DI0_CONFIG & MSK_DI0_CONF) == GM_DI0_RISE_FALL_UPDATE_STEP )
	{
		// transition from low to high
		if(previous_in0 == 0 && aux == 1)
		{
			// generate a STEP
			app_regs.REG_STEP_STATE = aux;
			app_write_REG_STEP_STATE(&app_regs.REG_STEP_STATE);
		}
	}
	
	if((app_regs.REG_DI0_CONFIG & MSK_DI0_CONF) == GM_DI0_RISE_START_PROTOCOL)
	{
		// transition from low to high
		if(previous_in0 == 0 && aux == 1)
			app_regs.REG_START_PROTOCOL = 1;
		else
			app_regs.REG_START_PROTOCOL = 0;
			
		app_write_REG_START_PROTOCOL(&app_regs.REG_START_PROTOCOL);
	}
	
	previous_in0 = aux;
	
	reti();
}


/************************************************************************/ 
/* SW_F, SW_R, TYPE0 & TYPE1                                            */
/************************************************************************/
extern void clear_sw_f();
extern void clear_sw_r();

ISR(PORTC_INT0_vect, ISR_NAKED)
{
	if(!(read_SW_F))
	{
		if(switch_f_active)
		{
			app_regs.REG_SW_FORWARD_STATE = 0;
			if(app_regs.REG_EVT_ENABLE & B_EVT_SW_FORWARD_STATE)
				core_func_send_event(ADD_REG_SW_FORWARD_STATE, true);
			if((app_regs.REG_DO0_CONFIG & MSK_OUT0_CONF) == GM_OUT0_SWLIMIT)
				clr_OUT00;
		}
		clear_sw_f();
	}
	
	if(!(read_SW_R))
	{
		if(switch_r_active)
		{
			app_regs.REG_SW_REVERSE_STATE = 0;
			if(app_regs.REG_EVT_ENABLE & B_EVT_SW_REVERSE_STATE)
				core_func_send_event(ADD_REG_SW_REVERSE_STATE, true);
			if((app_regs.REG_DO0_CONFIG & MSK_OUT0_CONF) == GM_OUT0_SWLIMIT)
				clr_OUT00;
		}
		clear_sw_r();
	}

	reti();
}

/************************************************************************/ 
/* EN_DRIVER_UC & BUT_PUSH & BUT_PULL & BUT_RESET                       */
/************************************************************************/
extern uint8_t but_reset_counter_ms;
extern void clear_but_push();
extern void clear_but_pull();

ISR(PORTD_INT0_vect, ISR_NAKED)
{
	if(!(read_BUT_PUSH))
	{
		clear_but_push();
	}
	
	if(!(read_BUT_PULL))
	{
		clear_but_pull();
	}
	
	if(!(read_BUT_RESET))
	{
		but_reset_counter_ms = 25;
	}
	
	if(!(read_EN_DRIVER_UC))
	{
		set_BUF_EN;
		set_EN_DRIVER;

		// change STEP, DIR and MSx as tristate
		io_pin2in(&PORTA, 0, PULL_IO_TRISTATE, SENSE_IO_NO_INT_USED);			// STEP
		io_pin2in(&PORTA, 1, PULL_IO_TRISTATE, SENSE_IO_NO_INT_USED);          // DIR
		io_pin2in(&PORTA, 2, PULL_IO_TRISTATE, SENSE_IO_NO_INT_USED);          // MS1
		io_pin2in(&PORTA, 3, PULL_IO_TRISTATE, SENSE_IO_NO_INT_USED);          // MS2
		io_pin2in(&PORTA, 4, PULL_IO_TRISTATE, SENSE_IO_NO_INT_USED);          // MS3
	}
	else
	{
		clr_BUF_EN;
		//clr_EN_DRIVER;
		
		// change STEP, DIR and MSx to default mode
		io_pin2out(&PORTA, 0, OUT_IO_DIGITAL, IN_EN_IO_EN);                  // STEP
		io_pin2out(&PORTA, 1, OUT_IO_DIGITAL, IN_EN_IO_EN);                  // DIR
		io_pin2out(&PORTA, 2, OUT_IO_DIGITAL, IN_EN_IO_DIS);                 // MS1
		io_pin2out(&PORTA, 3, OUT_IO_DIGITAL, IN_EN_IO_DIS);                 // MS2
		io_pin2out(&PORTA, 4, OUT_IO_DIGITAL, IN_EN_IO_DIS);                 // MS3
	}
	
	reti();
}


