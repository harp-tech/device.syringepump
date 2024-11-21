#include "hwbp_core.h"
#include "hwbp_core_regs.h"
#include "hwbp_core_types.h"

#include "app.h"
#include "app_funcs.h"
#include "app_ios_and_regs.h"

#define F_CPU 32000000
#include <util/delay.h>

/************************************************************************/
/* Declare application registers                                        */
/************************************************************************/
extern AppRegs app_regs;
extern uint8_t app_regs_type[];
extern uint16_t app_regs_n_elements[];
extern uint8_t *app_regs_pointer[];
extern void (*app_func_rd_pointer[])(void);
extern bool (*app_func_wr_pointer[])(void*);

/************************************************************************/
/* Initialize app                                                       */
/************************************************************************/
static const uint8_t default_device_name[] = "Pump";

void hwbp_app_initialize(void)
{
    /* Define versions */
    uint8_t hwH = 1;
    uint8_t hwL = 1;
    uint8_t fwH = 1;
    uint8_t fwL = 1;
    uint8_t ass = 0;
    
   	/* Start core */
    core_func_start_core(
        1296,
        hwH, hwL,
        fwH, fwL,
        ass,
        (uint8_t*)(&app_regs),
        APP_NBYTES_OF_REG_BANK,
        APP_REGS_ADD_MAX - APP_REGS_ADD_MIN + 1,
        default_device_name,
		false,	// The device is _not_ able to repeat the harp timestamp clock
		false,	// The device is _not_ able to generate the harp timestamp clock
		0		// Default timestamp offset
    );
}

/************************************************************************/
/* Handle if a catastrophic error occur                                 */
/************************************************************************/
void core_callback_catastrophic_error_detected(void)
{
	clr_STEP;
	clr_DIR;
	clr_MS1;
	clr_MS2;
	clr_MS3;
	clr_EN_DRIVER;
	clr_SLEEP;
	clr_RESET;
	
	clr_OUT00;
	clr_OUT01;
	clr_BUF_EN;
}

/************************************************************************/
/* User functions                                                       */
/************************************************************************/
#define DIR_FORWARD 1
#define DIR_REVERSE 0

uint16_t inactivity_counter = 0;

/* Buttons */
uint8_t but_push_counter_ms = 0;
uint16_t but_long_push_counter_ms = 0;
bool but_push_long_press = false;

uint8_t but_pull_counter_ms = 0;
uint16_t but_long_pull_counter_ms = 0;
bool but_pull_long_press = false;

uint8_t but_reset_counter_ms = 0;
bool but_reset_pressed = false;
bool but_reset_dir_change = false;

/* Switches */
bool switch_f_active = false;
bool switch_r_active = false;
uint8_t sw_f_counter_ms = 0;
uint8_t sw_r_counter_ms = 0;

uint8_t curr_dir = DIR_FORWARD;
uint8_t prev_dir = DIR_FORWARD;
uint16_t step_period_counter = 0;

bool running_protocol = false;
uint16_t prot_remaining_steps = 0;
uint16_t prot_step_period = 0;


void stop_and_reset_protocol()
{
	running_protocol = false;
	//note: + 1 because it starts counting from 1
	prot_remaining_steps = app_regs.REG_PROTOCOL_NUMBER_STEPS + 1;
	prot_step_period = app_regs.REG_PROTOCOL_PERIOD * 2;
	app_regs.REG_START_PROTOCOL = 0;
	
	// revert direction
	app_write_REG_DIR_STATE(&prev_dir);
}

void switch_pressed(uint8_t direction)
{
	if(direction == DIR_FORWARD)
	{
		switch_f_active = true;
		app_regs.REG_SW_FORWARD_STATE = 1;
	}
	
	if(direction == DIR_REVERSE)
	{
		switch_r_active = true;
		app_regs.REG_SW_REVERSE_STATE = 1;
	}
	
	if(curr_dir == direction)
	{
		stop_and_reset_protocol();
		step_period_counter = 0;
		but_reset_pressed = false;
		but_reset_dir_change = false;
	}
}

void take_step(uint8_t direction)
{
	inactivity_counter = 0;
	
	app_regs.REG_DIR_STATE = direction;
	app_regs.REG_STEP_STATE = 1;
	app_write_REG_DIR_STATE(&app_regs.REG_DIR_STATE);
	app_write_REG_STEP_STATE(&app_regs.REG_STEP_STATE);
}

void clear_step()
{
	// FIXME: this is because it is being called too many times and we don't want events every time
	if(app_regs.REG_STEP_STATE == 0)
		return;

	app_regs.REG_STEP_STATE = 0;

	clr_STEP;
	if((app_regs.REG_DO1_CONFIG & MSK_DI0_CONF) == GM_OUT1_STEP_STATE)
	{
		clr_OUT01;
	}
	
	app_write_REG_STEP_STATE(&app_regs.REG_STEP_STATE);
}

void clear_but_push()
{
	but_push_counter_ms = 25;
	but_long_push_counter_ms = 500;
	but_push_long_press = false;
}

void clear_but_pull()
{
	but_pull_counter_ms = 25;
	but_long_pull_counter_ms = 500;
	but_pull_long_press = false;
}

/* Switches */ 
extern void clear_sw_f()
{			
	switch_f_active = false;
	sw_f_counter_ms = 50;	
}

extern void clear_sw_r()
{
	switch_r_active = false;
	sw_r_counter_ms = 50;
}

/************************************************************************/
/* Initialization Callbacks                                             */
/************************************************************************/
void core_callback_define_clock_default(void) {}
	
void core_callback_initialize_hardware(void)
{
	/* Initialize IOs */
	/* Don't delete this function!!! */
	init_ios();
	
	// TODO: find out if this should be done here or if it is enough to set them on core_callback_registers_were_reinitialized
	/* Initialize hardware */
	clr_BUF_EN;
	set_DIR;
	clr_MS1;
	clr_MS2;
	clr_MS3;
	clr_SLEEP;
	
	// RESET -> clear, wait 10ms, set
	set_RESET;
	_delay_ms(10);
	clr_RESET;
	
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
		clr_EN_DRIVER;
	}
	
	clear_sw_f();
	clear_sw_r();
}

void core_callback_reset_registers(void)
{
	/* Initialize registers */
	app_regs.REG_ENABLE_MOTOR_DRIVER = B_MOTOR_ENABLE;
	app_regs.REG_START_PROTOCOL = B_START_PROTOCOL;
	app_regs.REG_PROTOCOL_STATE = B_PROTOCOL_STATE;
	app_regs.REG_SET_DOS |= (B_SET_DO0 | B_SET_DO1);
	app_regs.REG_CLEAR_DOS |= (B_CLR_DO0 | B_CLR_DO1);
	app_regs.REG_DO0_CONFIG = GM_OUT0_SOFTWARE;
	app_regs.REG_DO1_CONFIG = GM_OUT1_SOFTWARE;
	app_regs.REG_DI0_CONFIG = GM_DI0_SYNC;
	app_regs.REG_MOTOR_MICROSTEP = GM_STEP_FULL;
	
	app_regs.REG_PROTOCOL_DIRECTION = DIR_FORWARD;
	app_regs.REG_PROTOCOL_NUMBER_STEPS = 15;
	app_regs.REG_PROTOCOL_FLOWRATE = 0.5;
	app_regs.REG_PROTOCOL_PERIOD = 10;
	app_regs.REG_PROTOCOL_VOLUME = 0.5;
	app_regs.REG_PROTOCOL_TYPE = 0;
	// TODO: missing calibration values
	
	app_regs.REG_EVT_ENABLE = (B_EVT_STEP_STATE | B_EVT_DIR_STATE | B_EVT_SW_FORWARD_STATE | B_EVT_SW_REVERSE_STATE | B_EVT_INPUT_STATE | B_EVT_PROTOCOL_STATE);
}

void core_callback_registers_were_reinitialized(void)
{
	/* Update registers if needed */
	app_regs.REG_ENABLE_MOTOR_DRIVER = 0;
	app_regs.REG_START_PROTOCOL = 0;
	
	app_regs.REG_STEP_STATE = 0;
	app_regs.REG_DIR_STATE = 0;
	app_regs.REG_SW_FORWARD_STATE = 0;
	app_regs.REG_SW_REVERSE_STATE = 0;
	app_regs.REG_INPUT_STATE = 0;
	
	app_regs.REG_SET_DOS = 0;
	app_regs.REG_CLEAR_DOS = 0;
	
	stop_and_reset_protocol();
	
	/* Update config */
	app_write_REG_DO0_CONFIG(&app_regs.REG_DO0_CONFIG);
	app_write_REG_DO1_CONFIG(&app_regs.REG_DO1_CONFIG);
	app_write_REG_DI0_CONFIG(&app_regs.REG_DI0_CONFIG);

	app_write_REG_MOTOR_MICROSTEP(&app_regs.REG_MOTOR_MICROSTEP);
	clr_EN_DRIVER;
	
	// update switches initial state
	if(read_SW_F)
		switch_pressed(DIR_FORWARD);
	
	if(read_SW_R)
		switch_pressed(DIR_REVERSE);
}

/************************************************************************/
/* Callbacks: Visualization                                             */
/************************************************************************/
void core_callback_visualen_to_on(void)
{
	/* Update visual indicators */
	
}

void core_callback_visualen_to_off(void)
{
	/* Clear all the enabled indicators */
	
}

/************************************************************************/
/* Callbacks: Change on the operation mode                              */
/************************************************************************/
void core_callback_device_to_standby(void) {}
void core_callback_device_to_active(void) {}
void core_callback_device_to_enchanced_active(void) {}
void core_callback_device_to_speed(void) {}

/************************************************************************/
/* Callbacks: 1 ms timer                                                */
/************************************************************************/

#define STEP_PERIOD_HALF_MILLISECONDS 2
#define STEP_UPTIME_HALF_MILLISECONDS 1
#define INACTIVITY_TIME 30000

void core_callback_t_before_exec(void) 
{
	//FIXME: ugly fix for clearing long button presses if still active
	if(read_BUT_PUSH)
		clear_but_push();
	if(read_BUT_PULL)
		clear_but_pull();

	if(running_protocol)
	{
		++step_period_counter;
		if(step_period_counter == STEP_UPTIME_HALF_MILLISECONDS)
			clear_step();
			
		if(step_period_counter >= prot_step_period)
		{
			step_period_counter = 0;
			// make step if there are still steps remaining in the current running protocol
			if(--prot_remaining_steps)
			{
				app_regs.REG_DIR_STATE = curr_dir;
				app_regs.REG_STEP_STATE = 1;
				app_write_REG_DIR_STATE(&app_regs.REG_DIR_STATE);
				app_write_REG_STEP_STATE(&app_regs.REG_STEP_STATE);
			}
			else
			{
				// we reached the end, lets stop everything and reset variables
				app_regs.REG_START_PROTOCOL = 0;
				app_write_REG_START_PROTOCOL(&app_regs.REG_START_PROTOCOL);
			}
		}
	}
	else
	{
		// normal counting, outside of protocol
		++step_period_counter;
		if(step_period_counter == STEP_UPTIME_HALF_MILLISECONDS)
			clear_step();
		
		if(step_period_counter == STEP_PERIOD_HALF_MILLISECONDS)
		{
			step_period_counter = 0;
			
			if(but_reset_pressed)
			{
				// change direction once and continue steps
				// note: this flag is required so that the DIR change only happens near the first reset step
				if(!but_reset_dir_change)
				{
					app_regs.REG_DIR_STATE = !app_regs.REG_DIR_STATE;
					app_write_REG_DIR_STATE(&app_regs.REG_DIR_STATE);
					but_reset_dir_change = true;
				}
				
				take_step(curr_dir);
				
				// if reset was pressed, we don't really want to do anything else
				return;
			}
			
			// prevent steps on long press only if switch on the same direction is active
			if(but_push_long_press && !switch_f_active)
			{
				app_regs.REG_DIR_STATE = DIR_FORWARD;
				app_regs.REG_STEP_STATE = 1;
				app_write_REG_DIR_STATE(&app_regs.REG_DIR_STATE);
				app_write_REG_STEP_STATE(&app_regs.REG_STEP_STATE);
			}
			
			if(but_pull_long_press && !switch_r_active)
			{
				app_regs.REG_DIR_STATE = DIR_REVERSE;
				app_regs.REG_STEP_STATE = 1;
				app_write_REG_DIR_STATE(&app_regs.REG_DIR_STATE);
				app_write_REG_STEP_STATE(&app_regs.REG_STEP_STATE);
			}
		}
	}
}
void core_callback_t_after_exec(void) {}
void core_callback_t_new_second(void)
{
	if((app_regs.REG_DO1_CONFIG & MSK_OUT1_CONF) == GM_OUT1_DATA_SEC)
	{
		tgl_OUT01;
	}	
}
void core_callback_t_500us(void) {}
	
void core_callback_t_1ms(void) 
{
	// disable motor if there's no activity for a while
	++inactivity_counter;
	if(inactivity_counter == INACTIVITY_TIME)
	{
		app_write_REG_ENABLE_MOTOR_DRIVER(0);
		inactivity_counter = 0;
	}
	
	/* handle switches */
	/* De-bounce Switch FORWARD */
	if(sw_f_counter_ms)
	{
		if(read_SW_F)
		{
			if(!--sw_f_counter_ms)
			{
				switch_pressed(DIR_FORWARD);
						
				if(app_regs.REG_EVT_ENABLE & B_EVT_SW_FORWARD_STATE)
					core_func_send_event(ADD_REG_SW_FORWARD_STATE, true);
	
				if((app_regs.REG_DO0_CONFIG & MSK_OUT0_CONF) == GM_OUT0_SWLIMIT)
					set_OUT00;
				//{
					//if(read_SW_F)
						set_OUT00;
					//else
						//clr_OUT00;
				//}
			}
		}
	}
	
	/* De-bounce Switch REVERSE */
	if(sw_r_counter_ms)
	{
		if(read_SW_R)
		{
			if(!--sw_r_counter_ms)
			{
				switch_pressed(DIR_REVERSE);

				if(app_regs.REG_EVT_ENABLE & B_EVT_SW_REVERSE_STATE)
					core_func_send_event(ADD_REG_SW_REVERSE_STATE, true);
					
				if((app_regs.REG_DO0_CONFIG & MSK_OUT0_CONF) == GM_OUT0_SWLIMIT)
					set_OUT00;
				//{
					//if(read_SW_R)
						//set_OUT00;
					//else
						//clr_OUT00;
				//}
			}
		}
	}
	
	/* handle buttons */
	/* De-bounce PUSH button */
	if(but_push_counter_ms)
	{
		if (!(read_BUT_PUSH))
		{
			if (!--but_push_counter_ms)
			{
				// single press
				if(!running_protocol)
				{
					//FIXME: this enters here twice on every button press... why?
					// takes step except on active switch on same direction and reset was pressed
					if(!switch_f_active && !but_reset_pressed)
					{
						take_step(DIR_FORWARD);
					}
					
					// if reset is pressed and going in opposite direction, it should stop reset steps
					if(but_reset_pressed && curr_dir == DIR_REVERSE)
					{
						but_reset_pressed = false;
						but_reset_dir_change = false;
					}
				}
			}
		}
		else
		{
			clear_but_push();
		}
	}
	
	// detect PUSH button long press
	if(!but_push_counter_ms && but_long_push_counter_ms)
	{
		if (!(read_BUT_PUSH))
		{
			// long press detection
			if(!--but_long_push_counter_ms)
			{
				but_push_long_press = true;
			}
		}
		else
		{
			clear_but_push();
		}
	}
	
	/* De-bounce PULL button */
	if (but_pull_counter_ms)
	{
		if (!(read_BUT_PULL))
		{
			if (!--but_pull_counter_ms)
			{
				// single press
				if(!running_protocol)
				{
					// takes step except on switch on same direction is active and reset was pressed
					if(!switch_r_active && !but_reset_pressed)
					{
						take_step(DIR_REVERSE);
					}
					
					// if reset is pressed and going in opposite direction, it should stop reset steps
					if(but_reset_pressed && curr_dir == DIR_FORWARD)
					{
						but_reset_pressed = false;
						but_reset_dir_change = false;
					}
				}
			}
		}
		else
		{
			clear_but_pull();
		}
	}
	
	// detect PULL button long press
	if(!but_pull_counter_ms && but_long_pull_counter_ms)
	{
		if (!(read_BUT_PULL))
		{
			// long press detection
			if(!--but_long_pull_counter_ms)
			{
				but_pull_long_press = true;
			}
		}
		else
		{
			clear_but_pull();
		}
	}
	
	/* De-bounce RESET button */
	if (but_reset_counter_ms)
	{
		if (!(read_BUT_RESET))
		{
			if (!--but_reset_counter_ms)
			{
				but_reset_pressed = true;
				stop_and_reset_protocol();
				// if 0 and the period is too long, it will only stop after that time
				step_period_counter = 0;
			}
		}
		else
		{
			but_reset_counter_ms = 0;
		}
	}
}

/************************************************************************/
/* Callbacks: clock control                                              */
/************************************************************************/
void core_callback_clock_to_repeater(void) {}
void core_callback_clock_to_generator(void) {}
void core_callback_clock_to_unlock(void) {}
void core_callback_clock_to_lock(void) {}
	
/************************************************************************/
/* Callbacks: uart control                                              */
/************************************************************************/
void core_callback_uart_rx_before_exec(void) {}
void core_callback_uart_rx_after_exec(void) {}
void core_callback_uart_tx_before_exec(void) {}
void core_callback_uart_tx_after_exec(void) {}
void core_callback_uart_cts_before_exec(void) {}
void core_callback_uart_cts_after_exec(void) {}

/************************************************************************/
/* Callbacks: Read app register                                         */
/************************************************************************/
bool core_read_app_register(uint8_t add, uint8_t type)
{
	/* Check if it will not access forbidden memory */
	if (add < APP_REGS_ADD_MIN || add > APP_REGS_ADD_MAX)
		return false;
	
	/* Check if type matches */
	if (app_regs_type[add-APP_REGS_ADD_MIN] != type)
		return false;
	
	/* Receive data */
	(*app_func_rd_pointer[add-APP_REGS_ADD_MIN])();	

	/* Return success */
	return true;
}

/************************************************************************/
/* Callbacks: Write app register                                        */
/************************************************************************/
bool core_write_app_register(uint8_t add, uint8_t type, uint8_t * content, uint16_t n_elements)
{
	/* Check if it will not access forbidden memory */
	if (add < APP_REGS_ADD_MIN || add > APP_REGS_ADD_MAX)
		return false;
	
	/* Check if type matches */
	if (app_regs_type[add-APP_REGS_ADD_MIN] != type)
		return false;

	/* Check if the number of elements matches */
	if (app_regs_n_elements[add-APP_REGS_ADD_MIN] != n_elements)
		return false;

	/* Process data and return false if write is not allowed or contains errors */
	return (*app_func_wr_pointer[add-APP_REGS_ADD_MIN])(content);
}