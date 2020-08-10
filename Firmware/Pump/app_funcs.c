#include "app_funcs.h"
#include "app_ios_and_regs.h"
#include "hwbp_core.h"

/************************************************************************/
/* Create pointers to functions                                         */
/************************************************************************/
extern AppRegs app_regs;

void (*app_func_rd_pointer[])(void) = {
	&app_read_REG_ENABLE_MOTOR_DRIVER,
	&app_read_REG_ENABLE_MOTOR_UC,
	&app_read_REG_STEP_STATE,
	&app_read_REG_DIR_STATE,
	&app_read_REG_SW_FORWARD_STATE,
	&app_read_REG_SW_REVERSE_STATE,
	&app_read_REG_INPUT_STATE,
	&app_read_REG_SET_DOS,
	&app_read_REG_CLEAR_DOS,
	&app_read_REG_DO0_CONFIG,
	&app_read_REG_DO1_CONFIG,
	&app_read_REG_DI0_CONFIG,
	&app_read_REG_MOTOR_MICROSTEP,
	&app_read_REG_PROTOCOL_NUMBER_STEPS,
	&app_read_REG_PROTOCOL_FLOWRATE,
	&app_read_REG_PROTOCOL_VAR0,
	&app_read_REG_PROTOCOL_VAR1,
	&app_read_REG_PROTOCOL_VAR2,
	&app_read_REG_EVT_ENABLE
};

bool (*app_func_wr_pointer[])(void*) = {
	&app_write_REG_ENABLE_MOTOR_DRIVER,
	&app_write_REG_ENABLE_MOTOR_UC,
	&app_write_REG_STEP_STATE,
	&app_write_REG_DIR_STATE,
	&app_write_REG_SW_FORWARD_STATE,
	&app_write_REG_SW_REVERSE_STATE,
	&app_write_REG_INPUT_STATE,
	&app_write_REG_SET_DOS,
	&app_write_REG_CLEAR_DOS,
	&app_write_REG_DO0_CONFIG,
	&app_write_REG_DO1_CONFIG,
	&app_write_REG_DI0_CONFIG,
	&app_write_REG_MOTOR_MICROSTEP,
	&app_write_REG_PROTOCOL_NUMBER_STEPS,
	&app_write_REG_PROTOCOL_FLOWRATE,
	&app_write_REG_PROTOCOL_VAR0,
	&app_write_REG_PROTOCOL_VAR1,
	&app_write_REG_PROTOCOL_VAR2,
	&app_write_REG_EVT_ENABLE
};


/************************************************************************/
/* REG_ENABLE_MOTOR_DRIVER                                              */
/************************************************************************/
void app_read_REG_ENABLE_MOTOR_DRIVER(void){}
bool app_write_REG_ENABLE_MOTOR_DRIVER(void *a)
{
	uint8_t reg = *((uint8_t*)a);
	
	if(reg)
		set_EN_DRIVER;
	else
		clr_EN_DRIVER;
	
	app_regs.REG_ENABLE_MOTOR_DRIVER = reg;

	return true;
}


/************************************************************************/
/* REG_ENABLE_MOTOR_UC		                                            */
/************************************************************************/
void app_read_REG_ENABLE_MOTOR_UC(void){}
bool app_write_REG_ENABLE_MOTOR_UC(void *a)
{
	uint8_t reg = *((uint8_t*)a);
	
	app_write_REG_ENABLE_MOTOR_DRIVER(&reg);
	
	app_regs.REG_ENABLE_MOTOR_UC = reg;

	return true;
}


/************************************************************************/
/* REG_STEP_STATE                                                       */
/************************************************************************/
uint8_t step_period_counter = 0;
void app_read_REG_STEP_STATE(void)
{
	//app_regs.REG_STEP_STATE = 0;

}

bool app_write_REG_STEP_STATE(void *a)
{
	uint8_t reg = *((uint8_t*)a);
	
	if(reg)
	{
		// force starting counting from the start
		step_period_counter = 0;
		set_STEP;
		if((app_regs.REG_DO1_CONFIG & MSK_OUT1_CONF) == GM_OUT1_STEP_STATE)
		{
			set_OUT01;
		}
	}

	app_regs.REG_STEP_STATE = reg;
	return true;
}


/************************************************************************/
/* REG_DIR_STATE                                                        */
/************************************************************************/
uint8_t curr_dir = 0;
void app_read_REG_DIR_STATE(void)
{
	//app_regs.REG_DIR_STATE = 0;

}

bool app_write_REG_DIR_STATE(void *a)
{
	uint8_t reg = *((uint8_t*)a);
	
	if(reg != curr_dir)
		curr_dir = reg;

	app_regs.REG_DIR_STATE = reg;
	return true;
}


/************************************************************************/
/* REG_SW_FORWARD_STATE                                                 */
/************************************************************************/
bool disable_steps = false;
void app_read_REG_SW_FORWARD_STATE(void)
{
	//app_regs.REG_SW_FORWARD_STATE = 0;

}

bool app_write_REG_SW_FORWARD_STATE(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_SW_FORWARD_STATE = reg;
	return true;
}


/************************************************************************/
/* REG_SW_REVERSE_STATE                                                 */
/************************************************************************/
void app_read_REG_SW_REVERSE_STATE(void)
{
	//app_regs.REG_SW_REVERSE_STATE = 0;

}

bool app_write_REG_SW_REVERSE_STATE(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_SW_REVERSE_STATE = reg;
	return true;
}


/************************************************************************/
/* REG_INPUT_STATE                                                      */
/************************************************************************/
void app_read_REG_INPUT_STATE(void)
{
	//app_regs.REG_INPUT_STATE = 0;

}

bool app_write_REG_INPUT_STATE(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_INPUT_STATE = reg;
	return true;
}


/************************************************************************/
/* REG_SET_DOS                                                          */
/************************************************************************/
void app_read_REG_SET_DOS(void)
{
	//app_regs.REG_SET_DOS = 0;

}

bool app_write_REG_SET_DOS(void *a)
{
	uint8_t reg = *((uint8_t*)a);
	
	if((app_regs.REG_DO0_CONFIG & MSK_OUT0_CONF) == GM_OUT0_SOFTWARE)
		if(reg & B_SET_DO0)
			set_OUT00;
	
	if((app_regs.REG_DO1_CONFIG & MSK_OUT1_CONF) == GM_OUT1_SOFTWARE)
		if(reg & B_SET_DO1)
			set_OUT01;

	app_regs.REG_SET_DOS = reg;
	return true;
}


/************************************************************************/
/* REG_CLEAR_DOS                                                        */
/************************************************************************/
void app_read_REG_CLEAR_DOS(void)
{
	//app_regs.REG_CLEAR_DOS = 0;

}

bool app_write_REG_CLEAR_DOS(void *a)
{
	uint8_t reg = *((uint8_t*)a);
	
	if((app_regs.REG_DO0_CONFIG & MSK_OUT0_CONF) == GM_OUT0_SOFTWARE)
		if((reg & B_CLR_DO0) == 0)
			clr_OUT00;
	
	if((app_regs.REG_DO1_CONFIG & MSK_OUT1_CONF) == GM_OUT1_SOFTWARE)
		if((reg & B_CLR_DO1) == 0)
			clr_OUT01;

	app_regs.REG_CLEAR_DOS = reg;
	return true;
}


/************************************************************************/
/* REG_DO0_CONFIG                                                       */
/************************************************************************/
void app_read_REG_DO0_CONFIG(void)
{
	//app_regs.REG_DO0_CONFIG = 0;

}

bool app_write_REG_DO0_CONFIG(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_DO0_CONFIG = reg;
	return true;
}


/************************************************************************/
/* REG_DO1_CONFIG                                                       */
/************************************************************************/
void app_read_REG_DO1_CONFIG(void)
{
	//app_regs.REG_DO1_CONFIG = 0;

}

bool app_write_REG_DO1_CONFIG(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_DO1_CONFIG = reg;
	return true;
}


/************************************************************************/
/* REG_DI0_CONFIG                                                       */
/************************************************************************/
void app_read_REG_DI0_CONFIG(void)
{
	//app_regs.REG_DI0_CONFIG = 0;

}

bool app_write_REG_DI0_CONFIG(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_DI0_CONFIG = reg;
	return true;
}


/************************************************************************/
/* REG_MOTOR_MICROSTEP                                                  */
/************************************************************************/
void app_read_REG_MOTOR_MICROSTEP(void){}
bool app_write_REG_MOTOR_MICROSTEP(void *a)
{
	uint8_t reg = *((uint8_t*)a);
	
	switch (reg)
	{
		case GM_STEP_FULL:
			clr_MS1;
			clr_MS2;
			clr_MS3;
			break;
		
		case GM_STEP_HALF:
			set_MS1;
			clr_MS2;
			clr_MS3;
			break;
		
		case GM_STEP_QUARTER:
			clr_MS1;
			set_MS2;
			clr_MS3;
			break;
		
		case GM_STEP_EIGHTH:
			set_MS1;
			set_MS2;
			clr_MS3;
			break;

		case GM_STEP_SIXTEENTH:
			set_MS1;
			set_MS2;
			set_MS3;
			break;

		default:
			return false;
	}
	
	app_regs.REG_MOTOR_MICROSTEP = reg;
	return true;
}


/************************************************************************/
/* REG_PROTOCOL_NUMBER_STEPS                                            */
/************************************************************************/
void app_read_REG_PROTOCOL_NUMBER_STEPS(void)
{
	//app_regs.REG_PROTOCOL_NUMBER_STEPS = 0;

}

bool app_write_REG_PROTOCOL_NUMBER_STEPS(void *a)
{
	uint16_t reg = *((uint16_t*)a);
	
	/* Check range */
	if (reg < 1)
		return false;

	app_regs.REG_PROTOCOL_NUMBER_STEPS = reg;
	return true;
}


/************************************************************************/
/* REG_PROTOCOL_FLOWRATE                                                */
/************************************************************************/
void app_read_REG_PROTOCOL_FLOWRATE(void)
{
	//app_regs.REG_PROTOCOL_FLOWRATE = 0;

}

bool app_write_REG_PROTOCOL_FLOWRATE(void *a)
{
	float reg = *((float*)a);
	
	/* Check range */
	if (reg < 0.5 || reg > 2000.0)
		return false;

	app_regs.REG_PROTOCOL_FLOWRATE = reg;
	return true;
}


/************************************************************************/
/* REG_PROTOCOL_VAR0                                                    */
/************************************************************************/
void app_read_REG_PROTOCOL_VAR0(void)
{
	//app_regs.REG_PROTOCOL_VAR0 = 0;

}

bool app_write_REG_PROTOCOL_VAR0(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_PROTOCOL_VAR0 = reg;
	return true;
}


/************************************************************************/
/* REG_PROTOCOL_VAR1                                                    */
/************************************************************************/
void app_read_REG_PROTOCOL_VAR1(void)
{
	//app_regs.REG_PROTOCOL_VAR1 = 0;

}

bool app_write_REG_PROTOCOL_VAR1(void *a)
{
	uint16_t reg = *((uint16_t*)a);

	app_regs.REG_PROTOCOL_VAR1 = reg;
	return true;
}


/************************************************************************/
/* REG_PROTOCOL_VAR2                                                    */
/************************************************************************/
void app_read_REG_PROTOCOL_VAR2(void)
{
	//app_regs.REG_PROTOCOL_VAR2 = 0;

}

bool app_write_REG_PROTOCOL_VAR2(void *a)
{
	float reg = *((float*)a);

	app_regs.REG_PROTOCOL_VAR2 = reg;
	return true;
}


/************************************************************************/
/* REG_EVT_ENABLE                                                       */
/************************************************************************/
void app_read_REG_EVT_ENABLE(void)
{
	//app_regs.REG_EVT_ENABLE = 0;

}

bool app_write_REG_EVT_ENABLE(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_EVT_ENABLE = reg;
	return true;
}