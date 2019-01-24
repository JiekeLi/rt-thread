#include "imu_control.h"
#include "rs485.h"
#include <finsh.h>
#include <stdlib.h>
#include "JY901.h"

#define SEND_THREAD_PRIO 19
#define SEND_THREAD_STACK_SIZE 512
#define SEND_THREAD_TIMESIZE 10

static rt_thread_t send_tid = RT_NULL; //线程对象指针


static void send_thread_entry(void *parameter)
{
		RS485_Init(115200);
	
		while(1){
			
			ModbusRWReg(allowAddeId[0],imuRead,Roll,3);
			
			rt_thread_mdelay(20);
			
			ModbusRWReg(allowAddeId[1],imuRead,Roll,3);
			
			rt_thread_mdelay(20);	
			
		}
	
}

int imu_control_init(void)
{
				send_tid = rt_thread_create("SndThd",send_thread_entry,(void*)0,
			                      SEND_THREAD_STACK_SIZE,
		                        SEND_THREAD_PRIO,
		                        SEND_THREAD_TIMESIZE);
	
				if(send_tid != RT_NULL) rt_thread_startup(send_tid);
				else return -1;
	
				return 0;
}
