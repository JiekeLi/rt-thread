/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup.
 */

#include <rtthread.h>
//#include "epos4_control.h"
#include "epos4_application.h"
#include "imu_control.h"
#include "usart2_app.h"

int main(void)
{
    rt_epos4_application_init();
		//imu_control_init();
	  usart2_application_init();
    return 0;
}

