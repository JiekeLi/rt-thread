#include "epos4_application.h"
#include "epos4_node1.h"
#include "epos4_control.h"
#include <finsh.h>
#include <stdlib.h>

#define LIS_THREAD_PRIO 25
#define LIS_THREAD_STACK_SIZE 512
#define LIS_THREAD_TIMESIZE 5

#define CTL_THREAD_PRIO 25
#define CTL_THREAD_STACK_SIZE 512
#define CTL_THREAD_TIMESIZE 5

static rt_thread_t lis_tid = RT_NULL; //线程对象指针
static rt_thread_t ctl_tid = RT_NULL;
static struct rt_semaphore rx_sem ; //信号量对象
static struct rt_can_msg epos_msg; //消息对象

//设备接收消息回调函数
static rt_err_t epos4_rx_ind(rt_device_t dev, rt_size_t size)
{
		rt_sem_release(&rx_sem);
	  
	  return 0; 
}

//打开并设置设备
static void epos4_ctl_set_device(const char *device_name)
{
		rt_device_t dev = RT_NULL;
		dev = rt_device_find(device_name);
	
	  if(dev == RT_NULL)
		{
				rt_kprintf("epos4_control:can not find device :%s\r\n",device_name);
				return ;
		}
		
		if(rt_device_open(dev,RT_DEVICE_OFLAG_RDWR|
			                    RT_DEVICE_FLAG_INT_RX|
		                      RT_DEVICE_FLAG_INT_TX)==RT_EOK)
		{
			  
			  epos4_dev = dev ; //设置设备对象
			  
			  rt_sem_init(&rx_sem,"eporx",0,0);   //初始化接收信号量
			  
			  rt_device_set_rx_indicate(dev, epos4_rx_ind);//设置接收到数据的回调
			  
		}else{
				rt_kprintf("epos4_control:open device :%s fail\r\n",device_name);
		}	
}

//线程入口
static void listener_thread_entry(void *parameter)
{
		while(1){
				rt_sem_take(&rx_sem,RT_WAITING_FOREVER);
			  rt_device_read(epos4_dev,-1,(void *)&epos_msg,sizeof(struct rt_can_msg));
				
				node1.update(&epos_msg);
	
				rt_kprintf("id:0x%X \r\n",epos_msg.id);
				rt_kprintf("ide:0x%X \r\n",epos_msg.ide);
				rt_kprintf("rtr:0x%X \r\n",epos_msg.rtr);
				rt_kprintf("rlc:0x%X \r\n",epos_msg.len);		  
				for(int i=0;i<8;i++){
							rt_kprintf("data[%d]: 0x%X \r\n",i,epos_msg.data[i]);
				}
		}
}

//线程入口
static void controller_thread_entry(void *parameter)
{
	
		while( node1.nmt_state!=0x00 );
		
	  node1.start(OP_MODE_CSP);
		
	  while(1){
				rt_thread_mdelay(10);
		}
		
}


//初始化线程

int rt_epos4_application_init(void)
{
		
		epos4_ctl_set_device("can1");
	  if(epos4_dev != RT_NULL){
			node1.init(epos4_dev);
	  }else{
			rt_kprintf("Open devices error\n");
			return -1;
		}

		lis_tid = rt_thread_create("LisThd",listener_thread_entry,(void*)0,
			                      LIS_THREAD_STACK_SIZE,
		                        LIS_THREAD_PRIO,
		                        LIS_THREAD_TIMESIZE);
	
		ctl_tid = rt_thread_create("CtlThd",controller_thread_entry,(void*)0,
			                      CTL_THREAD_STACK_SIZE,
		                        CTL_THREAD_PRIO,
		                        CTL_THREAD_TIMESIZE);

	
	  if(lis_tid!=RT_NULL) rt_thread_startup(lis_tid);
		else return -1;
		
		if(ctl_tid!=RT_NULL) rt_thread_startup(ctl_tid);
		else return -1;

		
		return 0;

}

/*测试函数*/
//设置目标
static  int node1_set_target(int argc ,void** argv)
{
	
	if( argc >= 3 ){
		rt_uint8_t target = atoi(argv[1]);
		rt_int32_t val = atoi(argv[2]);
		node1.set_terget(target,val);
	
	}else{
		return -1;
	}
	return 0;
}
MSH_CMD_EXPORT(node1_set_target,Node1 set target)

//设置工作模式
static int node1_set_mode(int argc, void** argv)
{
	if( argc >=2  ){
		node1.operation_mode  = atoi(argv[1]);
		node1.set_op_mode(node1.operation_mode);
	
	}else{
		return -1;
	}
	return 0;
		
}

MSH_CMD_EXPORT(node1_set_mode,Node1 set op mode)

static int node1_falut_reset(int argc, void** argv)
{
	if( argc >=1  ){
		node1.falut_reset();
	
	}else{
		return -1;
	}
	return 0;
		
}

MSH_CMD_EXPORT(node1_falut_reset,Node1 fault reset)

static int node1_quick_stop(int argc, void** argv)
{
	if( argc >=1  ){
		
		node1.quick_stop();
	
	}else{
		return -1;
	}
	return 0;
		
}

MSH_CMD_EXPORT(node1_quick_stop,Node1 quick stop)

static int node1_state_reset(int argc, void** argv)
{
	if( argc >=1  ){
		
		node1.state_reset();
	
	}else{
		return -1;
	}
	return 0;
		
}
MSH_CMD_EXPORT(node1_state_reset,Node1 state reset)
