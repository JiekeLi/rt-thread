#include "epos4_control.h"

#define CTL_THREAD_PRIO 25
#define CTL_THREAD_STACK_SIZE 512
#define CTL_THREAD_TIMESIZE 5

static rt_thread_t tid = RT_NULL; //线程对象指针
static struct rt_semaphore rx_sem ; //信号量对象
static rt_device_t epos4_dev=RT_NULL; //设备对象指针
static struct rt_can_msg epos_msg; //消息对象


static rt_err_t epos4_rx_ind(rt_device_t dev, rt_size_t size)
{
		rt_sem_release(&rx_sem);
	  
	  return 0; 
}


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


static void thread_entry(void *parameter)
{
	  epos4_ctl_set_device("can1");
	  while(1){
	      rt_sem_take(&rx_sem,RT_WAITING_FOREVER);
			  rt_device_read(epos4_dev,-1,(void *)&epos_msg,sizeof(struct rt_can_msg));
			  rt_kprintf("id:0x%X \r\n",epos_msg.id);
			  rt_kprintf("ide:0x%X \r\n",epos_msg.ide);
			  rt_kprintf("rtr:0x%X \r\n",epos_msg.rtr);
			  rt_kprintf("rlc:0x%X \r\n",epos_msg.len);		  
			  for(int i=0;i<8;i++){
					rt_kprintf("data[%d]: 0x%X \r\n",i,epos_msg.data[i]);
				}
			
		}
}


void epos4_send_test(void){
	struct rt_can_msg epos_msg;
	epos_msg.id = 0x12;
	epos_msg.rtr = 0;
	epos_msg.ide = 0;
	epos_msg.len = 8;
	epos_msg.data[0]=0x011;
	
	rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));
		
}	




int rt_epos4_control_init(void)
{
		tid = rt_thread_create("CtlThd",thread_entry,(void*)0,
			                      CTL_THREAD_STACK_SIZE,
		                        CTL_THREAD_PRIO,
		                        CTL_THREAD_TIMESIZE);
	  if(tid!=RT_NULL)
			rt_thread_startup(tid);
		else
			return -1;
		
		return 0;

}

MSH_CMD_EXPORT(epos4_send_test,test can send msg)
