#include "usart2_app.h"
#include "epos4_application.h"

#define U2_THREAD_PRIO 21
#define U2_THREAD_STACK_SIZE 512
#define U2_THREAD_TIMESIZE 5

static rt_thread_t lis_tid = RT_NULL; //�̶߳���ָ��
static struct rt_semaphore rx_sem ; //�ź�������
static rt_device_t usart2_dev ; //�豸����ָ��

//�豸������Ϣ�ص�����
static rt_err_t usart2_rx_ind(rt_device_t dev, rt_size_t size)
{
		rt_sem_release(&rx_sem);
	  return 0; 
}

//�򿪲������豸
static void usart2_set_device(const char *device_name)
{
		rt_device_t dev = RT_NULL;
		dev = rt_device_find(device_name);
	
	  if(dev == RT_NULL)
		{
				rt_kprintf("usart2:can not find device :%s\r\n",device_name);
				return ;
		}
		
		if(rt_device_open(dev,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | \
                       RT_DEVICE_FLAG_STREAM)==RT_EOK)
		{
			  
			  usart2_dev = dev ; //�����豸����
			  
			  rt_sem_init(&rx_sem,"us2rx",0,0);   //��ʼ�������ź���
			  
			  rt_device_set_rx_indicate(dev, usart2_rx_ind);//���ý��յ����ݵĻص�
			  
		}else{
				rt_kprintf("usart2:open device :%s fail\r\n",device_name);
		}	
}


//�߳����
static void listener_thread_entry(void *parameter)
{
		rt_uint8_t data;
		while(1){
				rt_sem_take(&rx_sem,RT_WAITING_FOREVER);
			  rt_device_read(usart2_dev,-1,&data,sizeof(data));
			  rt_kprintf("Receive a signal %X\n",data);
			  if(data == 0x51){
					test_flag1 = 1;
					data = 0;
				}
				
				if(data == 0x52){
					test_flat2 = 1;
					data = 0;
				}
		}
}

//��ʼ���߳�

int usart2_application_init(void)
{
		
		usart2_set_device("uart2");

		lis_tid = rt_thread_create("u2Thd",listener_thread_entry,(void*)0,
			                      U2_THREAD_STACK_SIZE,
		                        U2_THREAD_PRIO,
		                        U2_THREAD_TIMESIZE);
	

	
	  if(lis_tid!=RT_NULL) rt_thread_startup(lis_tid);
		else return -1;
				
		return 0;

}



