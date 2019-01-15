#include "epos4_control.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdlib.h>

#define CTL_THREAD_PRIO 25
#define CTL_THREAD_STACK_SIZE 512
#define CTL_THREAD_TIMESIZE 5

static rt_thread_t tid = RT_NULL; //线程对象指针
static struct rt_semaphore rx_sem ; //信号量对象
static rt_device_t epos4_dev=RT_NULL; //设备对象指针
static struct rt_can_msg epos_msg; //消息对象
static rt_uint16_t control_world = 0x0000;
static rt_uint16_t status_word = 0;
static rt_uint8_t nmt_state = 0;
static rt_uint8_t operation_mode = 0;
static rt_int16_t error_code = 0;
static rt_int32_t csp_target = 0;
static rt_int32_t csv_target = 0;
static rt_int32_t cst_target = 0; 


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
static void thread_entry(void *parameter)
{
	  epos4_ctl_set_device("can1");
	  while(1){
			
	      rt_sem_take(&rx_sem,RT_WAITING_FOREVER);
			  rt_device_read(epos4_dev,-1,(void *)&epos_msg,sizeof(struct rt_can_msg));
			  
				if(epos_msg.id == DEVOCE1_HERTBEAT){       //收到心跳帧
						rt_kprintf("Receive a heartbeat msg\n");
						nmt_state = epos_msg.data[0];
					  switch(nmt_state){
							case NMT_STA_INI:
								rt_kprintf("The NMT state is: INI \n");break;
							case NMT_STA_STOP:
								rt_kprintf("The NMT state is: STOP\n");break;
							case NMT_STA_OPER:
								rt_kprintf("The NMT state is: OPER\n");break;
							case NMT_STA_PRE_OP:
								rt_kprintf("The NMT state is: PRE_OP\n");break;
						}							
						rt_kprintf("The NMT state is: %d\n",nmt_state);
				}
				
				if(epos_msg.id == DEVICE1_TPDO4){    //收到TPDO帧
						rt_kprintf("Receive a TPDO msg\n");
					  status_word = ( epos_msg.data[1]<<8 )| epos_msg.data[0];
					  operation_mode = ( epos_msg.data[3]<<8 )|epos_msg.data[2];
					  error_code = ( epos_msg.data[5]<<8 )|epos_msg.data[4];
					  switch(status_word & DEV_STATE_MSDK){
							case DEV_STA_NOT_RD_TO_SW_ON:
								rt_kprintf("The status world is: NOT_RD_TO_SW_ON \n");break;
							case DEV_STA_SW_ON_DISABLE:
								rt_kprintf("The status world is: SW_ON_DISABLE \n");break;
							case DEV_STA_RD_TO_SW_ON:
								rt_kprintf("The status world is: RD_TO_SW_ON \n");break;
							case DEV_STA_SWITCH_ON:
								rt_kprintf("The status world is: SWITCH_ON \n");break;
							case DEV_STA_OPER_EN:
								rt_kprintf("The status world is: OPER_EN \n");break;
							case DEV_STA_QK_STOP:
								rt_kprintf("The status world is: QK_STOP \n");break;
							case DEV_STA_FLT_RE_AC:
								rt_kprintf("The status world is: FLT_RE_AC \n");break;
							case DEV_STA_FAULT:
								rt_kprintf("The status world is:FAULT \n");break;
						}						
						rt_kprintf("The status world is: %X \n",status_word);
					  rt_kprintf("The operation mode is: %d \n",operation_mode);
					  rt_kprintf("The error code is:%X \n",error_code);
				}
				
				if(epos_msg.id == DEVICE1_TPDO3){    //收到TPDO帧
						rt_kprintf("Receive a TPDO msg\n");
					  status_word = ( epos_msg.data[1]<<8 )| epos_msg.data[0];
					  operation_mode = ( epos_msg.data[3]<<8 )|epos_msg.data[2];
					  error_code = ( epos_msg.data[5]<<8 )|epos_msg.data[4];
					  switch(status_word & DEV_STATE_MSDK){
							case DEV_STA_NOT_RD_TO_SW_ON:
								rt_kprintf("The status world is: NOT_RD_TO_SW_ON \n");break;
							case DEV_STA_SW_ON_DISABLE:
								rt_kprintf("The status world is: SW_ON_DISABLE \n");break;
							case DEV_STA_RD_TO_SW_ON:
								rt_kprintf("The status world is: RD_TO_SW_ON \n");break;
							case DEV_STA_SWITCH_ON:
								rt_kprintf("The status world is: SWITCH_ON \n");break;
							case DEV_STA_OPER_EN:
								rt_kprintf("The status world is: OPER_EN \n");break;
							case DEV_STA_QK_STOP:
								rt_kprintf("The status world is: QK_STOP \n");break;
							case DEV_STA_FLT_RE_AC:
								rt_kprintf("The status world is: FLT_RE_AC \n");break;
							case DEV_STA_FAULT:
								rt_kprintf("The status world is:FAULT \n");break;
						}
					  rt_kprintf("The status world is: %X \n",status_word);
					  rt_kprintf("The operation mode is: %d \n",operation_mode);
						rt_kprintf("The error code is:%X \n",error_code);
				}
				
				
				if(epos_msg.id == DEVICE1_SDO_RESP){  //收到SDO响应
						rt_kprintf("Receive a SDO respon\n");
						rt_kprintf("id:0x%X \r\n",epos_msg.id);
						rt_kprintf("ide:0x%X \r\n",epos_msg.ide);
						rt_kprintf("rtr:0x%X \r\n",epos_msg.rtr);
						rt_kprintf("rlc:0x%X \r\n",epos_msg.len);		  
						for(int i=0;i<8;i++){
							rt_kprintf("data[%d]: 0x%X \r\n",i,epos_msg.data[i]);
						}
				}	
						rt_kprintf("Receive a SDO respon\n");
						rt_kprintf("id:0x%X \r\n",epos_msg.id);
						rt_kprintf("ide:0x%X \r\n",epos_msg.ide);
						rt_kprintf("rtr:0x%X \r\n",epos_msg.rtr);
						rt_kprintf("rlc:0x%X \r\n",epos_msg.len);		  
						for(int i=0;i<8;i++){
							rt_kprintf("data[%d]: 0x%X \r\n",i,epos_msg.data[i]);
						}				
				
		}
}

//初始化线程

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






/*帧构造函数*/




//构造NMT数据
void creat_nmt_msg(rt_can_msg_t msg,rt_uint32_t node_id,rt_uint8_t op)
{
		msg->id = 0x00;
	  msg->rtr = 0;
	  msg->ide = 0;
	  msg->len = 2;
	  msg->data[0] = op;
	  msg->data[1] =  node_id;
}








//构造SYNC帧
void creat_sync_msg(rt_can_msg_t msg)
{
		msg->id = 0x80;
		msg->rtr = 0;
		msg->len = 0;
	  msg->ide = 0;
	  for(int i=0;i < 8;i++ ){
			msg->data[i] = 0x00;
		}
}







//构造RPDO帧
void create_rpdo_msg(rt_can_msg_t msg,
	                   rt_uint32_t cob_id,
										 rt_uint8_t* buf,
                     rt_uint8_t len)
{
		msg->id = cob_id;
		msg->rtr = 0;
	  msg->ide = 0;
	  if( len <= 8 ){
			msg->len = len;
			for(int i = 0; i < len; i++ ){
				msg->data[i] = buf[i];
			}
		}else{
			rt_kprintf("PDO data should less than 8 bytes\n");
		}
}









//构造SDO数据帧
void creat_sdo_msg(rt_can_msg_t msg,   //信息指针
	                 rt_uint32_t node_id,//node id
                   rt_uint32_t op, //读或写
                   rt_uint16_t index, //索引
                   rt_uint8_t subindex,//子索引
                   rt_uint8_t* buf,rt_uint8_t len)//数据和长度
{
		msg->ide = 0;
	  msg->rtr = 0;
	  msg->len = 8;
		switch(op){
			case SDO_READ_OD:{
				msg->id = 0x600+node_id;
				msg->data[0] = 0x40;
				msg->data[1] = index & 0x00FF;
				msg->data[2] = ( index >> 8 );

				msg->data[4] = 0x00;
				msg->data[5] = 0x00;
				msg->data[6] = 0x00;
				msg->data[7] = 0x00;
			};break;
			case SDO_WRITE_OD:{
				if(len <= 4){
					msg->id = 0x600+node_id;
					msg->data[0] = 0x23 | ( ( (4-len) << 2 ) & 0x0C );
				  msg->data[1] = index & 0x00FF;
				  msg->data[2] = ( index >> 8 );
				  msg->data[3] = subindex;
					msg->data[4] = buf[0];
					msg->data[5] = buf[1];
					msg->data[6] = buf[2];
					msg->data[7] = buf[3];
				}else{
					rt_kprintf("Too much data!\n");
				}
			};break;
		}
	
	
}









//构造设置控制模式消息(SDO)
void creat_operation_mode_msg(rt_can_msg_t epos_msg,rt_uint32_t node_id,rt_uint8_t mode)
{
	  rt_uint8_t tmp = mode;
	  creat_sdo_msg(epos_msg,
									node_id,SDO_WRITE_OD,
	                0x6060,0x00,&tmp,sizeof(tmp));					
}













//构造设置cmd和控制模式的消息(RPDO)
void creat_set_control_word_msg(rt_can_msg_t epos_msg,rt_uint32_t cob_id,rt_uint16_t op,rt_uint16_t cmd)
{
		rt_uint8_t data[4] = {0};
		control_world = cmd;
		operation_mode = op;
	  data[0] = control_world & 0x00FF;
    data[1] = ( control_world >> 8 ) & 0x00FF;
		data[2] = operation_mode & 0x00FF;
    data[3] = ( operation_mode >> 8 ) & 0x00FF;			
		create_rpdo_msg( epos_msg, cob_id, data, sizeof(data) );
	   
}


//构造设置目标消息（RPDO）
void creat_set_target_msg(rt_can_msg_t epos_msg, rt_uint32_t cob_id, rt_uint32_t target, rt_int32_t val)
{
			rt_uint8_t data[8]={0};
			rt_uint8_t len = 0;
		  switch(target){
				case TARGET_CSP: {
					csp_target = val;
					data[0] = csp_target & 0xFF;
					data[1] = ( csp_target>>8 ) & 0xFF;
					data[2] = ( csp_target>>16 ) & 0xFF;
					data[3] = ( csp_target>>24 ) & 0xFF;
					data[4] = data[5] = data[6] = data[7] = 0;
					len = 8;
				}break;
				case TARGET_CSV: {
					csv_target = val;
					data[4] = csv_target & 0xFF;
					data[5] = ( csv_target>>8 ) & 0xFF;
					data[6] = ( csv_target>>16 ) & 0xFF;
					data[7] = ( csv_target>>24 ) & 0xFF;
					data[0] = data[1] = data[2] = data[3] = 0;
					len = 8;
				}break;
				case TARGET_CST: {
					cst_target = val;
					data[0] = cst_target & 0xFF;
					data[1] = ( cst_target>>8 ) & 0xFF;
					data[2] = ( cst_target>>16 ) & 0xFF;
					data[3] = ( cst_target>>24 ) & 0xFF;
					data[4] = data[5] = data[6] = data[7] = 0;
					len = 4;
				}break;
			}
			create_rpdo_msg(epos_msg, cob_id, data, len);
}





/*测试函数*/

//设置目标函数
int epos4_send_target(int argc,void** argv)
{	
			struct rt_can_msg epos_msg;
			rt_uint32_t cob_id = atoi(argv[1]);
			rt_uint8_t target = atoi(argv[2]);
			rt_int32_t val = atoi(argv[3]);

			if(argc >= 4)
			{
					if(target != 1 && target != 2 && target!= 3){
						rt_kprintf(" Invalid arg %s\n", argv[0]);
						rt_kprintf(" Usage :%s cob_id target val \n", argv[0]);	
						rt_kprintf(" Target:1:CSP 2:CSV 3:CST \n");	
						return -1;						
					}else{
						creat_set_target_msg(&epos_msg, cob_id, target, val);
	          rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));									
					
					}
			}else{
					rt_kprintf(" Invalid Call %s\n", argv[0]);
					rt_kprintf(" Usage :%s cob_id target val \n", argv[0]);	
					rt_kprintf(" Target:1:CSP 2:CSV 3:CST \n");	
					return -1;
			}
			
			return 0;
}
MSH_CMD_EXPORT(epos4_send_target, Send target)



//发送命令和设置工作模式（通过RPDO）
int epos4_send_cmd(int argc, void** argv)
{
		struct rt_can_msg epos_msg;
	  rt_uint32_t cob_id = atoi(argv[1]);
		rt_uint16_t op_mode = atoi(argv[2]);
	  rt_uint16_t ctlwd = atoi(argv[3]);
	
	
		if(argc>=4){
					if(op_mode!=1 && op_mode!=3 && op_mode!=6 && op_mode!=8 && op_mode!=9 && op_mode!=10 &&
						 ctlwd!=1 && ctlwd!=2 && ctlwd!=3 && ctlwd!=4 && ctlwd!=5 && ctlwd!=6 ){
								rt_kprintf(" Invalid arg %s\n", argv[0]);
								rt_kprintf(" Usage :%s cob_id operation_mode ctlwd \n", argv[0]);	
								rt_kprintf(" Op_Mode:1:PPM 3:PVM 6:HMM 8:CSP 9:CSV 10:CST\n");	
								rt_kprintf(" Ctlwd: 1:CMD_EN_VOL 2:CMD_SW_ON 3:CMD_EN_OP 4:CMD_QK_STOP 5:CMD_DIS_VOL 6:FAULT_RST");
								return -1;
					}else{
						  
								switch(ctlwd){
									case 1:control_world = CMD_EN_VOL;break;
									case 2:control_world = CMD_SW_ON;break;
									case 3:control_world = CMD_EN_OP;break;
									case 4:control_world = CMD_QK_STOP;break;
									case 5:control_world = CMD_DIS_VOL;break;
									case 6:control_world = FAULT_RST;break;									
								}			
								creat_set_control_word_msg(&epos_msg,cob_id,op_mode,control_world);					
	              rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));			
								
					}
		}else{
			rt_kprintf(" Invalid Call %s\n", argv[0]);
			rt_kprintf(" Usage :%s cob_id operation_mode ctlwd \n", argv[0]);	
			rt_kprintf(" Op_Mode:1:PPM 3:PVM 6:HMM 8:CSP 9:CSV 10:CST\n");	
			rt_kprintf("Ctlwd: 1:CMD_EN_VOL 2:CMD_SW_ON 3:CMD_EN_OP 4:CMD_QK_STOP 5:CMD_DIS_VOL 6:FAULT_RST");
			return -1;
		}
		return 0;
		
		
		
}
MSH_CMD_EXPORT(epos4_send_cmd,Send cmd)


//设置操作模式(通过SDO)
int epos4_set_operation_mode(int argc,void** argv)
{
	  struct rt_can_msg epos_msg;
		rt_uint8_t mode = atoi(argv[2]);
		rt_uint8_t node_id = atoi(argv[1]);
		if(argc>=3){
					if(mode!=1 && mode!=3 && mode!=6 && mode!=8 && mode!=9 && mode!=10){
								rt_kprintf(" Invalid arg %s\n", argv[0]);
								rt_kprintf(" Usage :%s node_id state (1:PPM 3:PVM 6:HMM 8:CSP 9:CSV 10:CST)\n", argv[0]);	
								return -1;
					}else{
						
								creat_operation_mode_msg((rt_can_msg_t)&epos_msg,node_id,mode);
						
	              rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));			
								
					}
		}else{
			rt_kprintf(" Invalid Call %s\n", argv[0]);
			rt_kprintf(" Usage :%s node_id state (1:PPM 3:PVM 6:HMM 8:CSP 9:CSV 10:CST)\n", argv[0]);	
			return -1;
		}
		return 0;
		
}
MSH_CMD_EXPORT(epos4_set_operation_mode,Set operation mode)




//发送RPDO帧
void test_write_rpdo()
{
		struct rt_can_msg epos_msg;
		rt_uint8_t data[3] = { 0x06, 0x00, 0x01 };
	  create_rpdo_msg(&epos_msg,0x521,data,sizeof(data));
		rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));		
		
}
MSH_CMD_EXPORT(test_write_rpdo,Test write RPDO)



//发送NMT帧
int epos4_set_nmt_state(int argc,void** argv)
{
	  struct rt_can_msg epos_msg;
	  rt_uint8_t node_id;
	  rt_uint8_t op;
		if(argc >=3){
			node_id = atoi(argv[1]);
			op = atoi(argv[2]);
			
			if( op!=NMT_START_NODE && op!=NMT_STOP_NODE && op!=NMT_ENTER_OP && op!=RESET_NODE && op!=ERSET_COMMU ){
				  rt_kprintf(" Invalid arg %s\n", argv[0]);
				  rt_kprintf(" Usage :%s node_id state (1:Start node 2:Stop node 128:enter pre-operational 129:Reset node 130:Reset communication)\n", argv[0]);	
				  return -1;
			}
			
			rt_kprintf("Set nmt state: %d \n",op);
			creat_nmt_msg((rt_can_msg_t)&epos_msg,node_id,op);
			
	    rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));			
			
		}else{
			rt_kprintf(" Invalid Call %s\n", argv[0]);
			rt_kprintf(" Usage :%s node_id state (1:Start node 2:Stop node 128:enter pre-operational 129:Reset node 130:Reset communication)\n", argv[0]);	
			return -1;
		}
		return 0;
}
MSH_CMD_EXPORT(epos4_set_nmt_state,Set NMT state)



//测试发送同步帧
void epos4_test_send_sync()
{
		struct rt_can_msg epos_msg;
	  creat_sync_msg((rt_can_msg_t)&epos_msg);
	  rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));
}

MSH_CMD_EXPORT(epos4_test_send_sync,Test to send SYNC)


//测试写对象字典
void epos4_test_write_od()
{
		struct rt_can_msg epos_msg;
	  rt_uint32_t data = 0x60710010;
	  creat_sdo_msg((rt_can_msg_t)&epos_msg,
	                1,SDO_WRITE_OD,0x1060,0x02,(uint8_t*)&data,sizeof(data));
	
	  rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));
	
}
MSH_CMD_EXPORT(epos4_test_write_od,Test to write OD)


//测试读取对象字典
void epos4_test_read_od()
{
		struct rt_can_msg epos_msg;
	  creat_sdo_msg((rt_can_msg_t)&epos_msg,
	                 1,SDO_READ_OD,0x1000,0x00,RT_NULL,0);
	
	 
	  rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));
		
}

MSH_CMD_EXPORT(epos4_test_read_od,Test to read OD)




//设置CAN模式
int epos4_set_can_mode(int argc,void **argv)
{
	  rt_uint32_t mode;
		if(argc>=2){
			mode = atoi(argv[1]);
			
			if(mode!=0 && mode!=1 && mode!=2 && mode!=3){
				rt_kprintf(" Invalid arg %s\n", argv[0]);
			  rt_kprintf(" Usage :%s mode (0:normal 1:listen 2:loopback 3:loopback&listen)\n", argv[0]);	
				return -1;
			}
			
			rt_kprintf(" Set can mode %d\n",mode);			
			rt_device_control(epos4_dev,RT_CAN_CMD_SET_MODE,(void *)mode);	
		}else{
			rt_kprintf(" Invalid Call %s\n", argv[0]);
			rt_kprintf(" Usage :%s mode (0:normal 1:listen 2:loopback 3:loopback&listen)\n", argv[0]);	
			return -1;
		}
		return 0;
}
MSH_CMD_EXPORT(epos4_set_can_mode,set the mode of can);



//测试发送
void epos4_send_test(void){
	struct rt_can_msg epos_msg;
	epos_msg.id = 0x12;
	epos_msg.rtr = 0;
	epos_msg.ide = 0;
	epos_msg.len = 8;
	epos_msg.data[0]=0x011;
	
	rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));

}	
MSH_CMD_EXPORT(epos4_send_test,test can send msg)









