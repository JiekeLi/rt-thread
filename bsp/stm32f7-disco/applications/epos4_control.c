#include "epos4_control.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdlib.h>




rt_device_t epos4_dev=RT_NULL; //设备对象指针

/*帧构造函数*/


//构造NMT数据
static void creat_nmt_msg(rt_can_msg_t msg,rt_uint32_t node_id,rt_uint8_t op)
{
		msg->id = 0x00;
	  msg->rtr = 0;
	  msg->ide = 0;
	  msg->len = 2;
	  msg->data[0] = op;
	  msg->data[1] =  node_id;
}








//构造SYNC帧
static void creat_sync_msg(rt_can_msg_t msg)
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
static void create_rpdo_msg(rt_can_msg_t msg,
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
static void creat_sdo_msg(rt_can_msg_t msg,   //信息指针
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
static void creat_operation_mode_msg(rt_can_msg_t epos_msg,rt_uint32_t node_id,rt_uint8_t mode)
{
	  rt_uint8_t tmp = mode;
	  creat_sdo_msg(epos_msg,
									node_id,SDO_WRITE_OD,
	                0x6060,0x00,&tmp,sizeof(tmp));					
}


//构造设置cmd和控制模式的消息(RPDO)
static void creat_set_control_word_msg(rt_can_msg_t epos_msg,rt_uint32_t cob_id,rt_uint8_t op,rt_uint16_t cmd)
{
		
		//rt_uint8_t data[4] = {0};
	  rt_uint8_t data[3] = {0};
		rt_uint16_t control_world = cmd;
		rt_uint16_t operation_mode = op;
	  data[0] = control_world & 0x00FF;
    data[1] = ( control_world >> 8 ) & 0x00FF;
		data[2] = operation_mode & 0xFF;
    //data[3] = ( operation_mode >> 8 ) & 0x00FF;			
		create_rpdo_msg( epos_msg, cob_id, data, sizeof(data) );
	   
}


//构造设置目标消息（RPDO）
static void creat_set_target_msg(rt_can_msg_t epos_msg, rt_uint32_t cob_id, rt_uint32_t target, rt_int32_t val)
{
			rt_uint8_t data[8]={0};
			rt_uint8_t len = 0;
		  switch(target){
				case TARGET_CSP: {
					rt_int32_t csp_target = val;
					data[0] = csp_target & 0xFF;
					data[1] = ( csp_target>>8 ) & 0xFF;
					data[2] = ( csp_target>>16 ) & 0xFF;
					data[3] = ( csp_target>>24 ) & 0xFF;
					data[4] = data[5] = data[6] = data[7] = 0;
					len = 8;
				}break;
				case TARGET_CSV: {
					rt_int32_t csv_target = val;
					data[4] = csv_target & 0xFF;
					data[5] = ( csv_target>>8 ) & 0xFF;
					data[6] = ( csv_target>>16 ) & 0xFF;
					data[7] = ( csv_target>>24 ) & 0xFF;
					data[0] = data[1] = data[2] = data[3] = 0;
					len = 8;
				}break;
				case TARGET_CST: {
					rt_int32_t cst_target = val;
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



/*基本控制函数*/


//发送同步帧
void send_sync()
{
		struct rt_can_msg epos_msg;
	  creat_sync_msg((rt_can_msg_t)&epos_msg);
	  rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));
}

//发送NMT帧
int set_nmt_state(rt_uint8_t node_id, rt_uint8_t op)
{
	  struct rt_can_msg epos_msg;
	
		if( op!=NMT_START_NODE && op!=NMT_STOP_NODE && op!=NMT_ENTER_OP && op!=RESET_NODE && op!=ERSET_COMMU ){
				  rt_kprintf(" Invalid arg \n");
				  return -1;
		}
		
		creat_nmt_msg((rt_can_msg_t)&epos_msg,node_id,op);
			
	  rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));			

		return 0;
}

//发送命令和设置工作模式（通过RPDO）
int send_cmd( rt_uint32_t cob_id, rt_uint8_t op_mode, rt_uint16_t ctlwd)
{
		struct rt_can_msg epos_msg;
		rt_uint16_t control_world = 0x0000;
					
	  if(op_mode!=1 && op_mode!=3 && op_mode!=6 && op_mode!=8 && op_mode!=9 && op_mode!=10 &&
			               ctlwd!=1 && ctlwd!=2 && ctlwd!=3 && ctlwd!=4 && ctlwd!=5 && ctlwd!=6 ){
				rt_kprintf(" Invalid arg \n");
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

		return 0;	
}

//设置目标函数(通过RPDO)
int set_target(rt_uint32_t cob_id, rt_uint8_t target, rt_int32_t val)
{	
			struct rt_can_msg epos_msg;

			if(target != TARGET_CSP && target != TARGET_CSV && target!= TARGET_CST){
					rt_kprintf(" Invalid arg \n");
					return -1;						
			}else{
   				creat_set_target_msg(&epos_msg, cob_id, target, val);
	        rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));									
					
		 }	
		return 0;
}



//存储参数
void storage_parameter(rt_uint32_t node_id)
{
		rt_uint8_t data[4]={ 0x73, 0x61, 0x76, 0x65 };
		struct rt_can_msg epos_msg;
    
    creat_sdo_msg(&epos_msg, node_id, SDO_WRITE_OD, 0x1010, 0x01, data, sizeof(data));	

	  rt_device_write(epos4_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));						
		
}

































