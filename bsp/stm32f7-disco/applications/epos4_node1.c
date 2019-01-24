#include "epos4_node1.h"

static void node1_init(rt_device_t dev);

epos4_device node1 = {
		.node_id = 0x01,
		.rpdo_id ={0x221,0x321,0x421,0x521},
		.tpdo_id ={0x1A1,0x2A1,0x3A1,0x4A1},
		.heart_beat = 0x701,
		.sdo_id = 0x581,
		.epos4_dev = RT_NULL,
		.control_world = 0,
		.status_word = 0,
		.nmt_state = 0xFF,
		.operation_mode = 0,
		.error_code = 0,
		.csp_target = 0,
		.csv_target = 0,
		.cst_target = 0,
		.init = node1_init,
};

//更新状态函数
static void node1_update(rt_can_msg_t epos_msg)
{
		  if(epos_msg->id == node1.heart_beat){       //收到心跳帧
						rt_kprintf("Receive a heartbeat msg\n");
						node1.nmt_state = epos_msg->data[0];
					  switch(node1.nmt_state){
							case NMT_STA_INI:
								rt_kprintf("Node1: The NMT state is: INI \n");break;
							case NMT_STA_STOP:
								rt_kprintf("Node1: The NMT state is: STOP\n");break;
							case NMT_STA_OPER:
								rt_kprintf("Node1: The NMT state is: OPER\n");break;
							case NMT_STA_PRE_OP:
								rt_kprintf("Node1: The NMT state is: PRE_OP\n");break;
						}							
						rt_kprintf("Node1: The NMT state is: %d\n",node1.nmt_state);
				}
				
				if(epos_msg->id == node1.tpdo_id[3]){    //收到TPDO帧
						rt_kprintf("Node1: Receive a TPDO msg\n");
					  node1.status_word = ( epos_msg->data[1]<<8 )| epos_msg->data[0];
					  node1.operation_mode = epos_msg->data[2];//( epos_msg->data[3]<<8 )|epos_msg->data[2];
					  node1.error_code = ( epos_msg->data[5]<<8 )|epos_msg->data[4];
					  switch(node1.status_word & DEV_STATE_MSDK){
							case DEV_STA_NOT_RD_TO_SW_ON:
								rt_kprintf("Node1: The status world is: NOT_RD_TO_SW_ON \n");break;
							case DEV_STA_SW_ON_DISABLE:
								rt_kprintf("Node1: The status world is: SW_ON_DISABLE \n");break;
							case DEV_STA_RD_TO_SW_ON:
								rt_kprintf("Node1: The status world is: RD_TO_SW_ON \n");break;
							case DEV_STA_SWITCH_ON:
								rt_kprintf("Node1: The status world is: SWITCH_ON \n");break;
							case DEV_STA_OPER_EN:
								rt_kprintf("Node1: The status world is: OPER_EN \n");break;
							case DEV_STA_QK_STOP:
								rt_kprintf("Node1: The status world is: QK_STOP \n");break;
							case DEV_STA_FLT_RE_AC:
								rt_kprintf("Node1: The status world is: FLT_RE_AC \n");break;
							case DEV_STA_FAULT:
								rt_kprintf("Node1: The status world is:FAULT \n");break;
						}						
						rt_kprintf("Node1: The status world is: %X \n",node1.status_word);
					  rt_kprintf("Node1: The operation mode is: %d \n",node1.operation_mode);
					  rt_kprintf("Node1: The error code is:%X \n",node1.error_code);
				}
				
				if(epos_msg->id == node1.tpdo_id[2] ){    //收到TPDO帧
						rt_kprintf("Node1: Receive a TPDO msg\n");
					  node1.status_word = ( epos_msg->data[1]<<8 )| epos_msg->data[0];
					  node1.operation_mode = ( epos_msg->data[3]<<8 )|epos_msg->data[2];
					  node1.error_code = ( epos_msg->data[5]<<8 )|epos_msg->data[4];
					  switch(node1.status_word & DEV_STATE_MSDK){
							case DEV_STA_NOT_RD_TO_SW_ON:
								rt_kprintf("Node1: The status world is: NOT_RD_TO_SW_ON \n");break;
							case DEV_STA_SW_ON_DISABLE:
								rt_kprintf("Node1: The status world is: SW_ON_DISABLE \n");break;
							case DEV_STA_RD_TO_SW_ON:
								rt_kprintf("Node1: The status world is: RD_TO_SW_ON \n");break;
							case DEV_STA_SWITCH_ON:
								rt_kprintf("Node1: The status world is: SWITCH_ON \n");break;
							case DEV_STA_OPER_EN:
								rt_kprintf("Node1: The status world is: OPER_EN \n");break;
							case DEV_STA_QK_STOP:
								rt_kprintf("Node1: The status world is: QK_STOP \n");break;
							case DEV_STA_FLT_RE_AC:
								rt_kprintf("Node1: The status world is: FLT_RE_AC \n");break;
							case DEV_STA_FAULT:
								rt_kprintf("Node1: The status world is:FAULT \n");break;
						}
					  rt_kprintf("Node1: The status world is: %X \n",node1.status_word);
					  rt_kprintf("Node1: The operation mode is: %d \n",node1.operation_mode);
						rt_kprintf("Node1: The error code is:%X \n",node1.error_code);
				}
				
				if(epos_msg->id == node1.sdo_id ){  //收到SDO响应
						rt_kprintf("Node1: Receive a SDO respon\n");
						rt_kprintf("Node1: id:0x%X \r\n",epos_msg->id);
						rt_kprintf("Node1: ide:0x%X \r\n",epos_msg->ide);
						rt_kprintf("Node1: rtr:0x%X \r\n",epos_msg->rtr);
						rt_kprintf("Node1: rlc:0x%X \r\n",epos_msg->len);		  
						for(int i=0;i<8;i++){
							rt_kprintf("Node1: data[%d]: 0x%X \r\n",i,epos_msg->data[i]);
						}
				}	
		
}

//启动函数
static void node1_start(rt_uint8_t mode)
{
		node1.operation_mode = mode;
	
		set_nmt_state( node1.node_id, 1 ); //设置NMT为OPERATIONAL
	
	  rt_thread_mdelay(1);
  
	  node1.control_world = CMD_EN_VOL;
		send_cmd( node1.rpdo_id[3], node1.operation_mode, 1); 

		rt_thread_mdelay(1);
	  node1.control_world = CMD_SW_ON;
		send_cmd( node1.rpdo_id[3], node1.operation_mode, 2);

	  rt_thread_mdelay(1);	
	  node1.control_world = CMD_EN_OP;
	  send_cmd( node1.rpdo_id[3], node1.operation_mode, 3); //输入命令，默认CSP模式
}
//状态复位函数
static void node1_state_reset(void)
{
		node1.control_world = CMD_DIS_VOL;
		send_cmd( node1.rpdo_id[3], node1.operation_mode, 5);//掉电
}

//快停函数
static void node1_quick_stop(void)
{
		node1.control_world = CMD_QK_STOP;
		send_cmd( node1.rpdo_id[3], node1.operation_mode, 4);//快停
		//send_cmd( node1.rpdo_id[3], node1.operation_mode, 5);//掉电
}

//NMT复位函数
static void node1_nmt_reset(void)
{
		
		set_nmt_state( node1.node_id, RESET_NODE );
}

//错误复位
static void node1_falut_reset(void)
{
		node1.control_world = FAULT_RST;
		send_cmd( node1.rpdo_id[3], node1.operation_mode, 6);
}

//设置目标
static void node1_set_target( rt_uint8_t terget, rt_int32_t val)
{
		switch(terget){
			case TARGET_CSP:{ node1.csp_target = val; set_target(node1.rpdo_id[0],terget,node1.csp_target); }break;
			case TARGET_CSV:{ node1.csv_target = val; set_target(node1.rpdo_id[0],terget,node1.csv_target); }break;
			case TARGET_CST:{ node1.cst_target = val; set_target(node1.rpdo_id[1],terget,node1.cst_target); }break;
		}
}


static void node1_set_op_mode(rt_uint8_t op_mode)
{
		node1_state_reset();
	  
	  rt_thread_mdelay(5);
		
	  node1_start(op_mode);
		
}

//初始化
static void node1_init(rt_device_t dev)
{
		node1.epos4_dev = dev;
		node1.update = node1_update;
		node1.start = node1_start;
		node1.falut_reset = node1_falut_reset;
		node1.set_terget = node1_set_target;
		node1.nmt_reset = node1_nmt_reset;
		node1.state_reset = node1_state_reset;
		node1.quick_stop = node1_quick_stop;
	  node1.set_op_mode = node1_set_op_mode;
}

