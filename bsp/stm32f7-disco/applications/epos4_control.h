#ifndef __EPOS4_CONTROL_H__
#define __EPOS4_CONTROL_H__


int rt_epos4_control_init(void);


//操作目标
#define TARGET_CSP 1
#define TARGET_CSV 2
#define TARGET_CST 3

//读写字典操作
#define SDO_READ_OD 1
#define SDO_WRITE_OD 2

//NMT状态机命令
#define NMT_START_NODE  0x01
#define NMT_STOP_NODE   0x02
#define NMT_ENTER_OP    0x80
#define RESET_NODE      0x81
#define ERSET_COMMU     0x82


//NMT状态
#define NMT_STA_INI    0
#define NMT_STA_STOP   4
#define NMT_STA_OPER   5
#define NMT_STA_PRE_OP 127

//电机操作模式
#define OP_MODE_PPM 1
#define OP_MODE_PVM 3
#define OP_MODE_HMM 6
#define OP_MODE_CSP 8
#define OP_MODE_CSV 9
#define OP_MODE_CST 10




//数据帧的COD-ID
#define DEVICE1_TPDO4  0x4A1 
#define DEVICE1_TPDO3 0x3A1
#define DEVOCE1_HERTBEAT 0x701
#define DEVICE1_SDO_RESP 0x581




//命令定义宏
#define EN_VOL_BIT 0x02    //开电压最优先，电压一关，其他功能全失灵
#define QK_STOP_BIT 0x04   //QK STOP是在开电压的情况下的功能，QK STOP打开，其他功能也失灵,0电平有效
#define SWITCH_ON_BIT 0x01 //开关
#define EN_OP_BIT 0x08     //在开关打开后决定能否操作
#define FAULT_BIT 0x80     //发生错误，进行复位


#define CMD_EN_VOL  EN_VOL_BIT|QK_STOP_BIT
#define CMD_SW_ON   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT
#define CMD_EN_OP   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT|EN_OP_BIT

#define CMD_QK_STOP EN_VOL_BIT  //在电压打开形况下，QK Stop位置0，就是急停命令
#define CMD_DIS_VOL 0x00

#define FAULT_RST FAULT_BIT



//状态宏定义

#define DEV_STATE_MSDK  0x006F

#define DEV_STA_NOT_RD_TO_SW_ON 0x0000
#define DEV_STA_SW_ON_DISABLE 0x0040
#define DEV_STA_RD_TO_SW_ON 0x0021
#define DEV_STA_SWITCH_ON 0x0023
#define DEV_STA_OPER_EN 0x0027
#define DEV_STA_QK_STOP 0x0007
#define DEV_STA_FLT_RE_AC 0x000F
#define DEV_STA_FAULT 0x0008



#endif
