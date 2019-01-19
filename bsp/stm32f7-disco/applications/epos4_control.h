#ifndef __EPOS4_CONTROL_H__
#define __EPOS4_CONTROL_H__
#include <rtthread.h>
#include <rtdevice.h>

int rt_epos4_control_init(void);
void send_sync(void);
int set_nmt_state(rt_uint8_t node_id, rt_uint8_t op);
int send_cmd( rt_uint32_t cob_id, rt_uint16_t op_mode, rt_uint16_t ctlwd);
int set_target(rt_uint32_t cob_id, rt_uint8_t target, rt_int32_t val);

extern rt_device_t epos4_dev ; //�豸����ָ��


typedef struct{
	
		const rt_uint32_t node_id; //�ڵ��
		const rt_uint32_t rpdo_id[4];//RPOD��ʶ
		const rt_uint32_t tpdo_id[4];//TPOD��ʶ
		const rt_uint32_t heart_beat;
		const rt_uint32_t sdo_id;
	
		rt_device_t epos4_dev;       //�豸����ָ��
	
		rt_uint16_t control_world;  //������
		rt_uint16_t status_word ;   //״̬��
		rt_uint8_t nmt_state ;    //NMT״̬
		rt_uint16_t operation_mode; //����ģʽ
		rt_int16_t error_code;  //������
		rt_int32_t csp_target;  //λ��Ŀ��
		rt_int32_t csv_target;  //�ٶ�Ŀ��
		rt_int32_t cst_target;  //����Ŀ��
		void(*update)(rt_can_msg_t rcv_msg); //������Ϣ���ո��º���              
	  void(*start)(rt_uint16_t mode);
	  void(*falut_reset)(void);       //����λ����
	  void(*set_terget)( rt_uint8_t terget, rt_int32_t val);//����Ŀ�꺯��
	  void(*nmt_reset)(void);          //NMT��λ����
		void(*state_reset)(void);      //״̬��λ����
		void(*quick_stop)(void);
		void(*init)(rt_device_t dev);
		void(*set_op_mode)(rt_uint16_t mode);
} epos4_device;


//����Ŀ��
#define TARGET_CSP 1
#define TARGET_CSV 2
#define TARGET_CST 3

//��д�ֵ����
#define SDO_READ_OD 1
#define SDO_WRITE_OD 2

//NMT״̬������
#define NMT_START_NODE  0x01
#define NMT_STOP_NODE   0x02
#define NMT_ENTER_OP    0x80
#define RESET_NODE      0x81
#define ERSET_COMMU     0x82


//NMT״̬
#define NMT_STA_INI    0
#define NMT_STA_STOP   4
#define NMT_STA_OPER   5
#define NMT_STA_PRE_OP 127

//�������ģʽ
#define OP_MODE_PPM 1
#define OP_MODE_PVM 3
#define OP_MODE_HMM 6
#define OP_MODE_CSP 8
#define OP_MODE_CSV 9
#define OP_MODE_CST 10




//����֡��COD-ID
#define DEVICE1_TPDO4  0x4A1 
#define DEVICE1_TPDO3 0x3A1
#define DEVOCE1_HERTBEAT 0x701
#define DEVICE1_SDO_RESP 0x581




//������
#define EN_VOL_BIT 0x02    //����ѹ�����ȣ���ѹһ�أ���������ȫʧ��
#define QK_STOP_BIT 0x04   //QK STOP���ڿ���ѹ������µĹ��ܣ�QK STOP�򿪣���������Ҳʧ��,0��ƽ��Ч
#define SWITCH_ON_BIT 0x01 //����
#define EN_OP_BIT 0x08     //�ڿ��ش򿪺�����ܷ����
#define FAULT_BIT 0x80     //�������󣬽��и�λ


#define CMD_EN_VOL  EN_VOL_BIT|QK_STOP_BIT
#define CMD_SW_ON   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT
#define CMD_EN_OP   EN_VOL_BIT|QK_STOP_BIT|SWITCH_ON_BIT|EN_OP_BIT

#define CMD_QK_STOP EN_VOL_BIT  //�ڵ�ѹ���ο��£�QK Stopλ��0�����Ǽ�ͣ����
#define CMD_DIS_VOL 0x00

#define FAULT_RST FAULT_BIT



//״̬�궨��

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
