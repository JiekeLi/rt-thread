/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-05     Xeon Xu      the first version
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>




#define BS1SHIFT 16
#define BS2SHIFT 20
#define RRESCLSHIFT 0
#define SJWSHIFT 24
#define BS1MASK ( (0x0F) << BS1SHIFT )
#define BS2MASK ( (0x07) << BS2SHIFT )
#define RRESCLMASK ( 0x3FF << RRESCLSHIFT )
#define SJWMASK ( 0x3 << SJWSHIFT )

struct stm_baud_rate_tab
{
    rt_uint32_t baud_rate;
    rt_uint32_t confdata;
};

/* STM32 can driver */
struct stm32_drv_can
{
    CAN_HandleTypeDef CanHandle;
    CAN_TxHeaderTypeDef TxMessage;
    CAN_RxHeaderTypeDef RxMessage;
    CAN_FilterTypeDef FilterConfig;
	  unsigned char rcv_tmp_buf[8]; //���ݻ�����
};//�Ĵ�����ؽṹ��

//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1TQ~CAN_SJW_4TQ
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1TQ~CAN_BS2_8TQ;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1TQ~CAN_BS1_16TQ
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_MODE_NORMAL,��ͨģʽ;CAN_MODE_LOOPBACK,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ54M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_11tq,6,CAN_MODE_LOOPBACK);
//������Ϊ:54M/((6+11+1)*6)=500Kbps


static const struct stm_baud_rate_tab can_baud_rate_tab[] =
{
    {CAN1MBaud  , (CAN_SJW_1TQ | CAN_BS1_11TQ  | CAN_BS2_6TQ | 3)},
    {CAN500kBaud, (CAN_SJW_1TQ | CAN_BS1_11TQ | CAN_BS2_6TQ | 6)},
    {CAN250kBaud, (CAN_SJW_1TQ | CAN_BS1_11TQ  | CAN_BS2_6TQ | 12)},
    {CAN125kBaud, (CAN_SJW_1TQ | CAN_BS1_11TQ  | CAN_BS2_6TQ | 24)},
};//������

#define BAUD_DATA(TYPE,NO)                              \
    ((can_baud_rate_tab[NO].confdata & TYPE##MASK))

static rt_uint32_t get_can_baud_index(rt_uint32_t baud)
{
    rt_uint32_t len, index, default_index;

    len = sizeof(can_baud_rate_tab)/sizeof(can_baud_rate_tab[0]);
    default_index = len;

    for(index = 0; index < len; index++)
    {
        if(can_baud_rate_tab[index].baud_rate == baud)
            return index;

        if(can_baud_rate_tab[index].baud_rate == 1000UL * 1000) //Ĭ��1M
            default_index = index;
    }

    if(default_index != len)
        return default_index;

    return 0;
}



#ifdef USING_BXCAN1
static struct stm32_drv_can drv_can1; //can�Ĵ�����ؽṹ��
struct rt_can_device dev_can1;  //can�豸�����ṹ��
void CAN1_TX_IRQHandler(void)
{
    CAN_HandleTypeDef *hcan;

    rt_interrupt_enter();
    hcan = &drv_can1.CanHandle;

    HAL_CAN_IRQHandler(hcan);

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK0)) //����0�������
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_TX_DONE | 0 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_TX_FAIL | 0 << 8); //����ʧ��
    }

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK1)) //����1�������
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_TX_DONE | 1 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_TX_FAIL | 1 << 8);//����ʧ��
    }

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK2))//����2�������
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_TX_DONE | 2 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_TX_FAIL | 2 << 8);//����ʧ��
    }

    rt_interrupt_leave();
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	
		HAL_CAN_GetRxMessage(&drv_can1.CanHandle, 
	                              CAN_RX_FIFO0, 
	                       &drv_can1.RxMessage, 
	                       drv_can1.rcv_tmp_buf);
}


void CAN1_RX0_IRQHandler(void)
{
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can1.CanHandle;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV0)) //FIFO0���
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_RXOF_IND | 0 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_RX_IND | 0 << 8);//FIFO0�����ж�
    }

    rt_interrupt_leave();
}


void CAN1_RX1_IRQHandler(void)
{
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can1.CanHandle;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV1)) //FIFO1���
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_RXOF_IND | 1 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can1, RT_CAN_EVENT_RX_IND | 1 << 8);//FIFO1�����ж�
    }

    rt_interrupt_leave();
}


void CAN1_SCE_IRQHandler(void)
{
    rt_uint32_t errtype;
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can1.CanHandle;
    errtype = hcan->Instance->ESR; //�õ���������

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    if (errtype & 0x70 && dev_can1.status.lasterrtype == (errtype & 0x70))//����
    {
        switch ((errtype & 0x70) >> 4) //���ݴ������ͽ��м���
        {
        case RT_CAN_BUS_BIT_PAD_ERR:
            dev_can1.status.bitpaderrcnt++;
            break;
        case RT_CAN_BUS_FORMAT_ERR:
            dev_can1.status.formaterrcnt++;
            break;
        case RT_CAN_BUS_ACK_ERR:
            dev_can1.status.ackerrcnt++;
            break;
        case RT_CAN_BUS_IMPLICIT_BIT_ERR:
        case RT_CAN_BUS_EXPLICIT_BIT_ERR:
            dev_can1.status.biterrcnt++;
            break;
        case RT_CAN_BUS_CRC_ERR:
            dev_can1.status.crcerrcnt++;
            break;
        }
        dev_can1.status.lasterrtype = errtype & 0x70;
        hcan->Instance->ESR &= ~0x70;
    }
    dev_can1.status.rcverrcnt = errtype >> 24; //�õ����ܴ������
    dev_can1.status.snderrcnt = (errtype >> 16 & 0xFF); //�õ����ʹ������
    dev_can1.status.errcode = errtype & 0x07; //�õ���εĴ�������
    hcan->Instance->MSR |= CAN_MSR_ERRI;  //����жϱ�־
    rt_interrupt_leave();
}
#endif // USING_BXCAN1

#ifdef USING_BXCAN2
static struct stm32_drv_can drv_can2;
struct rt_can_device dev_can2;
/**
 * @brief This function handles CAN2 TX interrupts.
 */
void CAN2_TX_IRQHandler(void)
{
    CAN_HandleTypeDef *hcan;

    rt_interrupt_enter();
    hcan = &drv_can2.CanHandle;

    HAL_CAN_IRQHandler(hcan);

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK0))
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_TX_DONE | 0 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_TX_FAIL | 0 << 8);
    }

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK1))
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_TX_DONE | 1 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_TX_FAIL | 1 << 8);
    }

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK2))
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_TX_DONE | 2 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_TX_FAIL | 2 << 8);
    }

    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 RX0 interrupts.
 */
void CAN2_RX0_IRQHandler(void)
{
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can2.CanHandle;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV0))
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_RXOF_IND | 0 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_RX_IND | 0 << 8);
    }

    rt_interrupt_leave();

}

/**
 * @brief This function handles CAN2 RX1 interrupts.
 */
void CAN2_RX1_IRQHandler(void)
{
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can2.CanHandle;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV1))
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_RXOF_IND | 1 << 8);
    }
    else
    {
        rt_hw_can_isr(&dev_can2, RT_CAN_EVENT_RX_IND | 1 << 8);
    }

    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 SCE interrupts.
 */
void CAN2_SCE_IRQHandler(void)
{
    rt_uint32_t errtype;
    CAN_HandleTypeDef *hcan;

    hcan = &drv_can2.CanHandle;
    errtype = hcan->Instance->ESR;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    if (errtype & 0x70 && dev_can2.status.lasterrtype == (errtype & 0x70))
    {
        switch ((errtype & 0x70) >> 4)
        {
        case RT_CAN_BUS_BIT_PAD_ERR:
            dev_can2.status.bitpaderrcnt++;
            break;
        case RT_CAN_BUS_FORMAT_ERR:
            dev_can2.status.formaterrcnt++;
            break;
        case RT_CAN_BUS_ACK_ERR:
            dev_can2.status.ackerrcnt++;
            break;
        case RT_CAN_BUS_IMPLICIT_BIT_ERR:
        case RT_CAN_BUS_EXPLICIT_BIT_ERR:
            dev_can2.status.biterrcnt++;
            break;
        case RT_CAN_BUS_CRC_ERR:
            dev_can2.status.crcerrcnt++;
            break;
        }
        dev_can2.status.lasterrtype = errtype & 0x70;
        hcan->Instance->ESR &= ~0x70;
    }
    dev_can2.status.rcverrcnt = errtype >> 24;
    dev_can2.status.snderrcnt = (errtype >> 16 & 0xFF);
    dev_can2.status.errcode = errtype & 0x07;
    hcan->Instance->MSR |= CAN_MSR_ERRI;
    rt_interrupt_leave();
}

#endif // USING_BXCAN2







static rt_err_t drv_configure(struct rt_can_device *dev_can,
                              struct can_configure *cfg)
{
    struct stm32_drv_can *drv_can;
    rt_uint32_t baud_index;
    CAN_InitTypeDef *drv_init;
    CAN_FilterTypeDef *filterConf;

    RT_ASSERT(dev_can);
    RT_ASSERT(cfg);

    drv_can = (struct stm32_drv_can *)dev_can->parent.user_data; //�õ��Ĵ�����ؽṹ��
    drv_init = &drv_can->CanHandle.Init;    //�õ���ʼ����ز���

    drv_init->TimeTriggeredMode  = DISABLE;
    drv_init->AutoBusOff  = DISABLE;
    drv_init->AutoWakeUp  = DISABLE;
    drv_init->AutoRetransmission  = DISABLE;
    drv_init->ReceiveFifoLocked  = DISABLE;
    drv_init->TransmitFifoPriority  = DISABLE;

    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:
        drv_init->Mode = CAN_MODE_NORMAL;
        break;
    case RT_CAN_MODE_LISEN:
        drv_init->Mode = CAN_MODE_SILENT;
        break;
    case RT_CAN_MODE_LOOPBACK:
        drv_init->Mode = CAN_MODE_LOOPBACK;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        drv_init->Mode = CAN_MODE_SILENT_LOOPBACK;
        break;
    }

    baud_index = get_can_baud_index(cfg->baud_rate);
    drv_init->SyncJumpWidth  = BAUD_DATA(SJW, baud_index);
    drv_init->TimeSeg1  = BAUD_DATA(BS1, baud_index);
    drv_init->TimeSeg2 		= BAUD_DATA(BS2, baud_index);
    drv_init->Prescaler = BAUD_DATA(RRESCL, baud_index);
    if (HAL_CAN_Init(&drv_can->CanHandle) != HAL_OK)  //����CAN�Ĵ���
    {
        return RT_ERROR;
    }

    //����ֻ������һ��������
    filterConf = &drv_can->FilterConfig;
    filterConf->FilterBank = 0;
    filterConf->FilterMode = CAN_FILTERMODE_IDMASK;
    filterConf->FilterScale = CAN_FILTERSCALE_32BIT;
    filterConf->FilterIdHigh = 0x0000;
    filterConf->FilterIdLow = 0x0000;
    filterConf->FilterMaskIdHigh = 0x0000;
    filterConf->FilterMaskIdLow = 0x0000;
    filterConf->FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filterConf->FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&drv_can->CanHandle, filterConf); //���ù������Ĵ���
		
		
		if(HAL_CAN_Start(&drv_can->CanHandle)!=RT_EOK){
			return RT_ERROR;
		}

    return RT_EOK;
}

static rt_err_t drv_control(struct rt_can_device *can, int cmd, void *arg)
{
    struct stm32_drv_can *drv_can;
    rt_uint32_t argval;

    drv_can = (struct stm32_drv_can *) can->parent.user_data;
    assert_param(drv_can != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:  //�ر��ж�
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)  //�رս����ж�
        {
            if (CAN1 == drv_can->CanHandle.Instance){
                HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
                HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
            }
            else
            {
                HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
                HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
            }
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_FULL );
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_OVERRUN);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_MSG_PENDING);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_FULL );
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_OVERRUN);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)  //�رշ����ж�
        {
            if (CAN1 == drv_can->CanHandle.Instance)
            {
                HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
            }
            else
            {
                HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
            }
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_TX_MAILBOX_EMPTY);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR) //�رմ����ж�
        {
            if (CAN1 == drv_can->CanHandle.Instance)
            {
                NVIC_DisableIRQ(CAN1_SCE_IRQn);
            }
            else
            {
                NVIC_DisableIRQ(CAN2_SCE_IRQn);
            }
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_BUSOFF);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_LAST_ERROR_CODE);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_ERROR);
        }
        break;
    case RT_DEVICE_CTRL_SET_INT:  //���ж�
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX) //�������ж�
        {
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_FULL);
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_OVERRUN);
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_MSG_PENDING);
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_FULL);
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_OVERRUN);

            if (CAN1 == drv_can->CanHandle.Instance)
            {
                HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
                HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
            }
            else
            {
                HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
                HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
            }
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX) //�������ж�
        {
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_TX_MAILBOX_EMPTY);

            if (CAN1 == drv_can->CanHandle.Instance)
            {
                HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
            }
            else
            {
                HAL_NVIC_SetPriority(CAN2_TX_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
            }
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR) //�������ж�
        {
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_BUSOFF);
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_LAST_ERROR_CODE);
            __HAL_CAN_ENABLE_IT(&drv_can->CanHandle, CAN_IT_ERROR);

            if (CAN1 == drv_can->CanHandle.Instance)
            {
                HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
            }
            else
            {
                HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 1, 2);
                HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
            }
        }
        break;
    case RT_CAN_CMD_SET_FILTER:  //���ù�������ʵ����û��ʵ��
        /* TODO: filter*/
        break;
    case RT_CAN_CMD_SET_MODE:   //����CANģʽ
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_NORMAL ||
            argval != RT_CAN_MODE_LISEN ||
            argval != RT_CAN_MODE_LOOPBACK ||
            argval != RT_CAN_MODE_LOOPBACKANLISEN)
        {
            return RT_ERROR;
        }
        if (argval != can->config.mode)
        {
            can->config.mode = argval;
            if (HAL_CAN_Init(&drv_can->CanHandle) != HAL_OK) //���³�ʼ��һ��
            {
                return RT_ERROR;
            }
        }
        break;
    case RT_CAN_CMD_SET_BAUD:  //���ò�����
        argval = (rt_uint32_t) arg;
        if (argval != CAN1MBaud &&
            argval != CAN800kBaud &&
            argval != CAN500kBaud &&
            argval != CAN250kBaud &&
            argval != CAN125kBaud &&
            argval != CAN100kBaud &&
            argval != CAN50kBaud  &&
            argval != CAN20kBaud  &&
            argval != CAN10kBaud)
        {
            return RT_ERROR;
        }
        if (argval != can->config.baud_rate)
        {
            CAN_InitTypeDef *drv_init;
            rt_uint32_t baud_index;

            can->config.baud_rate = argval;

            drv_init = &drv_can->CanHandle.Init;

            drv_init->TimeTriggeredMode = DISABLE;
            drv_init->AutoBusOff  = DISABLE;
            drv_init->AutoWakeUp  = DISABLE;
            drv_init->AutoRetransmission  = DISABLE;
            drv_init->ReceiveFifoLocked = DISABLE;
            drv_init->TransmitFifoPriority  = DISABLE;
            baud_index = get_can_baud_index(can->config.baud_rate);
            drv_init->SyncJumpWidth  = BAUD_DATA(SJW, baud_index);
            drv_init->TimeSeg1  = BAUD_DATA(BS1, baud_index);
            drv_init->TimeSeg2  = BAUD_DATA(BS2, baud_index);
            drv_init->Prescaler = BAUD_DATA(RRESCL, baud_index);

            if (HAL_CAN_Init(&drv_can->CanHandle) != HAL_OK) //���³�ʼ��һ��
            {
                return RT_ERROR;
            }
        }
        break;
    case RT_CAN_CMD_SET_PRIV: //����privmode ??
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_PRIV ||
            argval != RT_CAN_MODE_NOPRIV)
        {
            return RT_ERROR;
        }
        if (argval != can->config.privmode)
        {
            can->config.privmode = argval;
            if (HAL_CAN_Init(&drv_can->CanHandle) != HAL_OK)//���³�ʼ��
            {
                return RT_ERROR;
            }
        }
        break;
    case RT_CAN_CMD_GET_STATUS:  //�õ���ǰ״̬
    {
        rt_uint32_t errtype;
        errtype = drv_can->CanHandle.Instance->ESR;
        can->status.rcverrcnt = errtype >> 24;
        can->status.snderrcnt = (errtype >> 16 & 0xFF);
        can->status.errcode = errtype & 0x07;
        if (arg != &can->status)
        {
            rt_memcpy(arg, &can->status, sizeof(can->status));
        }
    }
    break;
    }

    return RT_EOK;
}

/*
*�ײ㷢�ͺ���
*������can�豸����ָ��
*      buf����ָ��
*      boxno������Ϣ����
*
*/

static int drv_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    CAN_HandleTypeDef *hcan;
	  struct stm32_drv_can *drv_can;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
	  uint32_t  transmitmailbox;
   
	  drv_can = (struct stm32_drv_can *) can->parent.user_data;
    hcan = &((struct stm32_drv_can *) can->parent.user_data)->CanHandle; //�õ������Ĵ����ľ��
	
	  drv_can->TxMessage.StdId = pmsg->id;
	  drv_can->TxMessage.ExtId = pmsg->id;
	  drv_can->TxMessage.RTR =   pmsg->rtr;
	  drv_can->TxMessage.IDE = pmsg->ide;
	  drv_can->TxMessage.DLC =  pmsg->len;
	
	  
	   
	 // hcan->pTxMsg->StdId = pmsg->id;
    //hcan->pTxMsg->RTR = pmsg->rtr;
   // hcan->pTxMsg->IDE = pmsg->ide;
    //hcan->pTxMsg->DLC = pmsg->len;
    //rt_memset(&hcan->pTxMsg->Data, 0x00, 8);
    /* rt_memcpy(&hcan->pTxMsg->Data, &pmsg->data, 8); */
   /* hcan->pTxMsg->Data[0] = pmsg->data[0];
    hcan->pTxMsg->Data[1] = pmsg->data[1];
    hcan->pTxMsg->Data[2] = pmsg->data[2];
    hcan->pTxMsg->Data[3] = pmsg->data[3];
    hcan->pTxMsg->Data[4] = pmsg->data[4];
    hcan->pTxMsg->Data[5] = pmsg->data[5];
    hcan->pTxMsg->Data[6] = pmsg->data[6];
    hcan->pTxMsg->Data[7] = pmsg->data[7];
    HAL_CAN_Transmit_IT(hcan);*/
	
    if(HAL_CAN_AddTxMessage(hcan, 
			                     (CAN_TxHeaderTypeDef*)&drv_can->TxMessage,
		                        pmsg->data,&transmitmailbox)!=HAL_OK)
		{
			return RT_ERROR;
		}
		
		return RT_EOK;
}



static int drv_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{
    struct stm32_drv_can *drv_can;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    drv_can = (struct stm32_drv_can *) can->parent.user_data;
	

    pmsg->ide = drv_can->RxMessage.IDE;

	  if(pmsg->ide==0){
      pmsg->id = drv_can->RxMessage.StdId;
		}else{
			pmsg->id = drv_can->RxMessage.ExtId;
		}
    pmsg->rtr = drv_can->RxMessage.RTR;
    pmsg->len = drv_can->RxMessage.DLC;

    pmsg->data[0] = drv_can->rcv_tmp_buf[0];
    pmsg->data[1] = drv_can->rcv_tmp_buf[1];
    pmsg->data[2] = drv_can->rcv_tmp_buf[2];
    pmsg->data[3] = drv_can->rcv_tmp_buf[3];
    pmsg->data[4] = drv_can->rcv_tmp_buf[4];
    pmsg->data[5] = drv_can->rcv_tmp_buf[5];
    pmsg->data[6] = drv_can->rcv_tmp_buf[6];
    pmsg->data[7] = drv_can->rcv_tmp_buf[7];

    return RT_EOK;
}

//����������������
static const struct rt_can_ops drv_can_ops =
{
    drv_configure,
    drv_control,
    drv_sendmsg,
    drv_recvmsg,
};

//��ʼ��ʱ�� �ж� ����
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
    GPIO_InitTypeDef GPIO_Initure;
    if(canHandle->Instance==CAN1)
    {
       __HAL_RCC_CAN1_CLK_ENABLE();                //ʹ��CAN1ʱ��
       __HAL_RCC_GPIOA_CLK_ENABLE();			    //����GPIOAʱ��


			GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12;   //PA11,12���ų�ʼ�� 
			GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //���츴��
			GPIO_Initure.Pull=GPIO_PULLUP;              //����
			GPIO_Initure.Speed=GPIO_SPEED_FAST;         //����
			GPIO_Initure.Alternate=GPIO_AF9_CAN1;       //����ΪCAN1
			HAL_GPIO_Init(GPIOA,&GPIO_Initure);         //��ʼ��
			
	/*		__HAL_CAN_ENABLE_IT(canHandle,CAN1_RX0_IRQn);//FIFO0�����ж�  
      HAL_NVIC_SetPriority(CAN1_RX0_IRQn,1,2);    //��ռ���ȼ�1�������ȼ�2
      HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);          //ʹ���ж�
			
			__HAL_CAN_ENABLE_IT(canHandle,CAN1_RX1_IRQn);//FIFO1�����ж�  
      HAL_NVIC_SetPriority(CAN1_RX0_IRQn,1,3);    //��ռ���ȼ�1�������ȼ�3
      HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);          //ʹ���ж�
			
		  __HAL_CAN_ENABLE_IT(canHandle,CAN1_TX_IRQn);//FIFO0���� 
      HAL_NVIC_SetPriority(CAN1_TX_IRQn,1,4);    //��ռ���ȼ�1�������ȼ�4
      HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);          //ʹ���ж�
			
			__HAL_CAN_ENABLE_IT(canHandle,CAN1_SCE_IRQn);//�����ж� 
      HAL_NVIC_SetPriority(CAN1_TX_IRQn,1,5);    //��ռ���ȼ�1�������ȼ�5
      HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);          //ʹ���ж�
*///�ж��ڳ�ʼ��ʱ����
    }
    else if(canHandle->Instance==CAN2)
    {
        /* CAN2 clock enable */
        __HAL_RCC_CAN2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**CAN2 GPIO Configuration
           PB12     ------> CAN2_RX
           PB6     ------> CAN2_TX
        */
        GPIO_Initure.Pin = GPIO_PIN_12|GPIO_PIN_6;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_NOPULL;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_Initure.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_Initure);


    }
}

//����ʼ������ ʱ�� �ж�
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

    if(canHandle->Instance==CAN1)
    {

        __HAL_RCC_CAN1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
			  HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
			  HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);

    }
    else if(canHandle->Instance==CAN2)
    {
        __HAL_RCC_CAN2_CLK_DISABLE();

        /**CAN2 GPIO Configuration
           PB12     ------> CAN2_RX
           PB6     ------> CAN2_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_6);

        HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
        HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
        HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    }
}

int hw_can_init(void)
{
    struct stm32_drv_can *drv_can;
    struct can_configure config = CANDEFAULTCONFIG;
	
	  config.mode = RT_CAN_MODE_NORMAL;
    config.privmode = 0;
    config.ticks = 50;
    config.sndboxnumber = 3;
#ifdef RT_CAN_USING_HDR
    config.maxhdr = 28;
#endif

#ifdef USING_BXCAN1
    drv_can = &drv_can1;
    drv_can->CanHandle.Instance = CAN1;
    dev_can1.ops    = &drv_can_ops;
    dev_can1.config = config;

    rt_hw_can_register(&dev_can1, "can1",
                       &drv_can_ops,
                       drv_can);
#endif 

#ifdef USING_BXCAN2
    drv_can = &drv_can2;
    drv_can->CanHandle.Instance = CAN2;
    dev_can2.ops    = &drv_can_ops;
    dev_can2.config = config;

    rt_hw_can_register(&dev_can2, "can2",
                       &drv_can_ops,
                       drv_can);
#endif 

    return 0;
}

INIT_BOARD_EXPORT(hw_can_init);

