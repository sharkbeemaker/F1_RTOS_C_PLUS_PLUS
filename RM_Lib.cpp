
/**
  ******************************************************************************
  * File Name				: RM_Lib.cpp
  * Description			: RM c++库，包含USART，CAN，麦轮底盘算法，PID，遥控器� ?
											�?螺仪，snail电调，裁判系统交互等库函� ?
  * Version					: 1.2
  * Creation Date		: 2022.10.8
  ******************************************************************************
  */
#include "RM_Lib.hpp"

/**************************************** USART **********************************************************/
uint8_t info_ubuf[USART_BUF_SIZE];

/**************************************** C A N **********************************************************/
void USER_CAN::Init(uint16_t t,uint16_t x)
{
  CAN_FilterTypeDef sFilterConfig={0};

	this->FreeTxNum=0;
	this->TxHeader.DLC=8;

  sFilterConfig.FilterBank = t;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;

	if(this->FIFO == 0) sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	else sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;

	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = x;

  if (HAL_CAN_ConfigFilter(this->hcan, &sFilterConfig) != HAL_OK)Error_Handler();
  if (HAL_CAN_Start(this->hcan) != HAL_OK)Error_Handler();

	if(this->FIFO == 0)
	{
		if (HAL_CAN_ActivateNotification(this->hcan,CAN_IT_RX_FIFO0_MSG_PENDING)!= HAL_OK)
			Error_Handler();
	}
	else
	{
		if (HAL_CAN_ActivateNotification(this->hcan,CAN_IT_RX_FIFO1_MSG_PENDING)!= HAL_OK)
			Error_Handler();
	}
}
HAL_StatusTypeDef USER_CAN::Send(uint16_t Id,uint8_t* pData)
{
    this->TxHeader.StdId=Id;
    do {this->FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(this->hcan);}
    while (this->FreeTxNum == 0);
    return HAL_CAN_AddTxMessage(this->hcan,&this->TxHeader, pData,&this->TxMailbox);
}
HAL_StatusTypeDef USER_CAN::Send_RM(uint16_t Id,int16_t M_201, int16_t M_202, int16_t M_203,int16_t M_204)
{
	uint8_t pData[8];
	pData[0] = (int8_t) (M_201 >> 8);
	pData[1] = (int8_t) M_201;
	pData[2] = (int8_t) (M_202 >> 8);
	pData[3] = (int8_t) M_202;
	pData[4] = (int8_t) (M_203 >> 8);
	pData[5] = (int8_t) M_203;
	pData[6] = (int8_t) (M_204 >> 8);
	pData[7] = (int8_t) M_204;
	return this->Send(Id,pData);
}

HAL_StatusTypeDef USER_CAN::Receive(CAN_HandleTypeDef *hcan)
{
		if(this->FIFO == 0) return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO0,&this->RxHeader,this->rx_buf);
		else return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO1,&this->RxHeader,this->rx_buf);
}

HAL_StatusTypeDef MOTOR_RM::update(void)
{
	if(this->can_rev->RxHeader.StdId == this->ID)
	{
		this->mang=(int16_t) ((this->can_rev->rx_buf[0] << 8) | this->can_rev->rx_buf[1]);
		this->sp=(int16_t) ((this->can_rev->rx_buf[2] << 8) | this->can_rev->rx_buf[3]);
		this->AT_current=(int16_t) ((this->can_rev->rx_buf[4] << 8) | this->can_rev->rx_buf[5]);
		this->temp=this->can_rev->rx_buf[6];

		this->update_mang_inf();

		return HAL_OK;
	}
	else return HAL_ERROR;
}

void MOTOR_RM::update_mang_inf(void)
{
    if(this->first == 0)this->first=1,this->Last_mang=this->mang;

    if ((this->mang - this->Last_mang) < -5000)
    {
        this->mang_inf += 8191;
			  this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 5000)
    {
        this->mang_inf -= 8191;
			  this->motor_number--;
    }
    this->mang_inf -= this->Last_mang;
    this->mang_inf += this->mang;
    this->Last_mang = this->mang;
}
/**********************************  Cyber_Gear  ********************************************/
/*
id(bit 24-28)    描述
0								获取设备ID
1								运控模式控制电机(力矩控制)
3 							使能电机运行
4								电机停止运行
6								设置机械零位(掉电丢失)
7								设置电机CAN_ID(立即生效)
17   						单个参数读取
18 							单个参数写入
*/
HAL_StatusTypeDef USER_CAN::EXT_ID_Send(uint32_t Id,uint8_t* pData)
{
    this->TxHeader.IDE=CAN_ID_EXT;
		this->TxHeader.RTR=CAN_RTR_DATA;
		this->TxHeader.ExtId=Id;
		this->TxHeader.DLC=8;
    do {this->FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(this->hcan);}
    while (this->FreeTxNum == 0);
    return HAL_CAN_AddTxMessage(this->hcan,&this->TxHeader, pData,&this->TxMailbox);
}
HAL_StatusTypeDef Cyber_Gear::update(void)
{
	 if(Cyber_Gear_Motor_Id_Get(this->can_rev->RxHeader.ExtId) == this->MOTOR_ID)
	{
		this->angle=uint_to_float(this->can_rev->rx_buf[0]<<8|this->can_rev->rx_buf[1],-4*PI,4*PI,16);
		this->sp=uint_to_float(this->can_rev->rx_buf[2]<<8|this->can_rev->rx_buf[3],-30,30,16);
		this->torque=uint_to_float(this->can_rev->rx_buf[4]<<8|this->can_rev->rx_buf[5],-12,12,16);
		this->temp=1.0*(this->can_rev->rx_buf[6]<<8|this->can_rev->rx_buf[7])/10;
		return HAL_OK;
	}
	else
	{
		INFO("%02x\n",Cyber_Gear_Motor_Id_Get(this->can_rev->RxHeader.ExtId));
		return HAL_ERROR;
	}
}
HAL_StatusTypeDef USER_CAN::Cyber_Gear_torque_Send(uint8_t motor_id,float torque)
{
	uint8_t Data[8]={0X7F,0XFF,0X7F,0XFF,0,0,0,0};
		return this->EXT_ID_Send(Cyber_Gear_EXTID_SET(1,motor_id,float_to_uint(torque,T_MIN,T_MAX,16)),Data);
}
HAL_StatusTypeDef USER_CAN::Cyber_Gear_Stop(uint8_t motor_id)//停止工作
{
	uint8_t data[8]={0,0,0,0,0,0,0,0};
	return this->EXT_ID_Send(Cyber_Gear_EXTID_SET(4,motor_id,My_id),data);
}
HAL_StatusTypeDef USER_CAN::Cyber_Gear_Enable(uint8_t motor_id)//使能工作
{
	uint8_t data[8]={0,0,0,0,0,0,0,0};
	return this->EXT_ID_Send(Cyber_Gear_EXTID_SET(3,motor_id,My_id),data);
}

uint32_t Cyber_Gear_EXTID_SET(uint8_t mode, uint8_t Motor_id, uint16_t data)//设置EXTID
{
	if(mode==0||mode==3||mode==4||mode==17||mode==6)
	{
		return ((uint32_t)mode<<24|(uint32_t)My_id<<8|Motor_id)& 0x1FFFFFFF;
	}
  else  return ((uint32_t)mode << 24 | (uint32_t)data << 8 | Motor_id) & 0x1FFFFFFF; // � ?29� ?
}
uint8_t Cyber_Gear_Motor_Id_Get(uint32_t EXTID)//针对通讯协议2,从ID里面读取电机ID
{
	return (EXTID>>8)&0xFF;
}
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int value, float x_min, float x_max, int bits)
{
    return ((float)value * (x_max - x_min)) / (float)((1 << bits) - 1) + x_min;
}
/********************************************************************************************/
/********************************* 	 DM_4310  	*********************************************/

/*

tips:   这个达妙电机有时候重新烧录代码后� ?1s之内不能动电机，不然会死机，尽量不要频繁发�?�电机启动命� ?
使用DM4310的时候要在main函数设置电机原点然后启动电机，达妙电机会�?直发送报文给我们
第一个数据是err，内容代表如下：

*/
HAL_StatusTypeDef USER_CAN::DM_Start(uint16_t Id)//进入电机  发�?�指� ?
{
  uint8_t pData[8];
	pData[0] = 0xFF;
	pData[1] = 0xFF;
	pData[2] = 0xFF;
	pData[3] = 0xFF;
	pData[4] = 0xFF;
	pData[5] = 0xFF;
	pData[6] = 0xFF;
	pData[7] = 0xFC;
	return this->Send(Id,pData);
}
HAL_StatusTypeDef USER_CAN::DM_End(uint16_t Id)//�?出电� ?   发�?�指� ?
{
  uint8_t pData[8];
	pData[0] = 0xFF;
	pData[1] = 0xFF;
	pData[2] = 0xFF;
	pData[3] = 0xFF;
	pData[4] = 0xFF;
	pData[5] = 0xFF;
	pData[6] = 0xFF;
	pData[7] = 0xFD;
	return this->Send(Id,pData);
}
HAL_StatusTypeDef USER_CAN::DM_Savezero(uint16_t Id)//保存位置零点   发�?�指� ?
{
  uint8_t pData[8];
	pData[0] = 0xFF;
	pData[1] = 0xFF;
	pData[2] = 0xFF;
	pData[3] = 0xFF;
	pData[4] = 0xFF;
	pData[5] = 0xFF;
	pData[6] = 0xFF;
	pData[7] = 0xFE;
	return this->Send(Id,pData);
}

//int float_to_uint(float x, float x_min, float x_max, int bits){
//    /// Converts a float to an unsigned int, given range and number of bits ///
//    float span = x_max - x_min;//25.0
//    float offset = x_min;//-12.5
//    return (int) ((x-offset)*((float)((1<<bits)-1))/span);//12.5*
//    }
//
//
//float uint_to_float(int x_int, float x_min, float x_max, int bits){
//    /// converts unsigned int to float, given range and number of bits ///
//    float span = x_max - x_min;
//    float offset = x_min;
//    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
//    }
/**
	* @brief  MIT模式控下控制� ?,kp=0,kd不为0（kd� ?0会震荡）
 * @param  hcan   CAN的句� ?
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定� ?
 */
HAL_StatusTypeDef USER_CAN::DM_MIT(uint16_t Id, float _pos, float _vel, float _KP, float _KD, float _torq)//MIT 模式
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	uint8_t pData[8];
	pData[0] = (pos_tmp >> 8);
	pData[1] = pos_tmp;
	pData[2] = (vel_tmp >> 4);
	pData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	pData[4] = kp_tmp;
	pData[5] = (kd_tmp >> 4);
	pData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	pData[7] = tor_tmp;
	return this->Send(Id,pData);
}
/**
 * @brief� ? ID 为设定的 CAN ID 值加� ? 0x100 的偏� ?
 * @param _pos：位置给定，浮点型，低位在前，高位在� ?
 * @param _vel：�?�度给定，浮点型，低位在前，高位在后
 * @param
 * @param 此处发�?�命令的 CAN ID � ? 0x100+ID。�?�度给定是梯形加速度运行下最高�?�度的，即为�?速段的�?�度值�?
 */
HAL_StatusTypeDef USER_CAN::DM_POS(uint16_t Id, float _pos, float _vel)//位置速度模式
{
	uint8_t *pbuf,*vbuf;
	pbuf=(uint8_t*)&_pos;
	vbuf=(uint8_t*)&_vel;

	uint8_t pData[8];
	pData[0] = *pbuf;
	pData[1] = *(pbuf+1);
	pData[2] = *(pbuf+2);
	pData[3] = *(pbuf+3);
	pData[4] = *vbuf;
	pData[5] = *(vbuf+1);
	pData[6] = *(vbuf+2);
	pData[7] = *(vbuf+3);
	return this->Send(Id,pData);
}

/**
 * @brief  速度模式控下控制� ?
 * @param  hcan   CAN的句� ?
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */

HAL_StatusTypeDef USER_CAN::DM_VEL(uint16_t Id, float _vel)//速度模式
{
	uint8_t *vbuf;
	vbuf=(uint8_t*)&_vel;

	uint8_t pData[4];
	pData[0] = *vbuf;
	pData[1] = *(vbuf+1);
	pData[2] = *(vbuf+2);
	pData[3] = *(vbuf+3);

	return this->Send(Id,pData);
}

HAL_StatusTypeDef MOTOR_DM::DM_update(void)//得到数据
{
	if(this->can_rev->RxHeader.StdId ==ID)
	{
        id = (this->can_rev->rx_buf[0])&0x0F;
				ERR = (this->can_rev->rx_buf[0])>>4;
			  p_int=((this->can_rev->rx_buf[1]<<8)|this->can_rev->rx_buf[2]);
				v_int=(this->can_rev->rx_buf[3]<<4)|(this->can_rev->rx_buf[4]>>4)%16384;
				t_int=((this->can_rev->rx_buf[4]&0xF)<<8)|this->can_rev->rx_buf[5];
				mang = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
				sp = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
				Torque = uint_to_float(t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
				T_Rotor = (float)(this->can_rev->rx_buf[6]);
				T_MOS = (float)(this->can_rev->rx_buf[7]);
		return HAL_OK;
	}
	else
		return HAL_ERROR;
}

/********************************************************************************************/

/*************************************  RMD  ************************************************/

/*
Order_Id :  0x30, 读取当前电机的PID
            0x33，读取加速度参数
            0x90,读取编码器的当前位置
            0x19，写入当前位置到ROM作为电机零点，多次使用会影响芯片寿命
						0x92,读取多圈角度
						0x94,读取单圈角度
						0x95，清楚电机角� ?
						0x9A,读取电机状�?1和错误标� ?
            0x9B,清除电机错误标志
						0x9C,读取电机状�?2
						0x9D,读取电机状�?3
            0x80,电机关闭
						0x81,电机停止
            0x88,电机运行
*/
HAL_StatusTypeDef USER_CAN::RMD_Read_and_Write_Things(uint16_t Id,uint16_t Order_Id)
{
	uint8_t pData[8];
	pData[0]=Order_Id;
	pData[1] =0x00;
	pData[2] =0x00;
	pData[3] =0x00;
	pData[4] =0x00;
	pData[5] =0x00;
	pData[6] =0x00;
	pData[7] =0x00;
	return this->Send(Id,pData);
}

/*
Order_Id: 0x31,写入PID参数到RAM，断电后参数失效
          0x32,写入PID参数到ROM，断电后仍然有效
*/
HAL_StatusTypeDef USER_CAN::RMD_Write_PID(uint16_t Id,uint16_t Order_Id,uint16_t anglePidKp,uint16_t anglePidKi,uint16_t speedPidKp,uint16_t speedPidKi,uint16_t iqPidKp,uint16_t iqPidKi)
{
	uint8_t pData[8];
	pData[0] =Order_Id;
  pData[1] =0x00;
	pData[2] =anglePidKp;
	pData[3] =anglePidKi;
	pData[4] =speedPidKp;
	pData[5] =speedPidKi;
	pData[6] =iqPidKp;
	pData[7] =iqPidKi;
	return this->Send(Id,pData);
}
/*
0x34,写入加�?�度到RAM
*/
HAL_StatusTypeDef USER_CAN::RMD_Write_ACCLE_to_RAM(uint16_t Id,int32_t Accel)
{
	uint8_t pData[8];
	pData[0] = 0x34;
  pData[1] =0x00;
	pData[2] =0x00;
	pData[3] =0x00;
	pData[4] =Accel;
	pData[5] =(Accel>>8);
	pData[6] =(Accel>>16);
	pData[7] =(Accel>>24);
	return this->Send(Id,pData);
}

/*
0x91，写入编码器值到ROM作为电机零点
*/
HAL_StatusTypeDef USER_CAN::RMD_Write_EncoderOffset_to_ROM(uint16_t Id,uint16_t EncoderOffset)
{
	uint8_t pData[8];
	pData[0] = 0x91;
  pData[1] =0x00;
	pData[2] =0x00;
	pData[3] =0x00;
	pData[4] =0x00;
	pData[5] =0x00;
	pData[6] =EncoderOffset;
	pData[7] =EncoderOffset>>8;
	return this->Send(Id,pData);
}

/*
0xA1,转矩闭环控制
*/
HAL_StatusTypeDef USER_CAN::RMD_Iqcontrol_Motor(uint16_t Id,int16_t iqControl)
{
	uint8_t pData[8];
	pData[0] =0XA1;
  pData[1] =0x00;
	pData[2] =0x00;
	pData[3] =0x00;
	pData[4] =iqControl;
	pData[5] =iqControl>>8;
	pData[6] =0x00;
	pData[7] =0x00;
	return this->Send(Id,pData);
}

/*
Order_Id� ? 0xA2,速度闭环控制
					  0xA3,位置闭环控制命令1
						0xA4,位置闭环控制命令2
						0xA7,位置闭环控制命令5
						0xA8,位置闭环控制命令6
*/
HAL_StatusTypeDef USER_CAN::RMD_Speedcontrol_Motor(uint16_t Id,uint16_t Order_Id,uint16_t maxSpeed,int32_t angleControl)
{
	uint8_t pData[8];
	pData[0] =Order_Id;
  pData[1] =0x00;
	pData[2] =maxSpeed;
	pData[3] =maxSpeed>>8;
	pData[4] =angleControl;
	pData[5] =angleControl>>8;
	pData[6] =angleControl>>16;
	pData[7] =angleControl>>24;
	return this->Send(Id,pData);
}

/*
Order_Id� ?0xA5,位置闭环控制命令3，spinDirection 0x00代表顺时针，0x01代表逆时� ?
					0xA6,位置闭环控制命令4
*/
HAL_StatusTypeDef USER_CAN::RMD_Anglecontrol_Motor(uint16_t Id,uint16_t Order_Id,uint8_t spinDirection,uint16_t maxSpeed,uint16_t angleControl)
{
	uint8_t pData[8];
	pData[0] =Order_Id;
  pData[1] =spinDirection;
	pData[2] =maxSpeed;
	pData[3] =maxSpeed>>8;
	pData[4] =angleControl;
	pData[5] =angleControl>>8;
	pData[6] =0x00;
	pData[7] =0x00;
	return this->Send(Id,pData);
}

HAL_StatusTypeDef USER_CAN::RMD_Receive(CAN_HandleTypeDef *hcan)
{
		if(this->FIFO == 0) return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO0,&this->RxHeader,this->rx_buf);
		else return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO1,&this->RxHeader,this->rx_buf);
}

HAL_StatusTypeDef MOTOR_RMD::RMD_update(void)
{
	if(this->can_rev->RxHeader.StdId == this->ID)
	{
		switch(this->can_rev->rx_buf[0])
			{
				case 0x30:
				{
					this->RMD_X.anglePidKp=this->can_rev->rx_buf[2];
					this->RMD_X.anglePidKi=this->can_rev->rx_buf[3];
					this->RMD_X.speedPidKp=this->can_rev->rx_buf[4];
					this->RMD_X.speedPidKi=this->can_rev->rx_buf[5];
					this->RMD_X.iqPidKp=this->can_rev->rx_buf[6];
					this->RMD_X.iqPidKi=this->can_rev->rx_buf[7];
					 break;
				}
				case 0x33:
				{
					this->RMD_X.Accel=(this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8)|(this->can_rev->rx_buf[6]<<16)|(this->can_rev->rx_buf[7]<<24));
					break;
				}
				case 0x90:
				{
					this->RMD_X.encoder = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));
			  	this->RMD_X.encoderRaw = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));
				  this->RMD_X.encoderOffset = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
					break;
				}
				case 0x19:
				{
					this->RMD_X.encoderOffset = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
					break;
				}
				case 0x92:
				{
					this->RMD_X.motorAngle=(this->can_rev->rx_buf[1]|this->can_rev->rx_buf[2]<<8|(this->can_rev->rx_buf[3]<<16)|(this->can_rev->rx_buf[4]<<24));	//没完� ?
					break;
				}
				case 0x94:
				{
					this->RMD_X.circleAngle  = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
					break;
				}
			  case 0x9A:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
          this->RMD_X.voltage = (this->can_rev->rx_buf[3]|(this->can_rev->rx_buf[4]<<8));
					this->RMD_X.eerorState  = this->can_rev->rx_buf[7]&0x09; //1 低压保护� ?8 过温保护 � ?9低压保护，过温保� ?
					break;
				}
			  case 0x9C:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
					break;
				}
				case 0x9D:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iA = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.iB = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.iC = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
					break;
				}
			  case 0xA1:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
					break;
				}
				case 0xA2:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
          break;
				}
				case 0xA3:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
          break;
				}
				case 0xA4:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
          break;
				}
				case 0xA5:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
          break;
				}
				case 0xA6:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
          break;
				}
				case 0xA7:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
          break;
				}
				case 0xA8:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
          break;
				}
			}
		return HAL_OK;
	}
	else return HAL_ERROR;
}

/*************************************************************************************************************/

MOTOR_DiPan::MOTOR_DiPan(void)
{
	this->ML.qz = 0;
	this->ML.qy = 0;
	this->ML.hz = 0;
	this->ML.hy = 0;
}

void MOTOR_DiPan::ML_Data_Deal(float lx,float ly,float lp,int MAX_rate)
{
	float qy_in,qz_in,hy_in,hz_in;
	float sqy_in,sqz_in,shy_in,shz_in,sx,sp,sy;
	float sqy_per,sqz_per,shz_per,shy_per;
	float Y_max,M_max,P_max;
	float qy_per,qz_per,hz_per,hy_per,Y_per,P_per;

	if(ly>0)sy=ly;else sy=-ly;
	if(lp>0)sp=lp;else sp=-lp;
	if(lx>0)sx=lx;else sx=-lx;

	Y_max=sy;

	if(Y_max<sx)Y_max=sx;
	if(Y_max<sp)Y_max=sp;

	Y_per=Y_max/660;

	qy_in=lx+ly-lp;
	qz_in=lx-ly+lp;
	hy_in=lx-ly-lp;
	hz_in=lx+ly+lp;

	if(qy_in>0)sqy_in=qy_in;else sqy_in=-qy_in;
	if(qz_in>0)sqz_in=qz_in;else sqz_in=-qz_in;
	if(hy_in>0)shy_in=hy_in;else shy_in=-hy_in;
	if(hz_in>0)shz_in=hz_in;else shz_in=-hz_in;

	M_max=sqy_in;
	if(M_max<sqz_in)M_max=sqz_in;
	if(M_max<shy_in)M_max=shy_in;
	if(M_max<shz_in)M_max=shz_in;

	if(M_max==0)
	{
		qy_per=0;
		qz_per=0;
		hy_per=0;
		hz_per=0;
	}
	else {
		qy_per=qy_in/M_max;
		qz_per=qz_in/M_max;
		hy_per=hy_in/M_max;
		hz_per=hz_in/M_max;
	}

	if(qy_per>0)sqy_per=qy_per;else sqy_per=-qy_per;
	if(qz_per>0)sqz_per=qz_per;else sqz_per=-qz_per;
	if(hy_per>0)shy_per=hy_per;else shy_per=-hy_per;
	if(hz_per>0)shz_per=hz_per;else shz_per=-hz_per;

	P_max=sqy_per;
	if(P_max<sqz_per)P_max=sqz_per;
	if(P_max<shy_per)P_max=shy_per;
	if(P_max<shz_per)P_max=shz_per;

	P_per=(P_max*4-(sqy_per+sqz_per+shy_per+shz_per))/3.1415926f+1;
//		P_per=1;

	this->ML.qy=-qy_per*Y_per*MAX_rate*P_per;
	this->ML.qz=qz_per*Y_per*MAX_rate*P_per;
	this->ML.hy=-hy_per*Y_per*MAX_rate*P_per;
	this->ML.hz=hz_per*Y_per*MAX_rate*P_per;
}

/***************************************Duo Lun************************************************************************/

float RUDDER_DiPan:: Get_Ch0_Ch1_Vector_Speed(int16_t CH0, int16_t CH1)
{
  if (CH0 == 0)
    return abs(CH1);
  if (CH1 == 0)
    return abs(CH0);
  return sqrtf(CH0 * CH0 + CH1* CH1);
}

float RUDDER_DiPan:: Get_Max_Speed(int16_t CH0, int16_t CH1, int16_t CH2)
{
  float ch2_sqrt2 = 1.4142135623730950488016887242097f * abs(CH2);
  float ch01_max = Get_Ch0_Ch1_Vector_Speed(CH0,CH1);
  return ch2_sqrt2 > ch01_max ? ch2_sqrt2 :ch01_max;
}

float RUDDER_DiPan::absf(float d0)
{
  return d0 >= 0 ? d0 : -d0;
}

float RUDDER_DiPan:: Get_Max_float(float d0,float d1)
{
  d0 = absf(d0);
  return d0 > d1 ? d0 : d1;
}

int16_t RUDDER_DiPan:: Get_Max_int16(int16_t d0,int16_t d1)
{
  d0 = abs(d0);
  d1 = abs(d1);
  return d0 > d1 ? d0 : d1;
}

void RUDDER_DiPan:: Not_Xiaotuoluo_Jie_Suan(float CH0, float CH1, int16_t CH2)
{
  if(CH0 == 0 && CH1 == 0 && CH2 == 0)
	{
		g_wheel_3508[QZ].yaogan_speed = 0;
		g_wheel_3508[HZ].yaogan_speed = 0;
		g_wheel_3508[HY].yaogan_speed = 0;
		g_wheel_3508[QY].yaogan_speed = 0;
    return ;
	}

  float CH2_COS45 = COS_45 * CH2;
  QZ_xy.x = CH0 + CH2_COS45,QZ_xy.y = CH1 + CH2_COS45;
  HZ_xy.x = CH0 - CH2_COS45,HZ_xy.y = CH1 + CH2_COS45;
  HY_xy.x = CH0 - CH2_COS45,HY_xy.y = CH1 - CH2_COS45;
  QY_xy.x = CH0 + CH2_COS45,QY_xy.y = CH1 - CH2_COS45;
  g_wheel_3508[QZ].yaogan_speed = sqrtf(QZ_xy.x * QZ_xy.x + QZ_xy.y * QZ_xy.y);
  g_wheel_3508[HZ].yaogan_speed = sqrtf(HZ_xy.x * HZ_xy.x + HZ_xy.y * HZ_xy.y);
  g_wheel_3508[HY].yaogan_speed = sqrtf(HY_xy.x * HY_xy.x + HY_xy.y * HY_xy.y);
  g_wheel_3508[QY].yaogan_speed = sqrtf(QY_xy.x * QY_xy.x + QY_xy.y * QY_xy.y);
  float max_speed_nor = Get_Max_float(
                          Get_Max_float(
                            Get_Max_float(g_wheel_3508[QZ].yaogan_speed,g_wheel_3508[HZ].yaogan_speed)
                          ,g_wheel_3508[HY].yaogan_speed)
                        ,g_wheel_3508[QY].yaogan_speed);
  float max_speed_yaogan = Get_Max_int16(Get_Max_int16(Get_Max_int16(CH0,CH1),CH2),CH2);
  g_wheel_3508[QZ].yaogan_speed = g_wheel_3508[QZ].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
  g_wheel_3508[HZ].yaogan_speed = g_wheel_3508[HZ].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
  g_wheel_3508[HY].yaogan_speed = g_wheel_3508[HY].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
  g_wheel_3508[QY].yaogan_speed = g_wheel_3508[QY].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
  g_angle_6020[QZ] = atan2(QZ_xy.x,QZ_xy.y) * RAD2MANG+ ZERO[QZ];
  g_angle_6020[HZ] = atan2(HZ_xy.x,HZ_xy.y) * RAD2MANG+ ZERO[HZ];
  g_angle_6020[HY] = atan2(HY_xy.x,HY_xy.y) * RAD2MANG+ ZERO[HY];
  g_angle_6020[QY] = atan2(QY_xy.x,QY_xy.y) * RAD2MANG+ ZERO[QY];

}

void RUDDER_DiPan::Xiaotuoluo_jie_Suan(uint16_t Mang_yaw,int16_t CH0,int16_t CH1,int16_t CH2)
{
  int16_t mang_yaw_int16 = Mang_yaw << 3;
	mang_yaw_int16 /= 8;
	float yaw_angle =  -mang_yaw_int16 / RAD2MANG;
  float yaokong_angle = atan2(-CH0,-CH1);
  float max_speed_yaogan = Get_Max_int16(CH0,CH1);
  float vector_x = sinf(yaw_angle + yaokong_angle) * max_speed_yaogan,vector_y = cosf(yaw_angle + yaokong_angle) * max_speed_yaogan;
  Not_Xiaotuoluo_Jie_Suan(vector_x,vector_y,CH2);
}

/************************************************************************************************************/

/**************************************** P I D ***************************************************************************/
/*********************************************  D  L  *******************************************************/
bool PID_class::Get_6020mang_need_turn_direction_is(uint16_t goal, uint16_t now)
{
	return (abs(goal - now) > 2048);
}
void PID_class::PID_update_for_6020mang(int16_t goal, uint16_t now, struct wheel_dir_and_weight *wheel_dir_and_weight)
{
	int16_t deal_3508_g = goal << 3,deal_3508_n = now << 3;
	int now_error;
	goal &= 0x1fff;
	bool ret = Get_6020mang_need_turn_direction_is(goal, now);
	int16_t res;
	int16_t error;

	if (ret)
	{
		goal = (goal + (8192 / 2)) & 0x1fff;
	}

	res = (goal - now);
	if (res > 8192 / 2)
	{
		error = (res - 8192);
	}
	else if (res < -8192 / 2)
	{
		error = (res + 8192);
	}
	else
	{
		error = res;
	}

	now_error = error;

	wheel_dir_and_weight->speed = (float)((2048 - abs(now_error)) / (float)(2048)) * wheel_dir_and_weight->yaogan_speed;

	if(((deal_3508_g - (2048 << 3)) < deal_3508_n) && ((deal_3508_g + (2048 << 3)) > deal_3508_n))
	{
		wheel_dir_and_weight->dir = 0;
	}
	else{
		wheel_dir_and_weight->dir = 1;
		wheel_dir_and_weight->speed = -wheel_dir_and_weight->speed;
	}

	this->OUT_P = this->KP * now_error;
	this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);

	this->OUT_I += this->KI * (now_error);
	this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);

	this->OUT_D = this->KD * (now_error - this->LAST_Error);
	this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

	this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

	this->LAST_Error = now_error;
}

/*******************************************************************************************************/
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}

void PID_class::PID_new_update(float goal,float now)
{
	this->error = goal - now;

	if(fabs(this->error) < this->Deadzoom)//死区
	{
		this->error=0;
	}
	if(fabs(this->error) < this->Separate)//积分分离
	{
		this->Integral += this->error;
		this->Integral = LIMIT(this->Integral,-this->LIMIT_I,this->LIMIT_I);
	}
	else
	{
		this->OUT_I=0;
	}

	this->OUT_P = this->KP * this->error;

	this->OUT_I = this->KI*this->Integral;

	this->OUT_D = this->KD * (this->error - this->last_error);

	this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->OUT_PID = LIMIT(this->OUT_PID,-this->LIMIT_PID,this->LIMIT_PID);

	this->previous_error = this->last_error;
	this->last_error = this->error;
}

void PID_class::PID_update(float goal,float now)
{
	float now_error;

	now_error = goal - now;

	this->OUT_P = this->KP * now_error ;
	this->OUT_P=LIMIT(this->OUT_P,-this->LIMIT_P,this->LIMIT_P);

	this->OUT_I 	+=  this->KI*(now_error);
	this->OUT_I=LIMIT(this->OUT_I,-this->LIMIT_I,this->LIMIT_I);
	this->OUT_D = this->KD * (now_error - this->LAST_Error);
	this->OUT_D=LIMIT(this->OUT_D,-this->LIMIT_D,this->LIMIT_D);

	this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->OUT_PID = LIMIT(this->OUT_PID,-this->LIMIT_PID,this->LIMIT_PID);

	this->LAST_Error=now_error;
}

void PID_class::PID_Inc_update(float goal,float now)
{
	float now_error;

	now_error = goal - now;

	this->OUT_P = this->KP * (now_error-this->last_error) ;
	this->OUT_P = LIMIT(this->OUT_P,-this->LIMIT_P,this->LIMIT_P);

	this->OUT_I = this->KI * now_error;
	this->OUT_I = LIMIT(this->OUT_I,-this->LIMIT_I,this->LIMIT_I);

	this->OUT_D = this->KD * (now_error - 2*this->last_error + this->previous_error);
	this->OUT_D = LIMIT(this->OUT_D,-this->LIMIT_D,this->LIMIT_D);

	this->delta_OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->delta_OUT_PID = LIMIT(this->delta_OUT_PID,-this->LIMIT_PID,this->LIMIT_PID);

	this->OUT_PID+=this->delta_OUT_PID;

	this->previous_error = this->last_error;
	this->last_error = now_error;
}

void PID_Fuzzy_class::fuzzy(float goal,float now)
{
	this->error=goal - now;
	 float e = this->error/ this->stair;
	 float ec = (this->Out - this->Out_last) / this->stair;
	 short etemp,ectemp;
	 float eLefttemp,ecLefttemp;    //隶属� ?
	 float eRighttemp ,ecRighttemp;

	 short eLeftIndex,ecLeftIndex;  //标签
	 short eRightIndex,ecRightIndex;

	//模糊� ?
	 if(e>=PL)
		 etemp=PL;//超出范围
	 else if(e>=PM)
		 etemp=PM;
	 else if(e>=PS)
		 etemp=PS;
	 else if(e>=ZE)
		 etemp=ZE;
	 else if(e>=NS)
		 etemp=NS;
	 else if(e>=NM)
		 etemp=NM;
	 else if(e>=NL)
		 etemp=NL;
	 else
		 etemp=2*NL;

	 if( etemp == PL)
	{
	 //计算E隶属� ?
			eRighttemp= 0 ;    //右溢� ?
			eLefttemp= 1 ;

	 //计算标签
	 eLeftIndex = 6 ;
	 eRightIndex= 6 ;

	}else if( etemp == 2*NL )
	{
		//计算E隶属� ?
			eRighttemp = 1;    //左溢� ?
			eLefttemp = 0;

	 //计算标签
	 eLeftIndex = 0 ;
	 eRightIndex = 0 ;
	}	else
	{
		//计算E隶属� ?
			eRighttemp=(e-etemp);  //线�?�函数作为隶属函� ?
			eLefttemp=(1- eRighttemp);

	 //计算标签
	 eLeftIndex =(short) (etemp-NL);       //例如 etemp=2.5，NL=-3，那么得到的序列号为5  � ?0 1 2 3 4 5 6� ?
	 eRightIndex=(short) (eLeftIndex+1);
	}

	 if(ec>=PL)
		 ectemp=PL;
	 else if(ec>=PM)
		 ectemp=PM;
	 else if(ec>=PS)
		 ectemp=PS;
	 else if(ec>=ZE)
		 ectemp=ZE;
	 else if(ec>=NS)
		 ectemp=NS;
	 else if(ec>=NM)
		 ectemp=NM;
	 else if(ec>=NL)
		 ectemp=NL;
	 else
		 ectemp=2*NL;

   if( ectemp == PL )
	 {
    //计算EC隶属� ?
		 ecRighttemp= 0 ;      //右溢� ?
		 ecLefttemp= 1 ;

		 ecLeftIndex = 6 ;
	   ecRightIndex = 6 ;

	 } else if( ectemp == 2*NL)
	 {
    //计算EC隶属� ?
		 ecRighttemp= 1 ;
		 ecLefttemp= 0 ;

		 ecLeftIndex = 0 ;
	   ecRightIndex = 0 ;
	 }else
	 {
    //计算EC隶属� ?
		 ecRighttemp=(ec-ectemp);
		 ecLefttemp=(1- ecRighttemp);

		 ecLeftIndex =(short) (ectemp-NL);
	   ecRightIndex= (short)(eLeftIndex+1);
	 }
	this->dKp = this->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);

	this->dKi = this->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

	this->dKd = this->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
}

void PID_Fuzzy_class::FuzzyPID_update(float goal ,float now)
{
	  this->LastError = this->error;
		this->error = goal - now;

		fuzzy(goal,now);      //模糊调整  kp,ki,kd   形参1当前误差，形� ?2前后误差的差� ?

    float Kp = this->Kp0 + this->dKp , Ki = this->Ki0 + this->dKi , Kd = this->Kd0 + this->dKd ;   //PID均模� ?
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 + P->dKd ;           //仅PD均模� ?
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 ;                    //仅P均模� ?

    if(fabs(this->error) < this->I_L )
		{
			this->SumError += this->error/2;
			this->SumError = LIMIT(this->SumError,-this->IMax,this->IMax);
		}

		this->POut = Kp * this->error;
		this->IOut = Ki * this->SumError;
		this->DOut = Kd * (this->error - this->LastError);

		this->Out_last  = this->Out;
		this->Out = LIMIT(this->POut+this->IOut+this->DOut,this->OutMax,-this->OutMax);

}

/****************************************** D B U S **********************************************************/
void DBUS::watchdog_run(void)
{
	this->time_100ms++;
	if(this->time_100ms > 5)
	{
		this->set_zero();
		this->feed_watchdog();
		if(HAL_UART_GetState(this->huart) == HAL_UART_STATE_ERROR)
		{
			HAL_UART_Abort_IT(this->huart);
			this->Init();
		}
	}
}
void DBUS::Init(void)
{
	this->set_zero();
	while(this->huart->Init.BaudRate != 100000)
	{
		INFO("DEBUS Init Error\n");
		HAL_Delay(10);
	}
	__HAL_UART_ENABLE_IT(this->huart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(this->huart, dbus_rx_buffer,128);
}
void DBUS::DBUS_RxCplt_IRQHandler(void)
{
	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(this->huart,UART_FLAG_IDLE);
	if(tmp_flag != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(this->huart);
		temp = this->huart->Instance->SR;
		temp = this->huart->Instance->DR;
		HAL_UART_DMAStop(this->huart);
		if(check_and_deal() == HAL_OK)
		{
			this->data_deal();
			this->feed_watchdog();
		}
		HAL_UART_Receive_DMA(this->huart,dbus_rx_buffer,128);
	}
}
HAL_StatusTypeDef DBUS::check_and_deal(void)
{
		if(this->shubiao.z != 0 ||
			 dbus_rx_buffer[16]	!= 0 || dbus_rx_buffer[17] != 0 ||
			 this->shubiao.press_r >= 2 || this->shubiao.press_l >= 2)
		{
			this->set_zero();
			return HAL_ERROR;
		}
		else
			return HAL_OK;
}
void DBUS::set_zero(void)
{
	this->yaogan.ch0 			= 0;
	this->yaogan.ch1 			= 0;
	this->yaogan.ch2 			= 0;
	this->yaogan.ch3 			= 0;
	this->yaogan.s1 			= YK_SW_UP;
	this->yaogan.s2 			= YK_SW_UP;
	this->shubiao.press_l = 0;
	this->shubiao.press_r = 0;
	this->shubiao.x 			= 0;
	this->shubiao.y 			= 0;
	this->shubiao.z 			= 0;
	this->jianpan 				= 0;
}
void DBUS:: jianpan_deal(void)
{
	if(this->last_jianpan > this->jianpan)
	{
		this->index = (uint16_t)(log(this->last_jianpan-this->jianpan)/log(2));
		if(this->delaycount[this->index] < 8)
		{
			this->delaycount[this->index]++;
			this->jianpan |= 0x01<<this->index;
		}
		else
		{
			last_jianpan &= ~(0x01<<this->index);
			this->delaycount[this->index] = 0;
		}
	}
	else
	{
		this->last_jianpan = this->jianpan;
		memset(this->delaycount,0,16);
	}
}
void DBUS::data_deal(void)
{
	this->yaogan.ch0 = ((dbus_rx_buffer[0]| (dbus_rx_buffer[1] << 8)) & 0x07ff)-1024;        	//!< Channel 0
	this->yaogan.ch1 = (((dbus_rx_buffer[1] >> 3) | (dbus_rx_buffer[2] << 5)) & 0x07ff)-1024;	//!< Channel 1
	this->yaogan.ch2 = (((dbus_rx_buffer[2] >> 6) | (dbus_rx_buffer[3] << 2) |             		//!< Channel 2
										 (dbus_rx_buffer[4] << 10)) & 0x07ff)-1024;
	this->yaogan.ch3 = (((dbus_rx_buffer[4] >> 1) | (dbus_rx_buffer[5] << 7)) & 0x07ff)-1024; //!< Channel 3
	this->yaogan.s1  = ((dbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           		//!< Switch left
	this->yaogan.s2  = ((dbus_rx_buffer[5] >> 4)& 0x0003);                               		 	//!< Switch right
	this->shubiao.x = (int16_t)(dbus_rx_buffer[6] | (dbus_rx_buffer[7]<< 8));             		//!< Mouse X axis
	this->shubiao.y = (int16_t)(dbus_rx_buffer[8] | (dbus_rx_buffer[9]<< 8));             		//!< Mouse Y axis
	this->shubiao.z = (int16_t)(dbus_rx_buffer[10] | (dbus_rx_buffer[11] << 8));         			//!< Mouse Z axis
	this->shubiao.press_l = dbus_rx_buffer[12];                                        	 			//!< Mouse Left Is Press ?
	this->shubiao.press_r = dbus_rx_buffer[13];                                       	 			//!< Mouse Right Is Press ?
	this->jianpan = dbus_rx_buffer[14] | (dbus_rx_buffer[15]<< 8);

	this->jianpan_deal();
}

void DBUS::can_receive_data_deal(uint8_t num,uint8_t *buf)
{
	if(num==1)
	{
		this->yaogan.ch0 = ((buf[0]| (buf[1] << 8)) & 0x07ff)-1024;            //!< Channel 0
		this->yaogan.ch1 = (((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff)-1024;    //!< Channel 1
		this->yaogan.ch2 = (((buf[2] >> 6) | (buf[3] << 2) |     //!< Channel 2
							(buf[4] << 10)) & 0x07ff)-1024;
		this->yaogan.ch3 = (((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff)-1024;    //!< Channel 3
		this->yaogan.s1  = ((buf[5] >> 4)& 0x000C) >> 2;         //!< Switch left
		this->yaogan.s2  = ((buf[5] >> 4)& 0x0003);              //!< Switch right
		this->shubiao.x = (int16_t)(buf[6] | (buf[7]<< 8)); 	 //!< Mouse X axis
	}
	else if(num==2)
	{
		this->shubiao.y = (int16_t)(buf[0] | (buf[1]<< 8));      //!< Mouse Y axis
		this->shubiao.z = (int16_t)(buf[2] | (buf[3] << 8));     //!< Mouse Z axis
		this->shubiao.press_l = buf[4];                          //!< Mouse Left Is Press?
		this->shubiao.press_r = buf[5];                          //!< Mouse Right Is Press?
		this->jianpan = buf[6] | (buf[7]<< 8);                   //!< KeyBoard value

		this->jianpan_deal();
	}
  if(this->yaogan.ch0==-1024 || this->yaogan.ch1==-1024 ||
	   this->yaogan.ch2==-1024 || this->yaogan.ch3==-1024 ||
	   this->yaogan.s1 > 3	   || this->yaogan.s2 > 3			||
		 this->yaogan.s1 == 0	   || this->yaogan.s2 == 0 )
	{
		this->set_zero();
	}
}

uint8_t DBUS::Pressed_Check(uint16_t keyvalue)
{
	if(this->jianpan & keyvalue)  return 1;
	else  return 0;
}
/*******************************************************BMI088*******************************************************************/
#ifdef __SPI_H__

void BMI088::BMI088_writeByte(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t data)
{
	uint8_t t_buf[] = { (uint8_t)(subAddress & 0x7F)};
	this->BMI088_SPI_ON(GPIOx,GPIO_Pin);
	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);

//	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
//	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
//	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

	HAL_SPI_Transmit(hspi, &data, 1, 999);
	this->BMI088_SPI_OFF(GPIOx,GPIO_Pin);
}

void BMI088::BMI088_readBytes(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf)
{
	uint8_t t_buf[] = { (uint8_t)(subAddress | 0x80) };
	this->BMI088_SPI_ON(GPIOx,GPIO_Pin);
	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
	HAL_SPI_Receive(hspi, (uint8_t*)spi_rev_buf, len, 999);
	this->BMI088_SPI_OFF(GPIOx,GPIO_Pin);
}

void BMI088::BMI088_write_Acc(uint8_t subAddress, uint8_t data)
{
	BMI088_writeByte(this->CSB1_GPIOx,this->CSB1_GPIO_Pin,subAddress,data);
}
void BMI088::BMI088_write_Gyro(uint8_t subAddress, uint8_t data)
{
	BMI088_writeByte(this->CSB2_GPIOx,this->CSB2_GPIO_Pin,subAddress,data);
}
void BMI088::BMI088_read_Acc(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf)
{
	uint8_t t_buf[64],temp;
	this->BMI088_readBytes(this->CSB1_GPIOx,this->CSB1_GPIO_Pin,subAddress,len+1,t_buf);
	for(temp=0;temp<len;temp++)
		spi_rev_buf[temp] = t_buf[temp+1];
}

void BMI088::BMI088_read_Gyro(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf)
{
	this->BMI088_readBytes(this->CSB2_GPIOx,this->CSB2_GPIO_Pin,subAddress,len,spi_rev_buf);
}

void BMI088::set_zero(void)
{
	this->sensor_data={0};
}

BMI088_StatusTypeDef	BMI088::Init(void)
{
	uint8_t BMI088_rev_buf[32],loop_break=1,loop_count=0;

	while(strcmp("put the Update function in 400Hz interrupt",string_check_088) != 0)
	{
		INFO("BMI088 Init Error\n");
		HAL_Delay(10);
	}

	if(this->enable_acc)
	{
		BMI088_write_Acc(ACC_SOFTRESET,0xB6);   // 复位加�?�度� ?
		HAL_Delay(1);

		BMI088_read_Acc(ACC_CHIP_ID,1,BMI088_rev_buf);   // 初始化为SPI模式

		BMI088_read_Acc(ACC_CHIP_ID,1,BMI088_rev_buf);   // 确定ID无误
		if(BMI088_rev_buf[0]!=0x1E)  return BMI088_ACC_ID_ERROR;

		BMI088_write_Acc(ACC_PWR_CONF,0x00);		// 设置加�?�度计为正常模式
		HAL_Delay(5);
		BMI088_write_Acc(ACC_PWR_CTRL,0x04);		// 使能加�?�度计和温度� ?
		HAL_Delay(10);				// 加�?�度计启动延� ?

		BMI088_write_Acc(ACC_RANGE,0x01);				// 设置量程� ? ±6g
		BMI088_write_Acc(ACC_CONF,0x0A);				// 设置加�?�度计输出�?�率� ? 400Hz
		HAL_Delay(2);
	}

	BMI088_write_Gyro(GYRO_SOFTRESET,0xB6);  // 复位�?螺仪
	HAL_Delay(30);        // �?螺仪启动延时
	BMI088_read_Gyro(GYRO_CHIP_ID,1,BMI088_rev_buf);  // 确定ID无误
	if(BMI088_rev_buf[0]!=0x0F) return BMI088_GYRO_ID_ERROR;

	BMI088_write_Gyro(GYRO_BANDWIDTH,0x03); // 设置�?螺仪输出速率� ? 400Hz，过滤器带宽� ?47Hz
	BMI088_write_Gyro(GYRO_RANGE,GyroRange);

	BMI088_write_Gyro(GYRO_SELF_TEST,0x01);  // �?螺仪自检

	while(loop_break)
	{
		BMI088_read_Gyro(GYRO_SELF_TEST,1,BMI088_rev_buf);		// 自检
		if((BMI088_rev_buf[0]&0x02)==0x02)
		{
			loop_break=0;
		}
		loop_count++;
		if(loop_count>=40)			    // 自检超时，自�?失败
		{
				this->set_zero();
			  return BMI088_SELFTEXT_ERROR;
		}
		HAL_Delay(10);
	}

	HAL_Delay(1);
	this->BMI088_read_Gyro(GYRO_SELF_TEST,1,BMI088_rev_buf);
	if((BMI088_rev_buf[0]&0x04)!=0)  return BMI088_SELFTEXT_ERROR;

	if(this->AccRange==BMI088_ACC_RANGE_3) 		   this->AccRangsetting=0.0008974358974f;
	else if(this->AccRange==BMI088_ACC_RANGE_6)  this->AccRangsetting=0.00179443359375f;
	else if(this->AccRange==BMI088_ACC_RANGE_12) this->AccRangsetting=0.0035888671875f;
	else if(this->AccRange==BMI088_ACC_RANGE_24) this->AccRangsetting=0.007177734375f;

	if(this->GyroRange==BMI088_GYRO_RANGE_2000) this->GyroResolution=16.384f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_1000) this->GyroResolution=32.768f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_500) this->GyroResolution=65.536f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_250) this->GyroResolution=131.072f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_125) this->GyroResolution=262.144f;

	this->set_zero();
	HAL_TIM_Base_Start_IT(this->htim);
	this->low_pass_filter_init();
	return BMI088_OK;

}

void	BMI088::BMI088_update(void)
{
	uint8_t BMI088_rev_buf[32];
	int16_t temp=0;

	if(this->enable_acc)
	{
		BMI088_read_Acc(TEMP_MSB,2,BMI088_rev_buf);
		temp = (int16_t)((BMI088_rev_buf[1]&0xE0)|BMI088_rev_buf[0]<<8);
		temp >>= 5;
		this->sensor_data.temperature = temp*0.125+23;

		if(abs(this->sensor_data.temperature-this->last_temperature)>2 && this->filter_count_temperature<10)
		{
			this->sensor_data.temperature = this->last_temperature;
			this->filter_count_temperature++;
		}
		else
		{
			this->Acc_Temperature_Offset = (this->sensor_data.temperature-25)*0.2;
			this->Gyro_Temperature_Offset = (this->sensor_data.temperature-25)*0.015;
			this->last_temperature = this->sensor_data.temperature;
			this->filter_count_temperature=0;
		}
		BMI088_read_Acc(ACC_X_LSB,6,BMI088_rev_buf);   // 单位 mg
    this->sensor_data.acc.x = ((int16_t)(BMI088_rev_buf[1]<<8|BMI088_rev_buf[0])) * this->AccRangsetting;
		this->sensor_data.acc.y = ((int16_t)(BMI088_rev_buf[3]<<8|BMI088_rev_buf[2])) * this->AccRangsetting;
		this->sensor_data.acc.z = ((int16_t)(BMI088_rev_buf[5]<<8|BMI088_rev_buf[4])) * this->AccRangsetting;
	}

	BMI088_read_Gyro(GYRO_SELF_TEST,1,BMI088_rev_buf);
	if((BMI088_rev_buf[0]&0x10)==0x10)
	{
		BMI088_read_Gyro(RATE_X_LSB,6,BMI088_rev_buf);  // 单位 °/s
		this->sensor_data.gyro.origin.x = ((int16_t)(BMI088_rev_buf[1]<<8|BMI088_rev_buf[0]));
		this->sensor_data.gyro.origin.y = ((int16_t)(BMI088_rev_buf[3]<<8|BMI088_rev_buf[2]));
		this->sensor_data.gyro.origin.z = ((int16_t)(BMI088_rev_buf[5]<<8|BMI088_rev_buf[4]));

		this->sensor_data.gyro.LPF.x = this->low_pass_filter(this->sensor_data.gyro.origin.x);
		this->sensor_data.gyro.LPF.y = this->low_pass_filter(this->sensor_data.gyro.origin.y);
		this->sensor_data.gyro.LPF.z = this->low_pass_filter(this->sensor_data.gyro.origin.z);

		if(this->sensor_data.calibration)
		{
			this->sensor_data.gyro.calibration.x = this->sensor_data.gyro.LPF.x - this->sensor_data.gyro.offset.x + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.y = this->sensor_data.gyro.LPF.y - this->sensor_data.gyro.offset.y + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.z = this->sensor_data.gyro.LPF.z - this->sensor_data.gyro.offset.z + this->Acc_Temperature_Offset;
		}
		else
		{
			this->sensor_data.gyro.calibration.x = this->sensor_data.gyro.LPF.x + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.y = this->sensor_data.gyro.LPF.y + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.z = this->sensor_data.gyro.LPF.z + this->Acc_Temperature_Offset;
		}

		if(this->sensor_data.runningTimes < this->SELF_TEST_NUM)
	  {
			if (this->sensor_data.gyro.offset_max.x - this->sensor_data.gyro.offset_min.x > 75 ||
					this->sensor_data.gyro.offset_max.y - this->sensor_data.gyro.offset_min.y > 75 ||
					this->sensor_data.gyro.offset_max.z - this->sensor_data.gyro.offset_min.z > 75)
			{
				this->sensor_data.runningTimes = 0;
			}
			if(this->sensor_data.runningTimes == 0)
			{
				this->sensor_data.gyro.origin.x = 0;
				this->sensor_data.gyro.origin.y = 0;
				this->sensor_data.gyro.origin.z = 0;

				 this->sensor_data.gyro.dynamicSum.x = 0;
				 this->sensor_data.gyro.dynamicSum.y = 0;
				 this->sensor_data.gyro.dynamicSum.z = 0;

				 this->sensor_data.gyro.offset_max.x = -32768;
				 this->sensor_data.gyro.offset_max.y = -32768;
				 this->sensor_data.gyro.offset_max.z = -32768;
				 this->sensor_data.gyro.offset_min.x = 32768;
				 this->sensor_data.gyro.offset_min.y = 32768;
				 this->sensor_data.gyro.offset_min.z = 32768;
			}

			if (this->sensor_data.gyro.origin.x > this->sensor_data.gyro.offset_max.x)
				this->sensor_data.gyro.offset_max.x = this->sensor_data.gyro.origin.x;
			if (this->sensor_data.gyro.origin.y > this->sensor_data.gyro.offset_max.y)
				this->sensor_data.gyro.offset_max.y = this->sensor_data.gyro.origin.y;
			if (this->sensor_data.gyro.origin.z > this->sensor_data.gyro.offset_max.z)
				this->sensor_data.gyro.offset_max.z = this->sensor_data.gyro.origin.z;

			if (this->sensor_data.gyro.origin.x < this->sensor_data.gyro.offset_min.x)
				this->sensor_data.gyro.offset_min.x = this->sensor_data.gyro.origin.x;
			if (this->sensor_data.gyro.origin.y < this->sensor_data.gyro.offset_min.y)
				this->sensor_data.gyro.offset_min.y = this->sensor_data.gyro.origin.y;
			if (this->sensor_data.gyro.origin.z < this->sensor_data.gyro.offset_min.z)
				this->sensor_data.gyro.offset_min.z = this->sensor_data.gyro.origin.z;

			this->sensor_data.gyro.dynamicSum.x += this->sensor_data.gyro.origin.x;
			this->sensor_data.gyro.dynamicSum.y += this->sensor_data.gyro.origin.y;
			this->sensor_data.gyro.dynamicSum.z += this->sensor_data.gyro.origin.z;

			this->sensor_data.runningTimes++;
   	}
		else
		{
			this->sensor_data.calibration=1;
	  }

		this->sensor_data.gyro.offset.x = (float)(this->sensor_data.gyro.dynamicSum.x) / this->sensor_data.runningTimes;
		this->sensor_data.gyro.offset.y = (float)(this->sensor_data.gyro.dynamicSum.y) / this->sensor_data.runningTimes;
		this->sensor_data.gyro.offset.z = (float)(this->sensor_data.gyro.dynamicSum.z) / this->sensor_data.runningTimes;

		this->sensor_data.gyro.dps.x = this->sensor_data.gyro.calibration.x / this->GyroResolution;
		this->sensor_data.gyro.dps.y = this->sensor_data.gyro.calibration.y / this->GyroResolution;
		this->sensor_data.gyro.dps.z = this->sensor_data.gyro.calibration.z / this->GyroResolution;

		if(abs(this->sensor_data.gyro.dps.x-this->last_gyro_x)>7 && filter_count_x<2) this->filter_count_x++;
		else
		{
			if(abs(this->sensor_data.gyro.dps.x) < abs(this->dead_zoom)) this->sensor_data.gyro.x_nonoise = 0;
			else this->sensor_data.gyro.x_nonoise = this->sensor_data.gyro.dps.x;
			this->last_gyro_x = this->sensor_data.gyro.dps.x;
			this->filter_count_x=0;
		}

		if(abs(this->sensor_data.gyro.dps.y-this->last_gyro_y)>7 && filter_count_y<2) this->filter_count_y++;
		else
		{
			if(abs(this->sensor_data.gyro.dps.y) < abs(this->dead_zoom)) this->sensor_data.gyro.y_nonoise = 0;
			else this->sensor_data.gyro.y_nonoise = this->sensor_data.gyro.dps.y;
			this->last_gyro_y = this->sensor_data.gyro.dps.y;
			this->filter_count_y=0;
		}

		if(abs(this->sensor_data.gyro.dps.z-this->last_gyro_z)>7 && filter_count_z<2) this->filter_count_z++;
		else
		{
			if(abs(this->sensor_data.gyro.dps.z) < abs(this->dead_zoom)) this->sensor_data.gyro.z_nonoise = 0;
			else this->sensor_data.gyro.z_nonoise = this->sensor_data.gyro.dps.z;
			this->last_gyro_z = this->sensor_data.gyro.dps.z;
			this->filter_count_z=0;
		}

		this->sensor_data.mang.x += this->sensor_data.gyro.x_nonoise*0.0025f;
		this->sensor_data.mang.y += this->sensor_data.gyro.y_nonoise*0.0025f;
		this->sensor_data.mang.z += this->sensor_data.gyro.z_nonoise*0.0025f;
	}
}

void  BMI088::selftext_error_reset(void)
{
	switch(this->selftext_reset_step)
	{
		case 0:
			BMI088_write_Gyro(GYRO_SOFTRESET,0xB6);
			this->timer_1ms=0;
			this->selftext_reset_step=1;
		break;

		case 1:
			this->timer_1ms++;
			if(timer_1ms>30) this->selftext_reset_step=2;
		break;

		case 2:
			BMI088_write_Gyro(GYRO_BANDWIDTH,0x03);
			BMI088_write_Gyro(GYRO_RANGE,0x01);
			this->selftext_reset_step=3;
		break;

		case 3:
			BMI088_write_Gyro(GYRO_SELF_TEST,0x01);
			this->timer_1ms=0;
			this->selftext_reset_step=4;
		break;

		case 4:
			this->timer_1ms++;
			if(timer_1ms>10)
			{
				this->selftext_error_flag = 0;
				this->selftext_reset_step=0;
			}
		break;

		default: this->selftext_reset_step=0; break;
	}
}

/************************ 滤波器初始化 alpha *****************************/
void BMI088::low_pass_filter_init(void)
{
  float b = 2.0 * LPF_factor.pi * LPF_factor.CUTOFF_FREQ  * LPF_factor.SAMPLE_RATE;
  LPF_factor.alpha = b / (b + 1);
}

float BMI088::low_pass_filter(float value)
{
  static float out_last = 0; //上一次滤波�?
  float out;

  /***************** 如果第一次进入，则给 out_last 赋�? ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** �?阶滤� ? *********************************/
  out = out_last + LPF_factor.alpha * (value - out_last);
  out_last = out;

  return out;
}
#endif

