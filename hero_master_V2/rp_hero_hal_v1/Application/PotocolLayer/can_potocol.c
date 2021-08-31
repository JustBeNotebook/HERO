/**
 * @file        can_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       CAN Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "can_potocol.h"
#include "judge.h"
#include "drv_can.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "launcher_motor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	从CAN报文中读取电机的位置反馈
 */
static uint16_t CAN_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle;
	angle = ((uint16_t)rxData[0] << 8| rxData[1]);
	return angle;
}

/**
 *	@brief	从CAN报文中读取电机的转子转速反馈
 */
static int16_t CAN_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed;
	speed = ((uint16_t)rxData[2] << 8| rxData[3]);
	return speed;
}

/**
 *	@brief	从CAN报文中读取电机的实际转矩电流反馈
 */
static int16_t CAN_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current;
	current = ((int16_t)rxData[4] << 8 | rxData[5]);
	return current;
}

///**
// *	@brief	从CAN报文中读取电机的实际输出转矩
// */
//static int16_t CAN_GetMotorTorque(uint8_t *rxData)
//{
//	int16_t torque;
//	torque = ((int16_t)rxData[4] << 8 | rxData[5]);
//	return torque;
//}

/**
 *	@brief	RM3508 CAN标识符
 */
static uint32_t RM3508_GetStdId(drv_can_t *drv)
{
	if((drv->can_id - 0x201U) < 4)
		return 0x200;
	else if((drv->can_id - 0x201U) < 8)
		return 0x1FF;
	else
		return 0x2FF;
}

/**
 *	@brief	RM3508 CAN数据下标
 */
static uint8_t RM3508_GetDrvId(drv_can_t *drv)
{
	return (drv->can_id - 0x201U)%4;
}

///**
// *	@brief	GM6020 CAN标识符
// */
//static uint32_t GM6020_GetStdId(drv_can_t *drv)
//{
//	if((drv->can_id - 0x205U) < 4)
//		return 0x1FF;
//	else
//		return 0x2FF;
//}

///**
// *	@brief	GM6020 CAN数据下标
// */
//static uint8_t GM6020_GetDrvId(drv_can_t *drv)
//{
//	return (drv->can_id - 0x205U)%4;
//}

///**
// *	@brief	RM2006 CAN标识符
// */
//static uint32_t RM2006_GetStdId(drv_can_t *drv)
//{
//	if((drv->can_id - 0x201U) < 4)
//		return 0x200;
//	else
//		return 0x1FF;
//}

///**
// *	@brief	RM2006 CAN数据下标
// */
//static uint8_t RM2006_GetDrvId(drv_can_t *drv)
//{
//	return (drv->can_id - 0x201U)%4;
//}

/* Exported functions --------------------------------------------------------*/
void chassis_motor_update(chassis_motor_t *motor, uint8_t *rxBuf)
{
	chassis_motor_info_t *motor_info = motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info->offline_cnt = 0;
}

void chassis_motor_init(chassis_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;
	
	motor->errno = NONE_ERR;
	if(motor->id == DEV_ID_CHASSIS_LF) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->id == DEV_ID_CHASSIS_RF) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->id == DEV_ID_CHASSIS_LB) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else if(motor->id == DEV_ID_CHASSIS_RB) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);		
	}
	else {
		motor->errno = DEV_ID_ERR;
	}
}


void gimbal_motor_update(gimbal_motor_t *motor, uint8_t *rxBuf)
{
	gimbal_motor_info_t *motor_info = motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info->offline_cnt = 0;
}

void gimbal_motor_init(gimbal_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;
	
	motor->errno = NONE_ERR;
	if(motor->id == DEV_ID_GIMBAL_YAW) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->id == DEV_ID_GIMBAL_M_PITCH) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->id == DEV_ID_GIMBAL_S_PITCH) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else {
		motor->errno = DEV_ID_ERR;
	}
}


void launcher_motor_update(launcher_motor_t *motor, uint8_t *rxBuf)
{
	launcher_motor_info_t *motor_info = motor->info;
	
	motor_info->angle = CAN_GetMotorAngle(rxBuf);
	motor_info->speed = CAN_GetMotorSpeed(rxBuf);
	motor_info->current = CAN_GetMotorCurrent(rxBuf);
	
	motor_info->offline_cnt = 0;
}

void launcher_motor_init(launcher_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;
	
	motor->errno = NONE_ERR;
	if(motor->id == DEV_ID_LAUNCHER_MGZ) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->id == DEV_ID_LAUNCHER_DIAL) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);
	}
	else if(motor->id == DEV_ID_LAUNCHER_SAFE) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else if(motor->id == DEV_ID_LAUNCHER_FRICT_L) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else if(motor->id == DEV_ID_LAUNCHER_FRICT_R) {
		drv_can->drv_id = RM3508_GetDrvId(drv_can);
		drv_can->std_id = RM3508_GetStdId(drv_can);	
	}
	else {
		motor->errno = DEV_ID_ERR;
	}
}



/**
 *	@brief	CAN 发送单独数据
 */
void CAN_SendSingleData(drv_can_t *drv, int16_t txData)
{
	int16_t txArr[4] = {0, 0, 0, 0};
	
	txArr[drv->drv_id] = txData;
	if(drv->type == DRV_CAN1)
		CAN1_SendData(drv->std_id, txArr);
	else if(drv->type == DRV_CAN2)
		CAN2_SendData(drv->std_id, txArr);
}

/**
 *	@brief	CAN 发送数据缓冲
 */
void CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff)
{
	if(drv_type == DRV_CAN1)
		CAN1_SendData(std_id, txBuff);
	else if(drv_type == DRV_CAN2)
		CAN2_SendData(std_id, txBuff);
}


uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
{
  uint8_t transmit_mailbox = 0;
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
  assert_param(IS_CAN_RTR(TxMessage->RTR));
  assert_param(IS_CAN_DLC(TxMessage->DLC));

  /* Select one empty transmit mailbox */
  if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
  {
    transmit_mailbox = 0;
  }
  else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
  {
    transmit_mailbox = 1;
  }
  else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
  {
    transmit_mailbox = 2;
  }
  else
  {
    transmit_mailbox = CAN_TxStatus_NoMailBox;
  }

  if (transmit_mailbox != CAN_TxStatus_NoMailBox)
  {
    /* Set up the Id */
    CANx->sTxMailBox[transmit_mailbox].TIR &= TMIDxR_TXRQ;
    if (TxMessage->IDE == CAN_Id_Standard)
    {
      assert_param(IS_CAN_STDID(TxMessage->StdId)); //发送ID = 设定ID 
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->StdId << 21) | \
                                                  TxMessage->RTR);
    }
    else
    {
      assert_param(IS_CAN_EXTID(TxMessage->ExtId));
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->ExtId << 3) | \
                                                  TxMessage->IDE | \
                                                  TxMessage->RTR);
    }
    
    /* Set up the DLC */
    TxMessage->DLC &= (uint8_t)0x0000000F;
    CANx->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CANx->sTxMailBox[transmit_mailbox].TDTR |= TxMessage->DLC;

    /* Set up the data field */
    CANx->sTxMailBox[transmit_mailbox].TDLR = (((uint32_t)TxMessage->Data[3] << 24) | 
                                             ((uint32_t)TxMessage->Data[2] << 16) |
                                             ((uint32_t)TxMessage->Data[1] << 8) | 
                                             ((uint32_t)TxMessage->Data[0]));
    CANx->sTxMailBox[transmit_mailbox].TDHR = (((uint32_t)TxMessage->Data[7] << 24) | 
                                             ((uint32_t)TxMessage->Data[6] << 16) |
                                             ((uint32_t)TxMessage->Data[5] << 8) |
                                             ((uint32_t)TxMessage->Data[4]));
    /* Request transmission */
    CANx->sTxMailBox[transmit_mailbox].TIR |= TMIDxR_TXRQ;
  }
  return transmit_mailbox;
}

/**
 *	@brief	CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	/* 左前轮 */
	if(canId == CHASSIS_CAN_ID_LF)
	{
		// 更新底盘电机数据
		chassis_motor[CHAS_LF].update(&chassis_motor[CHAS_LF], rxBuf);
		chassis_motor[CHAS_LF].check(&chassis_motor[CHAS_LF]);
	}
	/* 右前轮 */
	else if(canId == CHASSIS_CAN_ID_RF)
	{
		// 更新底盘电机数据
		chassis_motor[CHAS_RF].update(&chassis_motor[CHAS_RF], rxBuf);
		chassis_motor[CHAS_RF].check(&chassis_motor[CHAS_RF]);
	}
	/* 左后轮 */
	else if(canId == CHASSIS_CAN_ID_LB)
	{
		// 更新底盘电机数据
		chassis_motor[CHAS_LB].update(&chassis_motor[CHAS_LB], rxBuf);
		chassis_motor[CHAS_LB].check(&chassis_motor[CHAS_LB]);
	}
	/* 右后轮 */
	else if(canId == CHASSIS_CAN_ID_RB)
	{
		// 更新底盘电机数据
		chassis_motor[CHAS_RB].update(&chassis_motor[CHAS_RB], rxBuf);
		chassis_motor[CHAS_RB].check(&chassis_motor[CHAS_RB]);
	}
	else if(canId == JUDGE_CAN_ID_STATE)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_STATE, rxBuf);
		Judge_Hero.state_flag = 1;
	}
	else if(canId == JUDGE_CAN_ID_OUT_HEAT2)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_OUT_HEAT2, rxBuf);
	}
	else if(canId == JUDGE_CAN_ID_OUT_HEAT1)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_OUT_HEAT1, rxBuf);
	}
	else if(canId == JUDGE_CAN_ID_STATE2)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_STATE2, rxBuf);
	}
	else if(canId == JUDGE_CAN_ID_SHOOT)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_SHOOT, rxBuf);
	}
	else if(canId == JUDGE_CAN_ID_HURT)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_HURT, rxBuf);
	}
	else if(canId == JUDGE_CAN_ID_BUFF)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_BUFF, rxBuf);
	}
	else if(canId == JUDGE_CAN_ID_RFID)
	{
		CAN_Judge_Analyze(JUDGE_CAN_ID_RFID, rxBuf);
	}
	else if(canId == GIMBAL_CAN_ID_YAW)
	{
		gimbal_motor[GIMBAL_YAW].update(&gimbal_motor[GIMBAL_YAW], rxBuf);
		gimbal_motor[GIMBAL_YAW].check(&gimbal_motor[GIMBAL_YAW]);
		if(gimbal_motor[GIMBAL_YAW].info->angle_sum >GIMBAL_YAW_CIRCULAR_STEP/2)
		{
			gimbal_motor[GIMBAL_YAW].info->angle_sum-=GIMBAL_YAW_CIRCULAR_STEP;
		}
		else if(gimbal_motor[GIMBAL_YAW].info->angle_sum < -GIMBAL_YAW_CIRCULAR_STEP/2)
		{
			gimbal_motor[GIMBAL_YAW].info->angle_sum +=GIMBAL_YAW_CIRCULAR_STEP;
		}
		gimbal_yaw = gimbal_motor[GIMBAL_YAW].info[GIMBAL_YAW];
	}
	else if(canId == LAUNCHER_CAN_ID_DIAL)
	{
		launcher_motor[LAUNCHER_DIAL].update(&launcher_motor[LAUNCHER_DIAL], rxBuf);
		launcher_motor[LAUNCHER_DIAL].check(&launcher_motor[LAUNCHER_DIAL]);
	}
	
}

/**
 *	@brief	CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(canId == LAUNCHER_CAN_ID_SAFE)
	{
		launcher_motor[LAUNCHER_SAFE].update(&launcher_motor[LAUNCHER_SAFE], rxBuf);
		launcher_motor[LAUNCHER_SAFE].check(&launcher_motor[LAUNCHER_SAFE]);
	}
	else if(canId == LAUNCHER_CAN_ID_FRICT_L)
	{
		launcher_motor[LAUNCHER_FRICT_L].update(&launcher_motor[LAUNCHER_FRICT_L], rxBuf);
		launcher_motor[LAUNCHER_FRICT_L].check(&launcher_motor[LAUNCHER_FRICT_L]);
	}
	else if(canId == LAUNCHER_CAN_ID_FRICT_R)
	{
		launcher_motor[LAUNCHER_FRICT_R].update(&launcher_motor[LAUNCHER_FRICT_R], rxBuf);
		launcher_motor[LAUNCHER_FRICT_R].check(&launcher_motor[LAUNCHER_FRICT_R]);
	}
	else if(canId == GIMBAL_CAN_ID_M_PITCH)
	{
		gimbal_motor[GIMBAL_M_PITCH].update(&gimbal_motor[GIMBAL_M_PITCH], rxBuf);
		gimbal_motor[GIMBAL_M_PITCH].check(&gimbal_motor[GIMBAL_M_PITCH]);
		gimbal_m_pitch = gimbal_motor[GIMBAL_M_PITCH].info[GIMBAL_M_PITCH];
	}
	else if(canId == GIMBAL_CAN_ID_S_PITCH)
	{
		gimbal_motor[GIMBAL_S_PITCH].update(&gimbal_motor[GIMBAL_S_PITCH], rxBuf);
		gimbal_motor[GIMBAL_S_PITCH].check(&gimbal_motor[GIMBAL_S_PITCH]);
		gimbal_s_pitch = gimbal_motor[GIMBAL_S_PITCH].info[GIMBAL_S_PITCH];
	}
	
	
}

