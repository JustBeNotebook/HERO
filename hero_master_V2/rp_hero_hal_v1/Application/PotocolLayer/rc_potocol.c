/**
 * @file        rc_potocol.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       DT7&DR16 Rc Potocol.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rc_potocol.h"
#include "rp_math.h"
#include "rc_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

moving_Average_Filter KEY_W_MF, KEY_A_MF, KEY_S_MF, KEY_D_MF;
moving_Average_Filter MOUSE_X_MF, MOUSE_Y_MF;

extKalman_t KEY_W_KF, KEY_S_KF,KEY_A_KF, KEY_D_KF;
extKalman_t MOUSE_X_KF, MOUSE_Y_KF;

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void rc_sensor_init(rc_sensor_t *rc_sen)
{
	// 初始化为离线状态
	rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
	rc_sen->work_state = DEV_OFFLINE;
	
	average_init(&KEY_W_MF, 10);
	average_init(&KEY_A_MF, 10);
	average_init(&KEY_S_MF, 10);
	average_init(&KEY_D_MF, 10);
	average_init(&MOUSE_X_MF, 10);
	average_init(&MOUSE_Y_MF, 10);
	
	KalmanCreate(&KEY_W_KF, 1, 50);
	KalmanCreate(&KEY_A_KF, 1, 50);
	KalmanCreate(&KEY_S_KF, 1, 50);
	KalmanCreate(&KEY_D_KF, 1, 50);
	KalmanCreate(&MOUSE_X_KF, 1, 1);
	KalmanCreate(&MOUSE_Y_KF, 1, 1);
	
	if(rc_sen->id == DEV_ID_RC)
		rc_sen->errno = NONE_ERR;
	else
		rc_sen->errno = DEV_ID_ERR;
}

int16_t Ramp_Key_move(int16_t value, uint8_t state)
{
	if(state)
	{
		if(value<3000)
		{
			value = RampInt(8000, value, 100);
		}
		else
		{
			value = RampInt(8000, value, 200);
		}
	}
	else 
	{
		value = RampInt(0, value, 2000);
	}
	
	return value;
}

/**
 *	@brief	遥控器数据解析协议
 */
short Key_W = 0, Key_A = 0, Key_S = 0, Key_D = 0;
void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf)
{
	rc_sensor_info_t *rc_info = rc_sen->info;

	
	rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF;
	rc_info->ch0 -= 1024;
	rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF;
	rc_info->ch1 -= 1024;
	rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
	rc_info->ch2 -= 1024;
	rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF;
	rc_info->ch3 -= 1024;

	rc_info->s1 = ((rxBuf[5] >> 4) & 0x000C) >> 2;
	rc_info->s2 = (rxBuf[5] >> 4) & 0x0003;	
	
	rc_info->mouse_vx = rxBuf[6]  | (rxBuf[7] << 8);
	rc_info->mouse_vy = rxBuf[8]  | (rxBuf[9] << 8);
	rc_info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8);
	rc_info->mouse_btn_l = rxBuf[12];
	rc_info->mouse_btn_r = rxBuf[13];
	
	rc_info->kb.key_v = rxBuf[14] | (rxBuf[15] << 8);	
	
	rc_info->thumbwheel = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	rc_info->thumbwheel -= 1024;
	
	rc_info->offline_cnt = 0;
	
	
//	Key_W = IF_KEY_PRESSED_W?RampInt(8000, Key_W, 200):RampInt(0, Key_W, 2000);
//	Key_A = IF_KEY_PRESSED_A?RampInt(8000, Key_A, 200):RampInt(0, Key_A, 2000);
//	Key_S = IF_KEY_PRESSED_S?RampInt(8000, Key_S, 200):RampInt(0, Key_S, 2000);
//	Key_D = IF_KEY_PRESSED_D?RampInt(8000, Key_D, 200):RampInt(0, Key_D, 2000);
	
	Key_W = Ramp_Key_move(Key_W, IF_KEY_PRESSED_W);
	Key_A = Ramp_Key_move(Key_A, IF_KEY_PRESSED_A);
	Key_S = Ramp_Key_move(Key_S, IF_KEY_PRESSED_S);
	Key_D = Ramp_Key_move(Key_D, IF_KEY_PRESSED_D);

	average_add(&MOUSE_X_MF, MOUSE_X_MOVE_SPEED);
	average_add(&MOUSE_Y_MF, MOUSE_Y_MOVE_SPEED);
	
	rc_move_info.key_w = KalmanFilter(&KEY_W_KF, (float)Key_W);
	rc_move_info.key_a = KalmanFilter(&KEY_A_KF, (float)Key_A);
	rc_move_info.key_s = KalmanFilter(&KEY_S_KF, (float)Key_S);
	rc_move_info.key_d = KalmanFilter(&KEY_D_KF, (float)Key_D);

	rc_move_info.mouse_vx = KalmanFilter(&MOUSE_X_KF, MOUSE_X_MF.aver_num);
	rc_move_info.mouse_vy = KalmanFilter(&MOUSE_Y_KF, MOUSE_Y_MF.aver_num);
	
	if(IF_KEY_PRESSED_R)
	{
		rc_move_info.mouse_vx *= 0.5;
		rc_move_info.mouse_vy *= 0.5;
	}
	
}

/**
 *	@brief	在串口2中解析遥控数据协议
 */
void USART2_rxDataHandler(uint8_t *rxBuf)
{
	// 更新遥控数据
	rc_sensor.update(&rc_sensor, rxBuf);
	rc_sensor.check(&rc_sensor);
}
