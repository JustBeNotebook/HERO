#ifndef __CAN_POTOCOL_H
#define __CAN_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/*
CANX_1FF 5 6 7 8
CANX_200 1 2 3 4
CANX_2FF 7~12
*/


/* Exported macro ------------------------------------------------------------*/
/*CAN1*/
#define CHASSIS_CAN_ID_LF	0x201U
#define CHASSIS_CAN_ID_RF	0x202U
#define CHASSIS_CAN_ID_LB	0x203U
#define CHASSIS_CAN_ID_RB	0x204U

#define LAUNCHER_CAN_ID_DIAL	0x207U
#define GIMBAL_CAN_ID_YAW	0x208U
/*CAN2*/

#define LAUNCHER_CAN_ID_MGZ	0x3ffU
#define LAUNCHER_CAN_ID_SAFE	0x202U
#define LAUNCHER_CAN_ID_FRICT_R	0x205U
#define LAUNCHER_CAN_ID_FRICT_L	0x206U
#define GIMBAL_CAN_ID_M_PITCH	0x207U
#define GIMBAL_CAN_ID_S_PITCH	0x208U

/*JUDGE*/
#define	JUDGE_CAN_ID_RFID 0X1a8U
#define	JUDGE_CAN_ID_BUFF 0X1a9U
#define	JUDGE_CAN_ID_HURT 0X1aaU
#define	JUDGE_CAN_ID_SHOOT 0X1abU
#define	JUDGE_CAN_ID_STATE2 0X1acU
#define	JUDGE_CAN_ID_STATE 0X1adU
#define	JUDGE_CAN_ID_OUT_HEAT2 0X1aeU
#define	JUDGE_CAN_ID_OUT_HEAT1 0X1afU


#define CAN_Id_Standard             ((uint32_t)0x00000000)  /*!< Standard Id */
#define CAN_Id_Extended             ((uint32_t)0x00000004)  /*!< Extended Id */


/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ       ((uint32_t)0x00000001) /* Transmit mailbox request */
#define CAN_TxStatus_NoMailBox      ((uint8_t)0x04) /*!< CAN cell did not provide 
                                                         an empty mailbox */

/* Exported types ------------------------------------------------------------*/

typedef struct
{
  uint32_t StdId;  /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

  uint32_t ExtId;  /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

  uint8_t IDE;     /*!< Specifies the type of identifier for the message that 
                        will be transmitted. This parameter can be a value 
                        of @ref CAN_identifier_type */

  uint8_t RTR;     /*!< Specifies the type of frame for the message that will 
                        be transmitted. This parameter can be a value of 
                        @ref CAN_remote_transmission_request */

  uint8_t DLC;     /*!< Specifies the length of the frame that will be 
                        transmitted. This parameter can be a value between 
                        0 to 8 */

  uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0 
                        to 0xFF. */
} CanTxMsg;

/* Exported functions --------------------------------------------------------*/
void	CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void 	CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff);
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN1_Send_Msg2(int16_t *msg, uint8_t len, uint16_t stdid);
uint8_t CAN2_Send_Msg2(int16_t *msg, uint8_t len, uint16_t stdid);

#endif
