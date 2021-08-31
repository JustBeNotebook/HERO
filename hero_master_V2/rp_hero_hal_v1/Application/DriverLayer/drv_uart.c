/**
 * @file        drv_uart.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-August-2020
 * @brief       UART Driver Package(Based on HAL).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

#include "string.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* Private macro -------------------------------------------------------------*/
#define USART2_RX_DATA_FRAME_LEN	(18u)	// 串口2数据帧长度
#define USART2_RX_BUF_LEN			(USART2_RX_DATA_FRAME_LEN + 6u)	// 串口2接收缓冲区长度

#define USART1_RX_BUF_LEN			200
#define USART4_RX_BUF_LEN			100
#define USART5_RX_BUF_LEN			200

/* Private function prototypes -----------------------------------------------*/
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART2_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART4_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf);

static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t usart1_dma_rxbuf[USART1_RX_BUF_LEN];
uint8_t usart2_dma_rxbuf[2][USART2_RX_BUF_LEN];
uint8_t usart4_dma_rxbuf[USART4_RX_BUF_LEN];
uint8_t usart5_dma_rxbuf[USART5_RX_BUF_LEN];

/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	// 将当前目标内存设置为Memory1
	hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
	USART2_rxDataHandler(usart2_dma_rxbuf[0]);
}

static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	// 将当前目标内存设置为Memory0
	hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
	USART2_rxDataHandler(usart2_dma_rxbuf[1]);
}

/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @usage   call in DRV_UART_IRQHandler() function
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);	
	/* handle received data in idle interrupt */
	if(huart == &huart1)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* handle dbus data dbus_buf from DMA */
		USART1_rxDataHandler(usart1_dma_rxbuf);
		memset(usart1_dma_rxbuf, 0, USART1_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
	else if(huart == &huart2)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		//uint32_t status = taskENTER_CRITICAL_FROM_ISR();
		if ((USART2_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART2_RX_DATA_FRAME_LEN)
		{
			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
			else
				huart->hdmarx->XferCpltCallback(huart->hdmarx);
		}
		//taskEXIT_CRITICAL_FROM_ISR(status);

		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART2_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}
	else if (huart == &huart4)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* handle dbus data dbus_buf from DMA */
		USART4_rxDataHandler(usart4_dma_rxbuf);
		memset(usart4_dma_rxbuf, 0, USART4_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);		
	}
	else if (huart == &huart5)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* handle dbus data dbus_buf from DMA */
		USART5_rxDataHandler(usart5_dma_rxbuf);
		memset(usart5_dma_rxbuf, 0, USART5_RX_BUF_LEN);
		/* restart dma transmission */	  
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
  
}

/**
 *	@brief	start double buffer dma transfer with no interrupt
 */
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Memory-to-memory transfer not supported in double buffering mode */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/* Set the UART DMA transfer complete callback */
	/* Current memory buffer used is Memory 1 callback */
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/* Current memory buffer used is Memory 0 callback */
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/* Check callback functions */
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
	hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
	return HAL_ERROR;
	}
	
	/* Process locked */
	__HAL_LOCK(hdma);
	
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Enable the Double buffer mode */
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

		/* Configure DMA Stream destination address */
		hdma->Instance->M1AR = SecondMemAddress;		

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;		
		
		/* Peripheral to Memory */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Memory to Peripheral */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/* Clear TC flags */
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* Enable TC interrupts*/
//		hdma->Instance->CR  |= DMA_IT_TC;
		
		/* Enable the peripheral */
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);	  

		/* Return error status */
		status = HAL_BUSY;		
	}
	/* Process unlocked */
	__HAL_UNLOCK(hdma);

	return status; 	
}

static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Configure the source, destination address and the data length */
		/* Clear DBM bit */
		hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;

		/* Memory to Peripheral */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Peripheral to Memory */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}

		/* Enable the Peripheral */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);

		/* Return error status */
		status = HAL_BUSY;
	} 
	return status; 	
}
/* Exported functions --------------------------------------------------------*/
/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    // 判断是否为空闲中断
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**
 *	@brief	USART1 Initialization
 */
void USART1_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);	
	
	DMA_Start(huart1.hdmarx, \
			  (uint32_t)&huart1.Instance->DR, \
			  (uint32_t)usart1_dma_rxbuf, \
			  USART1_RX_BUF_LEN);
}

/**
 *	@brief	USART2 Initialization
 */
void USART2_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);	
	
	DMAEx_MultiBufferStart_NoIT(huart2.hdmarx, \
							    (uint32_t)&huart2.Instance->DR, \
							    (uint32_t)usart2_dma_rxbuf[0], \
							    (uint32_t)usart2_dma_rxbuf[1], \
							    USART2_RX_DATA_FRAME_LEN);
}

/**
 *	@brief	USART4 Initialization
 */
void USART4_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart4);
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);	
	
	DMA_Start(huart4.hdmarx, \
			  (uint32_t)&huart4.Instance->DR, \
			  (uint32_t)usart4_dma_rxbuf, \
			  USART4_RX_BUF_LEN);
}

/**
 *	@brief	USART5 Initialization
 */
void USART5_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);	
	
	DMA_Start(huart5.hdmarx, \
			  (uint32_t)&huart5.Instance->DR, \
			  (uint32_t)usart5_dma_rxbuf, \
			  USART5_RX_BUF_LEN);
}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART1 处理协议
 */
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf)//视觉
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART2 处理协议
 */
__WEAK void USART2_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART4 处理协议
 */
__WEAK void USART4_rxDataHandler(uint8_t *rxBuf)
{	
}

/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART5 处理协议
 */
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf)//
{	
}

