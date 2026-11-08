#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } FDCAN_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct
{
  uint32_t IdType, RxFrameType, DataLength;
} FDCAN_RxHeaderTypeDef;
typedef struct
{
  uint32_t Identifier, DataLength;
} FDCAN_TxHeaderTypeDef;
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_8 8U
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *,
                                             FDCAN_TxHeaderTypeDef *, uint8_t *);
#endif
