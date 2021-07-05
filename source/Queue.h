#ifndef __QUEUE_H__
#define __QUEUE_H__
/*******************************************************************************
 * Include
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "MKL46Z4.h"

/*******************************************************************************
 * Define
 ******************************************************************************/
#define QUEUE_SIZE_RINGBUFF       80u/* max size of line in srec */
#define QUEUE_NUMBER_BUFFER       2u /* number of line */

/*******************************************************************************
 * Struct data
 ******************************************************************************/
/* struct data in queue */
typedef struct
{
    uint8_t Queue_RingRxBuffer[QUEUE_NUMBER_BUFFER][QUEUE_SIZE_RINGBUFF];/* buffer stores data */
    uint8_t RxTail;/* line data queue has data */
    uint8_t RxHead;/* line data readed */
    uint8_t PositionInline;/* position of data is writing in line */
} Queue_RingBuffer_t;

/**
 * @brief: reset all data in queue
 * @param none
 * @return none
 **/
void QUEUE_Init();
/**
 * @brief: receive data into queue
 * @param u8Data : data received
 * @return none s
 **/
void QUEUE_ReceiveBuffer(uint8_t u8Data);
/**
 * @brief: read string data from buffer queue
 * @param u8AddData : address of string data
 * @return true : have data in queue
 * @return false : haven't data in queue
 **/
bool QUEUE_ReadData(uint8_t **u8AddData);
#endif /*__QUEUE_H__ */