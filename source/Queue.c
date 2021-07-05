/*******************************************************************************
 * Include
 ******************************************************************************/
#include "Queue.h"

/*******************************************************************************
 * Variable
 ******************************************************************************/
static Queue_RingBuffer_t s_bufferQueue;/* struct queue data */

/*******************************************************************************
 * Variable
 ******************************************************************************/
#define QUEUE_CHAR_END_OF_LINE     0x0A /* character end of line in srec */

/*******************************************************************************
 * Code
 ******************************************************************************/
/* inititial queue */
void QUEUE_Init()
{
    uint8_t u8Count;
    uint8_t u8Line;
    /* reset queue*/
    s_bufferQueue.RxHead =0U;
    s_bufferQueue.RxTail = 0U;
    /* reset data in queue*/
    for(u8Line= 0u;u8Line < QUEUE_NUMBER_BUFFER;u8Line++ )
    {
        for(u8Count= 0u;u8Count < QUEUE_SIZE_RINGBUFF;u8Count++ )
        {
            s_bufferQueue.Queue_RingRxBuffer[u8Line][u8Count] = 0u;
        }
    }
}

/* push data into queue */
void QUEUE_ReceiveBuffer(uint8_t u8Data)
{
    /* posh char to buffer */
    s_bufferQueue.Queue_RingRxBuffer[s_bufferQueue.RxTail][s_bufferQueue.PositionInline] = u8Data;
    /* check none of end line */
    if(QUEUE_CHAR_END_OF_LINE != u8Data)
    {
        ++s_bufferQueue.PositionInline;
    }
    else/* get char end of line */
    {
        /* end of line queue */
        if(++s_bufferQueue.RxTail == QUEUE_NUMBER_BUFFER)
        {
            s_bufferQueue.RxTail = 0U;
        }
        s_bufferQueue.PositionInline = 0U;/* reset position of fisrt data in one line*/
    }
}

/* get address poiter of string data */
bool QUEUE_ReadData(uint8_t **u8AddData)
{
    bool retVal = true;

    /* check queue have data */
    if(s_bufferQueue.RxHead != s_bufferQueue.RxTail)
    {
        /* attach poiter of string data to poiter */
        *u8AddData = &s_bufferQueue.Queue_RingRxBuffer[s_bufferQueue.RxHead][0];
        if(++ s_bufferQueue.RxHead >= QUEUE_NUMBER_BUFFER)/* check head  is end of queue*/
        {
            s_bufferQueue.RxHead = 0;
        }
    }
    else
    {
        retVal = false;
    }

    return retVal;
}
/*******************************************************************************
 * END_FILE
 ******************************************************************************/