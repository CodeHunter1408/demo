/*
    Name:
    Copyright:
    Author: ThuanND22__FSoft
    Date: 06/05/21 07:30
    Description:
*/

#ifndef __SREC_PARSE_H__
#define __SREC_PARSE_H__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*
@brief: Buffer checked s , type , all data
parameter : Buffer : array data read
            ret: true if data buffer is true
                 false if data buffer is fail
*/
bool SREC_RecordCheck(uint8_t *u8PtrData);

#endif /* __SREC_PARSE_H__ */
