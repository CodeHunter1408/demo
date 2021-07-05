/*
    Name: SREC_motorola.c
    Copyright:
    Author: ThuanND22
    Date: 06/05/21 07:30
    Description: read data file srec text
*/

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "SrecParse.h"

/*******************************************************************************
 * Define
 ******************************************************************************/
#define BYTE_START     0U
#define C_CHAR         0x53U/* charecter c in ascii*/
#define END_LINE_CHAR  0x0D
#define NEW_LINE_CHAR  0x0A
#define BYTE_STYPE     1U /* bit check type */
#define POSITION_BYTE_ADDRESS 4U
#define POSITION_BYTE_COUNT 2U
#define BYTE_NEXT  2u
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* check s7 s8 s9 */
static bool SREC_CheckSumAndEnd(uint8_t *u8PtrData , uint8_t u8byteCount);
static inline uint8_t SREC_ConvertCharInt(uint8_t Data);

/**
*@brief: check data of u8PtrData
*@param: u8PtrData : array data read
*@return: true if line is true
          false : line is flase
**/
static bool SREC_CheckType(uint8_t *u8PtrData);

/**
*@brief: Check Number Data s1,s2,s3
*@param: u8PtrData : array data read
*@param: NumData : number of data in srec
*@param: Bytecount: sum address bytes, data bytes ,count bytes
*@return: true if line is true
          false : line is flase
**/
static bool SREC_CheckNumData(uint8_t *u8BufferPtr , uint32_t u32numData, uint8_t u8byteCount);
/*
@brief: get 2 byte from count
parameter : Buffer : array data read
            ret: type return
            SumByte : sum 2 byte
*/
#define SREC_GET2BYTE(Ptr , Count) ((SREC_ConvertCharInt(Ptr[(Count)]) << 4) | (SREC_ConvertCharInt(Ptr[(Count)+1])))

/*******************************************************************************
 * Code
 ******************************************************************************/


/*
@brief: check data of u8PtrData
parameter : u8PtrData : array data read
            ret: type return
            CheckType : check type of data
            u8Count :count data s1 s2 s3 ;
            Bytecount: sum address bytes, data bytes ,count bytes
*/
static bool SREC_CheckType(uint8_t *u8PtrData)
{
    bool ret = true;
    /* get ByteCount from byte 2 */
    uint8_t ByteCount = SREC_GET2BYTE(u8PtrData,2);
    uint8_t CheckType = SREC_ConvertCharInt(u8PtrData[BYTE_STYPE]); /* get byte stype */
    static uint32_t u32Count = 0U;

    /* check type of u8PtrData */
    switch (CheckType)
        {
        case 0:
            ret = SREC_CheckSumAndEnd(u8PtrData, ByteCount); /* check sum */
            u32Count = 0U;/* reset 1 file new */
            break;
        case 1:/* though case 3 */
        case 2:/* though case 3 */
        case 3:
            ++u32Count; /* count data s1 s2 s3 */
            /* check sum and check data endline */
            ret = SREC_CheckSumAndEnd(u8PtrData , ByteCount);
            break;
        case 4:
            ret = false;
            break;
        case 5: /* though case 6 */
        case 6:
            ret = SREC_CheckNumData(u8PtrData, u32Count, ByteCount);
            break;
        case 7:/* though case 9 */
        case 8:/* though case 9 */
        case 9:
            ret = SREC_CheckSumAndEnd(u8PtrData,ByteCount);
            break;
        default:
            ret = false;
            break;
    }
    return ret;
}

/*
@brief: control 2 char to interger
parameter : u8PtrData : array data read
            Data : data want convert
*/
static inline uint8_t SREC_ConvertCharInt(uint8_t u8Data)
{
    /* if char is A B C D E , flow ASCII */
    if( u8Data > 64 && u8Data < 71  )
    {
        u8Data = u8Data - 55;
    }
    else/* if char is 0 -> 9 , flow ASCII */
    {
        u8Data = u8Data - 48;
    }
    return u8Data;
}

/*
@brief: u8PtrData checked s , type , all data
parameter : u8PtrData : array data read
            ret: type return
            RecordStart : byte Record Start
*/
bool SREC_RecordCheck(uint8_t *u8PtrData)
{
    bool ret = true;

    if(u8PtrData[BYTE_START] == C_CHAR)
    {
        ret = SREC_CheckType(u8PtrData);
    }
    else
    {
        ret = false;
    }
    return ret;
}

/*
@brief: check sum of data
parameter : u8PtrData : array data read
            ret: type return
            NumAdd : number byte address
            u8count : count byte on u8PtrData
            checksum : store value sum bytecount bytes, address bytes, data bytes
*/
static bool SREC_CheckSumAndEnd(uint8_t *u8PtrData , uint8_t Bytecount)
{
    bool ret = true;
    uint8_t Sum = 0U;
    uint8_t u8count; /* first of address byte */
    /*2 + bytecount*2   2 byte start ,
                        bytecount*2 is byte add */
    uint8_t byteOfSum = Bytecount*2 + POSITION_BYTE_COUNT ;

    /* check sum of data */
    for(u8count = POSITION_BYTE_COUNT ;u8count < byteOfSum ; u8count = u8count + BYTE_NEXT)
    {
        Sum += SREC_GET2BYTE(u8PtrData, u8count);
    }
    Sum = ~Sum;
    /* check data is end of line */
    /* 2 charecter after data is end of line , 3 charecter after data is new line*/
    if(u8PtrData[byteOfSum + 2] != END_LINE_CHAR|| u8PtrData[byteOfSum + 3] != NEW_LINE_CHAR)
    {
        ret = false;
    }
    else
    {
        if(Sum != SREC_GET2BYTE(u8PtrData,byteOfSum))/* get value of sumcheck */
        {
            ret = false;
        }
    }

    return ret;
}

/*
@brief: Check Number Data s1,s2,s3
parameter : u8PtrData : array data read
            ret: type return
            ByteAdd: number byte address
            SumCheck : store address of buffer
            u8Count : variable coute temperory
*/
static bool SREC_CheckNumData(uint8_t *u8BufferPtr , uint32_t u32numData, uint8_t u8byteCount)
{
    bool ret = true;
    /* sum of data byte+ 4 byte first period of byte address - 2 byte check sum*/
    uint8_t u8byteAdd = u8byteCount * 2 + POSITION_BYTE_ADDRESS - 2;
    uint32_t u32sumCheck = 0U;
    uint8_t u8Count = 0U;

    /* get address start byte address is 4*/
    for(u8Count = POSITION_BYTE_ADDRESS;u8Count < u8byteAdd ; u8Count += BYTE_NEXT)
    {
        u32sumCheck = (u32sumCheck << 8) + SREC_GET2BYTE(u8BufferPtr, u8Count);
    }
    /* check address with number data line*/
    if(u32sumCheck != u32numData)
    {
       if(true != SREC_CheckSumAndEnd(u8BufferPtr,u8byteCount))/* check sum and charecter end line , new line */
       {
            ret = false;
       }
    }

    return ret;
}
