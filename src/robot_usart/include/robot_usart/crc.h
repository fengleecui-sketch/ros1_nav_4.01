/*
 * @Author: your name
 * @Date: 2023-06-26 09:08:31
 * @LastEditTime: 2023-07-08 00:17:11
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_usart/include/robot_usart/crc.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef _CRC_H
#define _CRC_H

#include "iostream"

using namespace std;

class CRC
{
  public:
    CRC(){}

  private:

  public:
    unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned  int dwLength,unsigned char ucCRC8);
    unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
    uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
    uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
    void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
    void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
};


#endif 
