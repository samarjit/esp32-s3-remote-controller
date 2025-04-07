/* 
* Copyright (c) 2023,2024 P.Cook (alias 'plainFlight')
*
* This file is part of the PlainFlightController distribution (https://github.com/plainFlight/plainFlightController).
* 
* This program is free software: you can redistribute it and/or modify  
* it under the terms of the GNU General Public License as published by  
* the Free Software Foundation, version 3.
*
* This program is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
* While very different, this module has been inspired by and gives credit to:
*
* Brian R Taylor
* brian.taylor@bolderflight.com
* Copyright (c) 2022 Bolder Flight Systems Inc
* https://github.com/bolderflight/sbus
*/

#include "Arduino.h"
#include "defines.h"
#include "Sbus_Rx.h"  

//Sbus related defines
#define SBUS_BAUD       100000
#define PAYLOAD_LEN     23
#define HEADER_LEN      1
#define FOOTER_LEN      1
#define NUM_SBUS_CH     06
#define HEADER          0xFF
#define FOOTER          0x00
#define FOOTER2         0x04
#define CH17_MASK       0x01
#define CH18_MASK       0x02
#define LOST_FRAME_MASK 0x04
#define FAILSAFE_MASK   0x08
  
Sbus_Data rxData;

/*
* DESCRIPTION: Sets up serial port for inverted serial sbus.
*/
void initSbusRx(void)
{
  Serial2.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, SBUS_TX_PIN, true);
  Serial2.flush();
}

typedef enum
{  
  switch_low = 0U,
  switch_middle = 1U,
  switch_high = 2U  
}Switch_States;

typedef struct
{
  int32_t throttle;
  int32_t roll;
  int32_t pitch;
  int32_t yaw;
  bool armSwitch;
  Switch_States modeSwitch;
  Switch_States aux1Switch;
  Switch_States aux2Switch;
  //Note: Ignoring the channels 7-15 from SBUS
  bool failsafe;
  bool throttleIsLow;
  bool headingHold;
  bool newSbusPacket;
}Rx_Commands;
Rx_Commands rxCommand = {0};

bool sbusRxMsg(void);

void getSbus(void) 
{
  static uint64_t sbusTimeout = 0;

  if (sbusRxMsg()) 
  {
    rxCommand.newSbusPacket = true;
    sbusTimeout = micros() + SBUS_TIMEOUT;
    #if defined(SBUS_DEBUG)
      printSbusData();
    #endif
  }
  else if (micros() >= sbusTimeout)
  {
    //For this situation we need to detect and force failsafe flag.
    rxCommand.failsafe = true;
  }
}

/*
* DESCRIPTION: Reads in serial sbus data from receiver.
* Note: To avoid blocking main loop we read a maximum of 6 bytes in one pass of this function. This means that it takes 5 calls of this function to 
* read the 25 bytes of an sbus message. This does technically delay the sbus update by 4ms - but you or your plane are not going to notice this.
*/
bool sbusRxMsg(void) 
{
  static uint32_t count = 0;
  static uint32_t prevByte = FOOTER;
  static uint32_t buff[25];
  uint32_t currentByte;
  uint32_t rxCount = 0;

  //We loop for a maximum of 6 bytes then get out of here to avoid blocking main loop.
  while (Serial2.available() && (rxCount < 6)) 
  {
    currentByte = Serial2.read();
    rxCount++;

    if (count == 0) 
    {
      if ((currentByte == HEADER) && ((prevByte == FOOTER) || ((prevByte & 0x0F) == FOOTER2))) 
      {
        buff[count] = currentByte;
        count++;
      } 
      else 
      {
        count = 0;
      }
    } 
    else if (count < (PAYLOAD_LEN + HEADER_LEN)) 
    {
        buff[count] = currentByte;
        count++;
    } 
    else if (count < (PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN)) 
    {
      count = 0;
      prevByte = currentByte;
      if ((currentByte == FOOTER) || ((currentByte & 0x0F) == FOOTER2)) 
      {
        //Decode sbus data
        rxData.ch[0]  = (uint32_t)(buff[1] | ((buff[2] << 8U) & 0x07FF));
        rxData.ch[1]  = (uint32_t)((buff[2] >> 3U) | ((buff[3] << 5U) & 0x07FF));
        rxData.ch[2]  = (uint32_t)((buff[3] >> 6U) | (buff[4] << 2U) | ((buff[5] << 10U) & 0x07FF));
        rxData.ch[3]  = (uint32_t)((buff[5] >> 1U) | ((buff[6] << 7U) & 0x07FF));
        rxData.ch[4]  = (uint32_t)((buff[6] >> 4U) | ((buff[7] << 4U) & 0x07FF));
        rxData.ch[5]  = (uint32_t)((buff[7] >> 7U) | (buff[8] << 1U) | ((buff[9] << 9U) & 0x07FF));
        rxData.ch[6]  = (uint32_t)((buff[9] >> 2U) | ((buff[10] << 6U) & 0x07FF));
        rxData.ch[7]  = (uint32_t)((buff[10] >> 5U) | ((buff[11] << 3U) & 0x07FF));
        #if defined(USE_ALL_18_CHANNELS)
          rxData.ch[8]  = (uint32_t)(buff[12] | ((buff[13] << 8U) & 0x07FF));
          rxData.ch[9]  = (uint32_t)((buff[13] >> 3U) | ((buff[14] << 5U) & 0x07FF));
          rxData.ch[10] = (uint32_t)((buff[14] >> 6U) | (buff[15] << 2U) | (buff[16] << 10U) & 0x07FF));
          rxData.ch[11] = (uint32_t)((buff[16] >> 1U) | ((buff[17] << 7U) & 0x07FF));
          rxData.ch[12] = (uint32_t)((buff[17] >> 4U) | ((buff[18] << 4U) & 0x07FF));
          rxData.ch[13] = (uint32_t)((buff[18] >> 7U) | (buff[19] << 1U) | ((buff[20] << 9U) & 0x07FF));
          rxData.ch[14] = (uint32_t)((buff[20] >> 2U) | ((buff[21] << 6U) & 0x07FF));
          rxData.ch[15] = (uint32_t)((buff[21] >> 5U) | ((buff[22] << 3U) & 0x07FF));                               
          rxData.ch17 = (buff[23] & CH17_MASK) ? true : false;
          rxData.ch18 = (buff[23] & CH18_MASK) ? true : false;
        #endif 
        //Decode lost frame bit
        rxData.lost_frame = (buff[23] & LOST_FRAME_MASK) ? true : false;
        //Decode failsafe bit
        rxData.failsafe = (buff[23] & FAILSAFE_MASK) ? true : false;
        return true;
      } 
      else 
      {
        return false;
      }
    } 
    else 
    {
      count = 0;
    }
    prevByte = currentByte;
  }
  return false;
}

Sbus_Data getSbusData(void)
{
  return rxData;
}