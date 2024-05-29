
// based on LIDARLite_v3HP.h: https://github.com/RobotShop/LIDARLite_v3_Arduino_Library/blob/master/src/LIDARLite_v3HP.h
#ifndef BB_LIDARLITE_V3HP_H
#define BB_LIDARLITE_V3HP_H

#include <robotcontrol.h>
#include <stdint.h>

#define LIDARLITE_ADDR_DEFAULT 0x62

class LIDARLite_v3HP
{
  public:
      void      configure   (uint8_t configuration = 0, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

      void      setI2Caddr  (uint8_t newAddress, uint8_t disableDefault, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      uint16_t  readDistance(uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      void      waitForBusy (uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      uint8_t   getBusyFlag (uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      void      takeRange   (uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

      void      write (uint8_t regAddr, uint8_t * dataBytes, uint16_t numBytes, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
      void      read  (uint8_t regAddr, uint8_t * dataBytes, uint16_t numBytes, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

      void      correlationRecordToSerial (uint16_t numberOfReadings = 1024, uint8_t lidarliteAddress = LIDARLITE_ADDR_DEFAULT);
};

#endif