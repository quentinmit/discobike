#ifndef BLE_RUN_TIME_STATS_H_
#define BLE_RUN_TIME_STATS_H_

#include <BLECharacteristic.h>
#include <BLEService.h>

class BLERunTimeStats : public BLEService
{
 public:
  static const uint8_t UUID128_SERVICE[16];
  static const uint8_t UUID128_CHR[16];

  BLERunTimeStats(void);
  BLERunTimeStats(BLEUuid service_uuid, BLEUuid data_uuid);

  err_t begin();

 protected:
  BLECharacteristic _characteristic;

  static void read_authorize_cb(uint16_t conn_hdl, BLECharacteristic* chr, ble_gatts_evt_read_t * request);

  //static char buf[2048];
};

#endif
