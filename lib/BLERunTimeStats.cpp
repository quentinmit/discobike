#include <FreeRTOS.h>
#include <task.h>

#include "BLERunTimeStats.h"

const uint8_t BLERunTimeStats::UUID128_SERVICE[16] =
{
  0x87, 0x8d, 0x53, 0x29, 0x5f, 0x2e, 0x43, 0x08,
  0x85, 0xc9, 0xbd, 0x1f, 0x00, 0xFF, 0x00, 0x00
};

const uint8_t BLERunTimeStats::UUID128_CHR[16] =
  {
    0x87, 0x8d, 0x53, 0x29, 0x5f, 0x2e, 0x43, 0x08,
    0x85, 0xc9, 0xbd, 0x1f, 0x01, 0xFF, 0x00, 0x00
  };

BLERunTimeStats::BLERunTimeStats(BLEUuid service_uuid, BLEUuid data_uuid)
  : BLEService(service_uuid), _characteristic(data_uuid, CHR_PROPS_READ, BLE_GATTS_VAR_ATTR_LEN_MAX)
{
}

BLERunTimeStats::BLERunTimeStats()
  : BLEService(UUID128_SERVICE), _characteristic(UUID128_CHR, CHR_PROPS_READ, BLE_GATTS_VAR_ATTR_LEN_MAX)
{
}

err_t BLERunTimeStats::begin()
{
  VERIFY_STATUS( BLEService::begin() );

  _characteristic.setReadAuthorizeCallback(read_authorize_cb, false);//true);
  VERIFY_STATUS( _characteristic.begin() );

  return ERROR_NONE;
}

//char BLERunTimeStats::buf[2048];

void BLERunTimeStats::read_authorize_cb(uint16_t conn_hdl, BLECharacteristic* chr, ble_gatts_evt_read_t * request)
{
  ble_gatts_rw_authorize_reply_params_t reply;
  reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
  reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
  reply.params.read.update = 0;
  if (request->offset == 0) {
    // Only update the value if this is not a continuation of a previous read.
    char buf[2048];
    vTaskGetRunTimeStats(buf);
    uint16_t len = strlen(buf);
    if (len > BLE_GATTS_VAR_ATTR_LEN_MAX) {
      len = BLE_GATTS_VAR_ATTR_LEN_MAX;
    }
    reply.params.read.update = 1;
    reply.params.read.offset = 0;
    reply.params.read.len = len;
    reply.params.read.p_data = (uint8_t*) buf;
  }
  sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
}
