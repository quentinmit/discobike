#ifndef BLE_REMOTE_VARIABLE_H_
#define BLE_REMOTE_VARIABLE_H_

#include <BLECharacteristic.h>
#include <BLEService.h>

template <class T>
class BLERemoteVariable : public BLECharacteristic
{
 public:
  BLERemoteVariable(T* variable, BLEUuid data_uuid)
    : BLECharacteristic(data_uuid, CHR_PROPS_READ|CHR_PROPS_WRITE, sizeof(T), true), _variable(variable)
  {
  }

  err_t begin()
  {
    setReadAuthorizeCallback(read_authorize_cb, false);
    setWriteCallback(write_cb, false);
    VERIFY_STATUS( BLECharacteristic::begin() );

    return ERROR_NONE;
  }

 protected:
  T* _variable;

  static void read_authorize_cb(uint16_t conn_hdl, BLECharacteristic* chr, ble_gatts_evt_read_t * request)
  {
    ble_gatts_rw_authorize_reply_params_t reply;
    reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
    reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
    reply.params.read.update = 1;
    reply.params.read.offset = 0;
    reply.params.read.len = sizeof(T);
    BLERemoteVariable<T>* svc = (BLERemoteVariable<T>*)chr;
    reply.params.read.p_data = (uint8_t*) svc->_variable;
    sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
  }

  static void write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
  {
    if (len == sizeof(T)) {
      BLERemoteVariable<T>* svc = (BLERemoteVariable<T>*)chr;
      memcpy((uint8_t*) svc->_variable, data, len);
    } else {
      Serial.print("Tried to write ");
      Serial.print(len);
      Serial.print(" bytes, but variable is ");
      Serial.print(sizeof(T));
      Serial.println(" bytes.");
    }
  }
};

#endif
