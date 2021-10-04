#include "Task.h"

PeriodicTask::PeriodicTask(TickType_t period, const char* const name, UBaseType_t priority, void(*callback)(), NRF_WDT_Type *wdt)
  : _period(period), _name(name), _priority(priority), _callback(callback), _wdt(wdt)
{
}

void PeriodicTask::create()
{
  if (_wdt) {
    for (_wdt_channel = NRF_WDT_RR0; _wdt_channel <= NRF_WDT_RR7; _wdt_channel = (nrf_wdt_rr_register_t)((uint8_t)_wdt_channel+1)) {
      if (!nrf_wdt_reload_request_is_enabled(_wdt, _wdt_channel)) {
        nrf_wdt_reload_request_enable(_wdt, _wdt_channel);
        break;
      }
    }
    if (_wdt_channel > NRF_WDT_RR7) {
      _wdt = NULL;
    }
  }
  if (_task_hdl == NULL) {
    xTaskCreate(task, _name, STACK_SIZE, this, _priority, &_task_hdl);
  }
}

void PeriodicTask::task(void *pvParameters) {
  PeriodicTask* pt = (PeriodicTask*)pvParameters;

  TickType_t xLastWakeTime;

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, pt->_period );
    pt->_callback();
    if (pt->_wdt) {
      nrf_wdt_reload_request_set(pt->_wdt, pt->_wdt_channel);
    }
    // If we've gotten behind, reset the last wake time.
    if (xLastWakeTime + pt->_period < xTaskGetTickCount()) {
      xLastWakeTime = xTaskGetTickCount();
      pt->_skipped++;
    }
  }
}
