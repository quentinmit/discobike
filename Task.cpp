#include "Task.h"

PeriodicTask::PeriodicTask(TickType_t period, const char* const name, UBaseType_t priority, void(*callback)())
  : _period(period), _name(name), _priority(priority), _callback(callback)
{
}

void PeriodicTask::create()
{
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
    // If we've gotten behind, reset the last wake time.
    if (xLastWakeTime + pt->_period < xTaskGetTickCount()) {
      xLastWakeTime = xTaskGetTickCount();
      pt->_skipped++;
    }
  }
}
