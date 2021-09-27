#ifndef TASK_H_
#define TASK_H_

#include <FreeRTOS.h>
#include <task.h>

class PeriodicTask
{
 public:
  PeriodicTask(TickType_t period, const char* const name, UBaseType_t priority, void(*callback)());

  void create();

  uint32_t skipped() { return _skipped; }

 protected:
  const TickType_t _period;
  const char* const _name;
  UBaseType_t _priority;
  void(*_callback)();

  uint32_t _skipped = 0;

  TaskHandle_t _task_hdl = NULL;

  static void task(void *pvParameters);
};

#define STACK_SIZE 500

#endif
