#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class TaskMonitor {
public:
  /**
   * @brief Initialize the TaskMonitor with a reporting interval.
   * @param report_interval_sec How often to log stats (in seconds).
   */
  static void begin(uint32_t report_interval_sec = 60);

  /**
   * @brief Manually trigger a report (useful for debugging).
   */
  static void print_task_stats();

private:
  static void monitoring_task(void *pvParameters);
  static uint32_t _report_interval_sec;
  static TaskHandle_t _monitor_task_handle;
};

#endif // TASK_MONITOR_H