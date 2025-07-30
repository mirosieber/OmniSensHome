#include "TaskMonitor.h"
#include <esp_heap_caps.h>
#include <inttypes.h>

uint32_t TaskMonitor::_report_interval_sec = 60;
TaskHandle_t TaskMonitor::_monitor_task_handle = nullptr;

void TaskMonitor::begin(uint32_t report_interval_sec) {
  _report_interval_sec = report_interval_sec;
  xTaskCreate(monitoring_task, "TaskMonitor",
              4096, // Stack size (adjust if needed)
              nullptr,
              tskIDLE_PRIORITY + 1, // Low priority
              &_monitor_task_handle);
  ESP_LOGI("TaskMonitor", "Started with %" PRIu32 "-second reporting interval",
           _report_interval_sec);
}

void TaskMonitor::print_task_stats() {
  // Print comprehensive task and heap information
  ESP_LOGI("TaskMonitor", "===== System Stats =====");

  // Get task count first
  UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
  ESP_LOGI("TaskMonitor", "Total Tasks: %d", (int)num_tasks);

  // Allocate memory for task status array
  TaskStatus_t *task_status_array =
      (TaskStatus_t *)pvPortMalloc(num_tasks * sizeof(TaskStatus_t));
  if (task_status_array == nullptr) {
    ESP_LOGE("TaskMonitor", "Failed to allocate memory for task status array");
    return;
  }

  // Get detailed task information
  uint32_t total_runtime;
  UBaseType_t actual_tasks =
      uxTaskGetSystemState(task_status_array, num_tasks, &total_runtime);

  ESP_LOGI("TaskMonitor", "----- Task Details -----");
  ESP_LOGI("TaskMonitor", "%-15s %-8s %-10s %-10s %-10s", "Name", "State",
           "Priority", "Stack HWM", "Runtime %%");

  for (UBaseType_t i = 0; i < actual_tasks; i++) {
    const char *state_str;
    switch (task_status_array[i].eCurrentState) {
    case eRunning:
      state_str = "Running";
      break;
    case eReady:
      state_str = "Ready";
      break;
    case eBlocked:
      state_str = "Blocked";
      break;
    case eSuspended:
      state_str = "Suspend";
      break;
    case eDeleted:
      state_str = "Deleted";
      break;
    default:
      state_str = "Unknown";
      break;
    }

    // Calculate runtime percentage
    uint32_t runtime_percent = 0;
    if (total_runtime > 0) {
      runtime_percent =
          (task_status_array[i].ulRunTimeCounter * 100) / total_runtime;
    }

    ESP_LOGI("TaskMonitor", "%-15s %-8s %-10u %-10u %-10" PRIu32,
             task_status_array[i].pcTaskName, state_str,
             (unsigned int)task_status_array[i].uxCurrentPriority,
             (unsigned int)task_status_array[i].usStackHighWaterMark,
             runtime_percent);
  }

  // Print heap status
  ESP_LOGI("TaskMonitor", "----- Heap Status -----");
  ESP_LOGI("TaskMonitor", "Free Heap: %" PRIu32 " bytes",
           esp_get_free_heap_size());
  ESP_LOGI("TaskMonitor", "Largest Block: %u bytes",
           (unsigned int)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  ESP_LOGI("TaskMonitor", "Min Free Ever: %u bytes",
           (unsigned int)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
  ESP_LOGI("TaskMonitor", "Total Runtime: %" PRIu32 " ticks", total_runtime);

  // Free allocated memory
  vPortFree(task_status_array);
}

void TaskMonitor::monitoring_task(void *pvParameters) {
  while (true) {
    print_task_stats();
    vTaskDelay(pdMS_TO_TICKS(_report_interval_sec * 1000));
  }
  vTaskDelete(nullptr);
}