#include <esp_log.h>

#define LOG_ERR(...)  ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_WARN(...) ESP_LOGW(TAG, __VA_ARGS__)
#define LOG_INF(...)  ESP_LOGI(TAG, __VA_ARGS__)
#define LOG_DBG(...)  ESP_LOGD(TAG, __VA_ARGS__)

#define LOG_HEXDUMP(...) ESP_LOG_BUFFER_HEX(__VA_ARGS__)
