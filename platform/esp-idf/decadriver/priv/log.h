#include <esp_log.h>

#define LOG_ERR(...)  ESP_LOGE(LOG_TAG, __VA_ARGS__)
#define LOG_WARN(...) ESP_LOGW(LOG_TAG, __VA_ARGS__)
#define LOG_INF(...)  ESP_LOGI(LOG_TAG, __VA_ARGS__)
#define LOG_DBG(...)  ESP_LOGD(LOG_TAG, __VA_ARGS__)

#define LOG_HEXDUMP(...) ESP_LOG_BUFFER_HEX(__VA_ARGS__)

#define DBG_UWB(...) ESP_LOGD(LOG_TAG, __VA_ARGS__)
