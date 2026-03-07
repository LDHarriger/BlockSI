/**
 * @file data_cache.c
 * @brief RAM ring buffer for DATA telemetry during LAN disconnection
 *
 * Implementation: circular array of fixed-width char buffers guarded by
 * a FreeRTOS mutex.  ~80 KB in .bss (500 × 160 B).
 *
 * Push is called from the 106-H UART callback (~every 2 s).
 * Drain (peek/pop) is called from the LAN client task on reconnect.
 */

#include "data_cache.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "DATA_CACHE";

/* ── ring buffer storage ────────────────────────────────────────────── */

static char         s_ring[DATA_CACHE_SLOTS][DATA_CACHE_LINE_MAX];
static uint16_t     s_head  = 0;   /* next write position          */
static uint16_t     s_tail  = 0;   /* oldest unread position       */
static uint16_t     s_count = 0;   /* number of valid entries       */
static SemaphoreHandle_t s_mutex = NULL;

/* ── public API ─────────────────────────────────────────────────────── */

esp_err_t data_cache_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    ESP_LOGI(TAG, "Initialised (%u slots × %u B = %u KB)",
             DATA_CACHE_SLOTS, DATA_CACHE_LINE_MAX,
             (DATA_CACHE_SLOTS * DATA_CACHE_LINE_MAX) / 1024);
    return ESP_OK;
}

void data_cache_push(const char *line)
{
    if (!s_mutex || !line) return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Push: mutex timeout — sample dropped");
        return;
    }

    /* Copy into current head slot (truncate if too long) */
    strncpy(s_ring[s_head], line, DATA_CACHE_LINE_MAX - 1);
    s_ring[s_head][DATA_CACHE_LINE_MAX - 1] = '\0';

    s_head = (s_head + 1) % DATA_CACHE_SLOTS;

    if (s_count < DATA_CACHE_SLOTS) {
        s_count++;
    } else {
        /* Overwrite oldest — advance tail */
        s_tail = (s_tail + 1) % DATA_CACHE_SLOTS;
        ESP_LOGD(TAG, "Ring full — oldest entry overwritten");
    }

    xSemaphoreGive(s_mutex);
}

uint16_t data_cache_count(void)
{
    if (!s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t n = s_count;
    xSemaphoreGive(s_mutex);
    return n;
}

bool data_cache_peek(char *buf, size_t buf_len)
{
    if (!s_mutex || !buf || buf_len == 0) return false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_count == 0) {
        xSemaphoreGive(s_mutex);
        return false;
    }
    strncpy(buf, s_ring[s_tail], buf_len - 1);
    buf[buf_len - 1] = '\0';
    xSemaphoreGive(s_mutex);
    return true;
}

void data_cache_pop(void)
{
    if (!s_mutex) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_count > 0) {
        s_tail = (s_tail + 1) % DATA_CACHE_SLOTS;
        s_count--;
    }
    xSemaphoreGive(s_mutex);
}

void data_cache_clear(void)
{
    if (!s_mutex) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_head  = 0;
    s_tail  = 0;
    s_count = 0;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Cache cleared");
}
