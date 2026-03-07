/**
 * @file data_cache.h
 * @brief RAM ring buffer for DATA telemetry during LAN disconnection
 *
 * Caches pre-formatted DATA lines while the PC is disconnected.
 * On reconnect the PC requests backfill and the cache is drained oldest-first.
 *
 * Storage: static array of fixed-width entries in .bss (~82 KB at 512 slots).
 * Thread-safe via FreeRTOS mutex (push from 106-H UART task, drain from LAN task).
 */

#ifndef DATA_CACHE_H
#define DATA_CACHE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum length of a single cached DATA line (including '\n' + NUL). */
#define DATA_CACHE_LINE_MAX  160

/** Number of slots in the ring buffer. */
#define DATA_CACHE_SLOTS     500

/**
 * @brief Initialise the data cache (call once at startup).
 *
 * Creates the internal mutex.  Safe to call before any push/peek/pop.
 *
 * @return ESP_OK on success
 */
esp_err_t data_cache_init(void);

/**
 * @brief Push a DATA line into the ring buffer.
 *
 * If the buffer is full the **oldest** entry is silently overwritten (ring).
 * The line is truncated to DATA_CACHE_LINE_MAX-1 characters.
 *
 * @param line  NUL-terminated DATA string (with or without trailing '\n')
 */
void data_cache_push(const char *line);

/**
 * @brief Return the number of cached entries waiting to be sent.
 */
uint16_t data_cache_count(void);

/**
 * @brief Peek at the oldest cached entry without removing it.
 *
 * @param[out] buf     Destination buffer (at least DATA_CACHE_LINE_MAX bytes)
 * @param      buf_len Size of destination buffer
 * @return true if an entry was copied, false if cache is empty
 */
bool data_cache_peek(char *buf, size_t buf_len);

/**
 * @brief Remove the oldest cached entry.
 *
 * Typically called after a successful send of the peek'd line.
 */
void data_cache_pop(void);

/**
 * @brief Discard all cached entries.
 */
void data_cache_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* DATA_CACHE_H */
