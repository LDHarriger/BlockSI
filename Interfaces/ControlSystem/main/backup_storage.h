/**
 * @file backup_storage.h
 * @brief Flash-based backup storage for sequence data
 * 
 * Stores sequence recordings to SPIFFS during automated sequences.
 * Provides file listing and transfer capabilities for PC download.
 */

#ifndef BACKUP_STORAGE_H
#define BACKUP_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "model_106h_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of backup files to retain
 */
#define BACKUP_MAX_FILES        5

/**
 * @brief Maximum filename length
 */
#define BACKUP_MAX_FILENAME     48

/**
 * @brief File info structure
 */
typedef struct {
    char filename[BACKUP_MAX_FILENAME];
    uint32_t size_bytes;
    uint32_t sample_count;
    uint32_t timestamp;         // Unix timestamp of creation
} backup_file_info_t;

/**
 * @brief Storage status
 */
typedef struct {
    uint32_t total_bytes;
    uint32_t used_bytes;
    uint32_t free_bytes;
    uint8_t file_count;
    uint8_t max_files;
} backup_storage_status_t;

/**
 * @brief Initialize backup storage (mount SPIFFS)
 * 
 * @return ESP_OK on success
 */
esp_err_t backup_storage_init(void);

/**
 * @brief Deinitialize backup storage
 */
void backup_storage_deinit(void);

/**
 * @brief Start recording a new sequence
 * 
 * Creates a new file with naming: YYYY-MM-DD_<sequence_type>.csv
 * If max files reached, deletes oldest file.
 * 
 * @param sequence_type Sequence identifier (e.g., "Sterilization")
 * @return ESP_OK on success
 */
esp_err_t backup_start_sequence(const char *sequence_type);

/**
 * @brief Stop recording current sequence
 * 
 * @return ESP_OK on success
 */
esp_err_t backup_stop_sequence(void);

/**
 * @brief Check if currently recording
 * 
 * @return true if recording active
 */
bool backup_is_recording(void);

/**
 * @brief Get current sequence name
 * 
 * @return Sequence name or NULL if not recording
 */
const char *backup_get_sequence_name(void);

/**
 * @brief Write a sample to backup (if recording)
 * 
 * @param sample Sample data from 106-H
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not recording
 */
esp_err_t backup_write_sample(const m106h_sample_t *sample);

/**
 * @brief Get storage status
 * 
 * @param status Output status structure
 * @return ESP_OK on success
 */
esp_err_t backup_get_status(backup_storage_status_t *status);

/**
 * @brief List all backup files
 * 
 * @param files Array to fill with file info
 * @param max_files Size of array
 * @param count Output: number of files found
 * @return ESP_OK on success
 */
esp_err_t backup_list_files(backup_file_info_t *files, uint8_t max_files, uint8_t *count);

/**
 * @brief Get info for a specific file
 * 
 * @param filename File to query
 * @param info Output file info
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not exists
 */
esp_err_t backup_get_file_info(const char *filename, backup_file_info_t *info);

/**
 * @brief Delete a backup file
 * 
 * @param filename File to delete
 * @return ESP_OK on success
 */
esp_err_t backup_delete_file(const char *filename);

/**
 * @brief Delete all backup files
 * 
 * @return ESP_OK on success
 */
esp_err_t backup_delete_all(void);

/**
 * @brief Read file chunk for transfer
 * 
 * Used for streaming file to PC in chunks.
 * 
 * @param filename File to read
 * @param offset Byte offset to start reading
 * @param buffer Output buffer
 * @param buffer_size Size of buffer
 * @param bytes_read Output: actual bytes read
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if done/error
 */
esp_err_t backup_read_chunk(const char *filename, uint32_t offset,
                            char *buffer, uint32_t buffer_size, 
                            uint32_t *bytes_read);

/**
 * @brief Format storage (delete all and reinitialize)
 * 
 * @return ESP_OK on success
 */
esp_err_t backup_format(void);

#ifdef __cplusplus
}
#endif

#endif // BACKUP_STORAGE_H
