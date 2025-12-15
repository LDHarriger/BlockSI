/**
 * @file backup_storage.c
 * @brief Flash-based backup storage implementation using SPIFFS
 */

#include "backup_storage.h"
#include <string.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <limits.h>
#include <unistd.h>

#include "esp_spiffs.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "BACKUP";

// SPIFFS configuration
#define SPIFFS_BASE_PATH    "/backup"
#define SPIFFS_PARTITION    "storage"
#define SPIFFS_MAX_FILES    5

// Module state
static struct {
    bool initialized;
    bool recording;
    char sequence_name[32];
    char current_filename[BACKUP_MAX_FILENAME];
    FILE *current_file;
    uint32_t sample_count;
    uint32_t file_start_time;
} s_backup = {0};

/**
 * @brief Get current date string for filename
 */
static void get_date_string(char *buf, size_t len)
{
    time_t now;
    struct tm timeinfo;
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // If time not set, use a default or timestamp
    if (timeinfo.tm_year < (2020 - 1900)) {
        // Use boot time in seconds as fallback
        uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000);
        snprintf(buf, len, "T%lu", (unsigned long)uptime_sec);
    } else {
        strftime(buf, len, "%Y-%m-%d", &timeinfo);
    }
}

/**
 * @brief Check space and warn if low
 */
static void check_space_warning(void)
{
    size_t total = 0, used = 0;
    esp_spiffs_info(SPIFFS_PARTITION, &total, &used);
    
    uint32_t free_bytes = total - used;
    uint32_t free_pct = (free_bytes * 100) / total;
    
    if (free_pct < 20) {
        ESP_LOGW(TAG, "Storage low: %lu bytes free (%lu%%)", 
                 (unsigned long)free_bytes, (unsigned long)free_pct);
    }
}

/**
 * @brief Count existing backup files
 */
static int count_files(void)
{
    DIR *dir = opendir(SPIFFS_BASE_PATH);
    if (!dir) {
        return 0;
    }
    
    int count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {  // Regular file
            count++;
        }
    }
    closedir(dir);
    return count;
}

/**
 * @brief Safely build filepath from base path and filename
 * 
 * Limits filename to BACKUP_MAX_FILENAME to satisfy compiler bounds checking
 */
static void build_filepath(char *dest, size_t dest_size, const char *filename)
{
    char safe_name[BACKUP_MAX_FILENAME];
    size_t name_len = strlen(filename);
    if (name_len >= BACKUP_MAX_FILENAME) {
        name_len = BACKUP_MAX_FILENAME - 1;
    }
    memcpy(safe_name, filename, name_len);
    safe_name[name_len] = '\0';
    
    snprintf(dest, dest_size, "%s/%s", SPIFFS_BASE_PATH, safe_name);
}

/**
 * @brief Find and delete oldest file
 */
static esp_err_t delete_oldest_file(void)
{
    DIR *dir = opendir(SPIFFS_BASE_PATH);
    if (!dir) {
        return ESP_FAIL;
    }
    
    char oldest_name[BACKUP_MAX_FILENAME] = {0};
    time_t oldest_time = LONG_MAX;
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {
            char filepath[80];
            build_filepath(filepath, sizeof(filepath), entry->d_name);
            
            struct stat st;
            if (stat(filepath, &st) == 0) {
                if (st.st_mtime < oldest_time) {
                    oldest_time = st.st_mtime;
                    strncpy(oldest_name, entry->d_name, sizeof(oldest_name) - 1);
                }
            }
        }
    }
    closedir(dir);
    
    if (oldest_name[0]) {
        char filepath[80];
        build_filepath(filepath, sizeof(filepath), oldest_name);
        ESP_LOGW(TAG, "Deleting oldest backup: %s", oldest_name);
        unlink(filepath);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t backup_storage_init(void)
{
    if (s_backup.initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing backup storage");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = SPIFFS_BASE_PATH,
        .partition_label = SPIFFS_PARTITION,
        .max_files = SPIFFS_MAX_FILES,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "SPIFFS partition not found");
        } else {
            ESP_LOGE(TAG, "SPIFFS init failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    
    // Check and log storage info
    size_t total = 0, used = 0;
    esp_spiffs_info(SPIFFS_PARTITION, &total, &used);
    ESP_LOGI(TAG, "Storage: %lu total, %lu used, %lu free", 
             (unsigned long)total, (unsigned long)used, 
             (unsigned long)(total - used));
    
    int file_count = count_files();
    ESP_LOGI(TAG, "Existing backup files: %d", file_count);
    
    s_backup.initialized = true;
    s_backup.recording = false;
    
    check_space_warning();
    
    return ESP_OK;
}

void backup_storage_deinit(void)
{
    if (!s_backup.initialized) {
        return;
    }
    
    if (s_backup.recording) {
        backup_stop_sequence();
    }
    
    esp_vfs_spiffs_unregister(SPIFFS_PARTITION);
    s_backup.initialized = false;
    
    ESP_LOGI(TAG, "Backup storage deinitialized");
}

esp_err_t backup_start_sequence(const char *sequence_type)
{
    if (!s_backup.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_backup.recording) {
        ESP_LOGW(TAG, "Already recording, stopping current sequence");
        backup_stop_sequence();
    }
    
    // Check if we need to delete old files
    int file_count = count_files();
    if (file_count >= BACKUP_MAX_FILES) {
        ESP_LOGW(TAG, "Max files (%d) reached, deleting oldest", BACKUP_MAX_FILES);
        delete_oldest_file();
    }
    
    // Build filename
    char date_str[16];
    get_date_string(date_str, sizeof(date_str));
    
    snprintf(s_backup.current_filename, sizeof(s_backup.current_filename),
             "%s_%s.csv", date_str, sequence_type);
    
    // Store sequence name
    strncpy(s_backup.sequence_name, sequence_type, sizeof(s_backup.sequence_name) - 1);
    s_backup.sequence_name[sizeof(s_backup.sequence_name) - 1] = '\0';
    
    // Open file
    char filepath[80];
    snprintf(filepath, sizeof(filepath), "%s/%s", SPIFFS_BASE_PATH, s_backup.current_filename);
    
    s_backup.current_file = fopen(filepath, "w");
    if (!s_backup.current_file) {
        ESP_LOGE(TAG, "Failed to create file: %s", filepath);
        return ESP_FAIL;
    }
    
    // Write CSV header
    fprintf(s_backup.current_file, 
            "esp_timestamp_ms,ozone_wt_pct,temperature_c,pressure_mbar,"
            "sample_pdv_v,ref_pdv_v,day,month,year,hour,minute,second\n");
    fflush(s_backup.current_file);
    
    s_backup.sample_count = 0;
    s_backup.file_start_time = (uint32_t)(esp_timer_get_time() / 1000000);
    s_backup.recording = true;
    
    ESP_LOGI(TAG, "Started recording: %s", s_backup.current_filename);
    check_space_warning();
    
    return ESP_OK;
}

esp_err_t backup_stop_sequence(void)
{
    if (!s_backup.recording) {
        return ESP_OK;
    }
    
    if (s_backup.current_file) {
        fclose(s_backup.current_file);
        s_backup.current_file = NULL;
    }
    
    uint32_t duration = (uint32_t)(esp_timer_get_time() / 1000000) - s_backup.file_start_time;
    
    ESP_LOGI(TAG, "Stopped recording: %s (%lu samples, %lu sec)",
             s_backup.current_filename, 
             (unsigned long)s_backup.sample_count,
             (unsigned long)duration);
    
    s_backup.recording = false;
    s_backup.sequence_name[0] = '\0';
    
    return ESP_OK;
}

bool backup_is_recording(void)
{
    return s_backup.recording;
}

const char *backup_get_sequence_name(void)
{
    if (s_backup.recording) {
        return s_backup.sequence_name;
    }
    return NULL;
}

esp_err_t backup_write_sample(const m106h_sample_t *sample)
{
    if (!s_backup.recording || !s_backup.current_file) {
        return ESP_ERR_INVALID_STATE;
    }
    
    int64_t timestamp_ms = esp_timer_get_time() / 1000;
    
    int written = fprintf(s_backup.current_file,
                          "%lld,%.4f,%.2f,%.2f,%.5f,%.5f,%u,%u,%u,%u,%u,%u\n",
                          timestamp_ms,
                          sample->ozone_wt_pct,
                          sample->temperature_c,
                          sample->pressure_mbar,
                          sample->sample_pdv_v,
                          sample->ref_pdv_v,
                          (unsigned)sample->day,
                          (unsigned)sample->month,
                          (unsigned)sample->year,
                          (unsigned)sample->hour,
                          (unsigned)sample->minute,
                          (unsigned)sample->second);
    
    if (written < 0) {
        ESP_LOGE(TAG, "Write failed");
        return ESP_FAIL;
    }
    
    // Flush periodically (every 10 samples) to ensure data is saved
    s_backup.sample_count++;
    if (s_backup.sample_count % 10 == 0) {
        fflush(s_backup.current_file);
    }
    
    return ESP_OK;
}

esp_err_t backup_get_status(backup_storage_status_t *status)
{
    if (!s_backup.initialized || !status) {
        return ESP_ERR_INVALID_STATE;
    }
    
    size_t total = 0, used = 0;
    esp_spiffs_info(SPIFFS_PARTITION, &total, &used);
    
    status->total_bytes = total;
    status->used_bytes = used;
    status->free_bytes = total - used;
    status->file_count = count_files();
    status->max_files = BACKUP_MAX_FILES;
    
    return ESP_OK;
}

esp_err_t backup_list_files(backup_file_info_t *files, uint8_t max_files, uint8_t *count)
{
    if (!s_backup.initialized || !files || !count) {
        return ESP_ERR_INVALID_STATE;
    }
    
    DIR *dir = opendir(SPIFFS_BASE_PATH);
    if (!dir) {
        *count = 0;
        return ESP_OK;
    }
    
    *count = 0;
    struct dirent *entry;
    
    while ((entry = readdir(dir)) != NULL && *count < max_files) {
        if (entry->d_type == DT_REG) {
            char filepath[80];
            build_filepath(filepath, sizeof(filepath), entry->d_name);
            
            struct stat st;
            if (stat(filepath, &st) == 0) {
                strncpy(files[*count].filename, entry->d_name, BACKUP_MAX_FILENAME - 1);
                files[*count].filename[BACKUP_MAX_FILENAME - 1] = '\0';
                files[*count].size_bytes = st.st_size;
                files[*count].timestamp = st.st_mtime;
                
                // Count lines for sample_count (approximate)
                files[*count].sample_count = (st.st_size > 100) ? (st.st_size / 80) : 0;
                
                (*count)++;
            }
        }
    }
    
    closedir(dir);
    return ESP_OK;
}

esp_err_t backup_get_file_info(const char *filename, backup_file_info_t *info)
{
    if (!s_backup.initialized || !filename || !info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    char filepath[80];
    snprintf(filepath, sizeof(filepath), "%s/%s", SPIFFS_BASE_PATH, filename);
    
    struct stat st;
    if (stat(filepath, &st) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    strncpy(info->filename, filename, BACKUP_MAX_FILENAME - 1);
    info->filename[BACKUP_MAX_FILENAME - 1] = '\0';
    info->size_bytes = st.st_size;
    info->timestamp = st.st_mtime;
    info->sample_count = (st.st_size > 100) ? (st.st_size / 80) : 0;
    
    return ESP_OK;
}

esp_err_t backup_delete_file(const char *filename)
{
    if (!s_backup.initialized || !filename) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Don't delete currently recording file
    if (s_backup.recording && strcmp(filename, s_backup.current_filename) == 0) {
        ESP_LOGE(TAG, "Cannot delete active recording");
        return ESP_ERR_INVALID_STATE;
    }
    
    char filepath[80];
    snprintf(filepath, sizeof(filepath), "%s/%s", SPIFFS_BASE_PATH, filename);
    
    if (unlink(filepath) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Deleted: %s", filename);
    return ESP_OK;
}

esp_err_t backup_delete_all(void)
{
    if (!s_backup.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_backup.recording) {
        backup_stop_sequence();
    }
    
    DIR *dir = opendir(SPIFFS_BASE_PATH);
    if (!dir) {
        return ESP_OK;
    }
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {
            char filepath[80];
            build_filepath(filepath, sizeof(filepath), entry->d_name);
            unlink(filepath);
        }
    }
    
    closedir(dir);
    ESP_LOGI(TAG, "Deleted all backup files");
    return ESP_OK;
}

esp_err_t backup_read_chunk(const char *filename, uint32_t offset,
                            char *buffer, uint32_t buffer_size, 
                            uint32_t *bytes_read)
{
    if (!s_backup.initialized || !filename || !buffer || !bytes_read) {
        return ESP_ERR_INVALID_ARG;
    }
    
    char filepath[80];
    snprintf(filepath, sizeof(filepath), "%s/%s", SPIFFS_BASE_PATH, filename);
    
    FILE *f = fopen(filepath, "r");
    if (!f) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Seek to offset
    if (fseek(f, offset, SEEK_SET) != 0) {
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read chunk
    *bytes_read = fread(buffer, 1, buffer_size, f);
    fclose(f);
    
    if (*bytes_read == 0 && offset > 0) {
        // End of file
        return ESP_ERR_NOT_FOUND;
    }
    
    return ESP_OK;
}

esp_err_t backup_format(void)
{
    if (s_backup.recording) {
        backup_stop_sequence();
    }
    
    if (s_backup.initialized) {
        esp_vfs_spiffs_unregister(SPIFFS_PARTITION);
        s_backup.initialized = false;
    }
    
    ESP_LOGW(TAG, "Formatting storage...");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = SPIFFS_BASE_PATH,
        .partition_label = SPIFFS_PARTITION,
        .max_files = SPIFFS_MAX_FILES,
        .format_if_mount_failed = true
    };
    
    // Force format
    esp_err_t ret = esp_spiffs_format(SPIFFS_PARTITION);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Format failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Remount
    ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_OK) {
        s_backup.initialized = true;
        ESP_LOGI(TAG, "Storage formatted and remounted");
    }
    
    return ret;
}
