/**
 * @file main_backup_handlers.c
 * @brief Backup command handlers to integrate into main.c lan_command_handler
 * 
 * Add these cases to your lan_command_handler switch/if-else block
 */

#include "backup_storage.h"

// Add to includes in main.c:
// #include "backup_storage.h"

// Add to app_main() initialization, after relay_control_init():
//     backup_storage_init();

// Add to on_106h_sample() callback, after LAN send:
//     if (backup_is_recording()) {
//         backup_write_sample(sample);
//     }

// Add these command handlers to lan_command_handler():

/**
 * Handle: CMD,backup_start,<sequence_type>
 */
static bool handle_backup_start(const char *args, char *response, size_t resp_size)
{
    if (!args || !args[0]) {
        snprintf(response, resp_size, "missing_sequence_type");
        return false;
    }
    
    esp_err_t err = backup_start_sequence(args);
    if (err == ESP_OK) {
        snprintf(response, resp_size, "started=%s", args);
        
        // Notify PC of file change
        char notify[64];
        snprintf(notify, sizeof(notify), "FILE,START,%s", args);
        lan_client_send_message(notify);
        
        return true;
    }
    
    snprintf(response, resp_size, "start_failed");
    return false;
}

/**
 * Handle: CMD,backup_stop
 */
static bool handle_backup_stop(char *response, size_t resp_size)
{
    const char *seq_name = backup_get_sequence_name();
    
    esp_err_t err = backup_stop_sequence();
    if (err == ESP_OK) {
        snprintf(response, resp_size, "stopped=%s", seq_name ? seq_name : "none");
        
        // Notify PC
        lan_client_send_message("FILE,STOP");
        
        return true;
    }
    
    snprintf(response, resp_size, "stop_failed");
    return false;
}

/**
 * Handle: CMD,backup_status
 */
static bool handle_backup_status(char *response, size_t resp_size)
{
    backup_storage_status_t status;
    if (backup_get_status(&status) != ESP_OK) {
        snprintf(response, resp_size, "status_failed");
        return false;
    }
    
    snprintf(response, resp_size, 
             "total=%lu,used=%lu,free=%lu,files=%u/%u,recording=%d",
             (unsigned long)status.total_bytes,
             (unsigned long)status.used_bytes,
             (unsigned long)status.free_bytes,
             status.file_count,
             status.max_files,
             backup_is_recording() ? 1 : 0);
    
    return true;
}

/**
 * Handle: CMD,backup_list
 */
static bool handle_backup_list(char *response, size_t resp_size)
{
    backup_file_info_t files[BACKUP_MAX_FILES];
    uint8_t count = 0;
    
    if (backup_list_files(files, BACKUP_MAX_FILES, &count) != ESP_OK) {
        snprintf(response, resp_size, "list_failed");
        return false;
    }
    
    if (count == 0) {
        snprintf(response, resp_size, "no_files");
        return true;
    }
    
    // Format: file1.csv:1234:50;file2.csv:5678:100
    char *p = response;
    size_t remaining = resp_size;
    
    for (uint8_t i = 0; i < count && remaining > 50; i++) {
        int len = snprintf(p, remaining, "%s%s:%lu:%lu",
                          (i > 0) ? ";" : "",
                          files[i].filename,
                          (unsigned long)files[i].size_bytes,
                          (unsigned long)files[i].sample_count);
        if (len > 0 && len < (int)remaining) {
            p += len;
            remaining -= len;
        }
    }
    
    return true;
}

/**
 * Handle: CMD,backup_delete,<filename>
 */
static bool handle_backup_delete(const char *args, char *response, size_t resp_size)
{
    if (!args || !args[0]) {
        snprintf(response, resp_size, "missing_filename");
        return false;
    }
    
    esp_err_t err = backup_delete_file(args);
    if (err == ESP_OK) {
        snprintf(response, resp_size, "deleted=%s", args);
        return true;
    } else if (err == ESP_ERR_INVALID_STATE) {
        snprintf(response, resp_size, "cannot_delete_active");
        return false;
    }
    
    snprintf(response, resp_size, "not_found");
    return false;
}

/**
 * Handle: CMD,backup_delete_all
 */
static bool handle_backup_delete_all(char *response, size_t resp_size)
{
    esp_err_t err = backup_delete_all();
    if (err == ESP_OK) {
        snprintf(response, resp_size, "all_deleted");
        return true;
    }
    
    snprintf(response, resp_size, "delete_failed");
    return false;
}

/**
 * Handle: CMD,backup_download,<filename>
 * 
 * Sends file in chunks via LAN
 */
static bool handle_backup_download(const char *args, char *response, size_t resp_size)
{
    if (!args || !args[0]) {
        snprintf(response, resp_size, "missing_filename");
        return false;
    }
    
    backup_file_info_t info;
    if (backup_get_file_info(args, &info) != ESP_OK) {
        snprintf(response, resp_size, "not_found");
        return false;
    }
    
    // Send file in chunks
    char chunk_buf[256];
    char send_buf[512];
    uint32_t offset = 0;
    uint32_t bytes_read;
    
    // Send start notification
    snprintf(send_buf, sizeof(send_buf), "CHUNK,START,%s,%lu",
             args, (unsigned long)info.size_bytes);
    lan_client_send_message(send_buf);
    
    while (backup_read_chunk(args, offset, chunk_buf, sizeof(chunk_buf), &bytes_read) == ESP_OK 
           && bytes_read > 0) {
        
        // Base64 encode chunk (simplified - in practice use proper base64)
        // For now, send as hex or raw
        // This is a simplified implementation - production would use proper chunked transfer
        
        snprintf(send_buf, sizeof(send_buf), "CHUNK,%lu,%lu,",
                 (unsigned long)offset, (unsigned long)bytes_read);
        
        // Append chunk data (in real implementation, base64 encode)
        size_t header_len = strlen(send_buf);
        if (header_len + bytes_read < sizeof(send_buf)) {
            memcpy(send_buf + header_len, chunk_buf, bytes_read);
            send_buf[header_len + bytes_read] = '\0';
        }
        
        lan_client_send_message(send_buf);
        
        offset += bytes_read;
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between chunks
    }
    
    // Send end notification
    lan_client_send_message("CHUNK,END");
    
    snprintf(response, resp_size, "sent=%lu", (unsigned long)offset);
    return true;
}


/**
 * Integration: Add to lan_command_handler() in main.c
 * 
 * Example lan_command_handler with backup commands:
 */
#if 0  // Example integration - modify your existing handler

static bool lan_command_handler(const char *cmd, const char *args,
                                char *response, size_t resp_size)
{
    ESP_LOGI(TAG, "LAN CMD: %s, args: %s", cmd, args ? args : "none");
    
    // Relay commands
    if (strcmp(cmd, "relay_set") == 0) {
        // ... existing relay_set handling ...
    }
    else if (strcmp(cmd, "relay_get") == 0) {
        // ... existing relay_get handling ...
    }
    else if (strcmp(cmd, "relay_all_off") == 0) {
        // ... existing relay_all_off handling ...
    }
    else if (strcmp(cmd, "status") == 0) {
        // ... existing status handling ...
    }
    
    // Backup commands
    else if (strcmp(cmd, "backup_start") == 0) {
        return handle_backup_start(args, response, resp_size);
    }
    else if (strcmp(cmd, "backup_stop") == 0) {
        return handle_backup_stop(response, resp_size);
    }
    else if (strcmp(cmd, "backup_status") == 0) {
        return handle_backup_status(response, resp_size);
    }
    else if (strcmp(cmd, "backup_list") == 0) {
        return handle_backup_list(response, resp_size);
    }
    else if (strcmp(cmd, "backup_delete") == 0) {
        return handle_backup_delete(args, response, resp_size);
    }
    else if (strcmp(cmd, "backup_delete_all") == 0) {
        return handle_backup_delete_all(response, resp_size);
    }
    else if (strcmp(cmd, "backup_download") == 0) {
        return handle_backup_download(args, response, resp_size);
    }
    
    else {
        snprintf(response, resp_size, "unknown_command");
        return false;
    }
    
    return true;
}

#endif
