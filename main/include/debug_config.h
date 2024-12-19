#ifndef DEBUG_CONFIG_H
#define DEBUG_CONFIG_H

// Master debug switch - Set to false to disable all debug output
#define DEBUG_ENABLE false

// Individual debug category flags
#define DEBUG_VU_METER false   // Disable VU meter output for less console traffic
#define DEBUG_WARNINGS true    // Keep warnings enabled
#define DEBUG_ERRORS true      // Keep errors enabled for critical issues
#define DEBUG_INFO false       // Disable general info messages


// Debug print macros for ESP logging
#if DEBUG_ENABLE
    // Info level messages
    #if DEBUG_INFO
        #define DEBUG_INFO_PRINT(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
    #else
        #define DEBUG_INFO_PRINT(tag, format, ...) 
    #endif

    // Warning messages
    #if DEBUG_WARNINGS
        #define DEBUG_WARNING_PRINT(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
    #else
        #define DEBUG_WARNING_PRINT(tag, format, ...)
    #endif

    // Error messages
    #if DEBUG_ERRORS
        #define DEBUG_ERROR_PRINT(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
    #else
        #define DEBUG_ERROR_PRINT(tag, format, ...)
    #endif

    // VU meter specific printing
    #if DEBUG_VU_METER
        #define DEBUG_VU_PRINT(format, ...) printf(format, ##__VA_ARGS__)
    #else
        #define DEBUG_VU_PRINT(format, ...)
    #endif

#else
    // If master debug is disabled, all prints are disabled
    #define DEBUG_INFO_PRINT(tag, format, ...)
    #define DEBUG_WARNING_PRINT(tag, format, ...)
    #define DEBUG_ERROR_PRINT(tag, format, ...)
    #define DEBUG_VU_PRINT(format, ...)
#endif

// Function to print current debug configuration
static void print_debug_config(void) {
    printf("\nDebug Configuration:\n");
    printf("Master Debug:  %s\n", DEBUG_ENABLE ? "ON" : "OFF");
    printf("VU Meter:     %s\n", DEBUG_VU_METER ? "ON" : "OFF");
    printf("Warnings:     %s\n", DEBUG_WARNINGS ? "ON" : "OFF");
    printf("Errors:       %s\n", DEBUG_ERRORS ? "ON" : "OFF");
    printf("Info:         %s\n", DEBUG_INFO ? "ON" : "OFF");
    printf("\n");
}

#endif // DEBUG_CONFIG_H