#include "vl53l0x_platform.h"

#include <string.h>
#include <stdlib.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// In the new driver, ACK handling is internal.
// Keep this for compatibility if other code references it.
#define ACK_CHECK_EN true

static const char *TAG = "vl53l0x_platform";

VL53L0X_Error esp_to_vl53l0x_error(esp_err_t esp_err) {
    switch (esp_err) {
        case ESP_OK:
            return VL53L0X_ERROR_NONE;
        case ESP_ERR_INVALID_ARG:
            return VL53L0X_ERROR_INVALID_PARAMS;
        case ESP_FAIL:
        case ESP_ERR_INVALID_STATE:
            return VL53L0X_ERROR_CONTROL_INTERFACE;
        case ESP_ERR_TIMEOUT:
            return VL53L0X_ERROR_TIME_OUT;
        default:
            return VL53L0X_ERROR_UNDEFINED;
    }
}

/**
 * Writes the supplied byte buffer to the device
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;

    // Expect Dev to provide an i2c device handle (new driver).
    // Add this to your Dev struct: i2c_master_dev_handle_t i2c_dev;
    if (!Dev->i2c_dev) {
        ESP_LOGE(TAG, "Dev->i2c_dev is NULL (new I2C driver requires device handle)");
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    // Build TX buffer: [register index][payload...]
    size_t n = (size_t)count + 1;

    // Use stack for small writes; heap for larger.
    uint8_t stack_buf[64];
    uint8_t *buf = (n <= sizeof(stack_buf)) ? stack_buf : (uint8_t *)malloc(n);
    if (!buf) {
        ESP_LOGE(TAG, "malloc(%u) failed", (unsigned)n);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    buf[0] = index;
    if (count && pdata) {
        memcpy(&buf[1], pdata, count);
    }

    // Timeout: use a finite timeout like before (1000ms)
    esp_err_t ret = i2c_master_transmit(Dev->i2c_dev, buf, n, pdMS_TO_TICKS(1000));

    if (buf != stack_buf) free(buf);

    return esp_to_vl53l0x_error(ret);
}

/**
 * Reads the requested number of bytes from the device
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (!Dev) return VL53L0X_ERROR_INVALID_PARAMS;

    if (!Dev->i2c_dev) {
        ESP_LOGE(TAG, "Dev->i2c_dev is NULL (new I2C driver requires device handle)");
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    if (!pdata || count == 0) {
        return VL53L0X_ERROR_NONE;
    }

    // Write register index, then read count bytes.
    esp_err_t ret = i2c_master_transmit_receive(
        Dev->i2c_dev,
        &index, 1,
        pdata, (size_t)count,
        pdMS_TO_TICKS(1000)
    );

    return esp_to_vl53l0x_error(ret);
}

/**
 * Write single byte register
 */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

/**
 * Write word register
 */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0x00FF);
    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

/**
 * Write double word (4 byte) register
 */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)((data & 0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data & 0x0000FF00) >> 8);
    buffer[3] = (uint8_t)(data & 0x000000FF);
    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

/**
 * Read single byte register
 */
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

/**
 * Read word (2byte) register
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error status;
    uint8_t buffer[2];

    status = VL53L0X_ReadMulti(Dev, index, buffer, 2);
    if (status != VL53L0X_ERROR_NONE) return status;

    *data = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return status;
}

/**
 * Read dword (4byte) register
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error status;
    uint8_t buffer[4];

    status = VL53L0X_ReadMulti(Dev, index, buffer, 4);
    if (status != VL53L0X_ERROR_NONE) return status;

    *data = ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8)  |
            ((uint32_t)buffer[3]);
    return status;
}

/**
 * Thread safe Update (read/modify/write) single byte register
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error status;
    uint8_t data;

    status = VL53L0X_RdByte(Dev, index, &data);
    if (status != VL53L0X_ERROR_NONE) return status;

    data = (data & AndData) | OrData;
    return VL53L0X_WrByte(Dev, index, data);
}

/**
 * @brief execute delay in all polling API call
 */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    vTaskDelay(1);
    return VL53L0X_ERROR_NONE;
}
