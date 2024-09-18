#include <string.h>             // For memset and memcpy
#include <stdint.h>             // For standard integer types
#include "sys/param.h"
#include "esp_err.h"            // For esp_err_t and ESP error codes
#include "esp_log.h"            // For logging (ESP_LOG macros)
#include "esp_intr_alloc.h"     // For interrupt allocation functions
#include "freertos/FreeRTOS.h"  // For FreeRTOS data types
#include "freertos/semphr.h"    // For FreeRTOS semaphore functionality
#include "freertos/queue.h"     // For FreeRTOS queue handling
#include "hal/i2c_hal.h"        // For I2C HAL functions and structures
#include "soc/i2c_periph.h"     // For accessing SOC-specific I2C details
#include "driver/gpio.h"        // For GPIO handling (SDA, SCL pin configuration)
#include "driver/i2c_peripheral.h" // For the i2c_peripheral API (your component header)






#if CONFIG_I2C_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif

#define I2C_PERIPHERAL_TIMEOUT_DEFAULT (32000) /* I2C slave timeout value, APB clock cycle number */
#define I2C_FIFO_EMPTY_THRESH_VAL_DEFAULT (SOC_I2C_FIFO_LEN / 2)
#define I2C_FIFO_FULL_THRESH_VAL_DEFAULT (SOC_I2C_FIFO_LEN / 2)

static const char *TAG = "i2c.peripheral";
static esp_err_t i2c_peripheral_bus_destroy(i2c_peripheral_dev_handle_t i2c_peripheral);




static IRAM_ATTR void s_peripheral_isr_handle_default(void *arg)
{
    i2c_peripheral_dev_t *i2c_peripheral = (i2c_peripheral_dev_t *)arg;
    i2c_hal_context_t *hal = &i2c_peripheral->base->hal;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // Used to track if a context switch is required
    uint32_t int_mask;

    // Get the interrupt mask and status
    i2c_ll_get_intr_mask(hal->dev, &int_mask);
    uint32_t status = hal->dev->status_reg.val;

    // Handle RX FIFO if there is data available
    uint32_t rx_fifo_cnt;
    i2c_ll_get_rxfifo_cnt(hal->dev, &rx_fifo_cnt);

    if (rx_fifo_cnt > 0) {
        if (i2c_peripheral->current_buffer) {
            // Calculate how much data we can read into the current buffer
            uint8_t *rxdata = i2c_peripheral->current_buffer->buffer + i2c_peripheral->current_buffer->length;
            uint32_t fifo_cnt_rd = MIN(sizeof(i2c_peripheral->current_buffer->buffer) - i2c_peripheral->current_buffer->length, rx_fifo_cnt);

            // Read data from RX FIFO into the buffer
            i2c_ll_read_rxfifo(hal->dev, rxdata, fifo_cnt_rd);
            i2c_peripheral->current_buffer->length += fifo_cnt_rd;
        } else {
            // No buffer is available, discard data and reset RX FIFO to prevent overflow
            i2c_ll_rxfifo_rst(hal->dev);
        }
    }

    // Handle filling the TX FIFO when a read request has been received
    if (!i2c_peripheral->tx_fill_done &&
        i2c_peripheral->current_buffer &&
        i2c_peripheral->current_buffer->length >= i2c_peripheral->address_size &&
        i2c_peripheral->callbacks.fill_tx_buffer) {

        uint8_t *tx_buffer = NULL;
        uint32_t tx_length = 0;

        // Call the user-defined callback to fill the TX buffer for the next read
        i2c_peripheral->callbacks.fill_tx_buffer(i2c_peripheral, i2c_peripheral->current_buffer,
                                                 i2c_peripheral->user_ctx, &tx_buffer, &tx_length);

        // If the callback provided data, write it to the TX FIFO
        if (tx_length > 0) {
            i2c_ll_write_txfifo(hal->dev, tx_buffer, tx_length);
        }
        i2c_peripheral->tx_fill_done = true;
    }

    // Handle transaction completion interrupt
    if (int_mask & I2C_TRANS_COMPLETE_INT_ENA) {
        if (i2c_peripheral->current_buffer) {
            // Determine if the transaction was a read or write operation
            i2c_peripheral->current_buffer->is_read = (status & (1 << 1)) ? true : false;

            // Queue the filled buffer for processing. Should not fail as the queue is large enough to hold all buffers
            xQueueSendFromISR(i2c_peripheral->filledBufferQueue, &i2c_peripheral->current_buffer, &xHigherPriorityTaskWoken)
        }

        // Retrieve the next available buffer from the free buffer queue
        i2c_peripheral_buffer_t *element = NULL;
        if (xQueueReceiveFromISR(i2c_peripheral->freeBufferQueue, &element, &xHigherPriorityTaskWoken) == pdTRUE) {
            i2c_peripheral->current_buffer = element;
            element->length = 0;  // Initialize the new buffer
            element->is_read = false;
        } else {
            // No buffer available, set current buffer to NULL
            i2c_peripheral->current_buffer = NULL;
        }

        // Reset the TX FIFO and clear the tx_fill_done flag
        i2c_peripheral->tx_fill_done = false;
        i2c_ll_txfifo_rst(hal->dev);
    }

    // Clear the interrupt mask after handling the interrupt
    i2c_ll_clear_intr_mask(hal->dev, int_mask);

    // If a higher priority task was woken, request a context switch
    portYIELD_FROM_ISR_ARG(xHigherPriorityTaskWoken);
}


#define I2C_INTR_ALL 0x0001FFFF
#define I2C_INTR_PERIPHERAL ( I2C_TRANS_COMPLETE_INT_ENA | I2C_RX_REC_FULL_INT_ST)


esp_err_t i2c_new_peripheral_device(const i2c_peripheral_config_t *peripheral_config, i2c_peripheral_dev_handle_t *ret_handle)
{

    esp_err_t ret = ESP_OK;
    i2c_peripheral_dev_t *i2c_peripheral = NULL;
    ESP_RETURN_ON_FALSE(peripheral_config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(peripheral_config->sda_io_num) && GPIO_IS_VALID_GPIO(peripheral_config->scl_io_num), ESP_ERR_INVALID_ARG, TAG, "invalid SDA/SCL pin number");

    int i2c_port_num = peripheral_config->i2c_port;
    i2c_peripheral = heap_caps_calloc(1, sizeof(i2c_peripheral_dev_t), I2C_MEM_ALLOC_CAPS);
    ESP_RETURN_ON_FALSE(i2c_peripheral, ESP_ERR_NO_MEM, TAG, "no memory for i2c peripheral bus");

    ESP_GOTO_ON_ERROR(i2c_acquire_bus_handle(i2c_port_num, &i2c_peripheral->base, I2C_BUS_MODE_SLAVE), err, TAG, "I2C bus acquire failed");


    i2c_peripheral->buffers = heap_caps_calloc(peripheral_config->buffer_pool_size, sizeof(i2c_peripheral_buffer_t), I2C_MEM_ALLOC_CAPS);
    ESP_RETURN_ON_FALSE(i2c_peripheral->buffers, ESP_ERR_NO_MEM, TAG, "no memory for i2c peripheral buffers");

    memset(i2c_peripheral->buffers, 0, peripheral_config->buffer_pool_size * sizeof(i2c_peripheral_buffer_t));
    
    // Create queues for buffer management
    i2c_peripheral->freeBufferQueue = xQueueCreate(peripheral_config->buffer_pool_size, sizeof(i2c_peripheral_buffer_t *));
    if (i2c_peripheral->freeBufferQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create freeBufferQueue");
        goto err; // Ensure you handle errors
    }

    i2c_peripheral->filledBufferQueue = xQueueCreate(peripheral_config->buffer_pool_size, sizeof(i2c_peripheral_buffer_t * *));
    if (i2c_peripheral->filledBufferQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create filledBufferQueue");
        goto err; // Ensure you handle errors
    }

    i2c_peripheral->current_buffer = NULL;
    i2c_peripheral->tx_fill_done = false;
    i2c_peripheral->address_size = peripheral_config->register_addr_size;
    // Initialize the free buffer pool
    for (int32_t i = 0; i < peripheral_config->buffer_pool_size; i++)
    {
        i2c_peripheral_buffer_t * element = &i2c_peripheral->buffers[i];
        if (i == 0)
        {
            i2c_peripheral->current_buffer = element;
        }
        else
        {
            xQueueSend(i2c_peripheral->freeBufferQueue, &element, portMAX_DELAY);
        }
      
    }
  

    i2c_hal_context_t *hal = &i2c_peripheral->base->hal;
    i2c_peripheral->base->scl_num = peripheral_config->scl_io_num;
    i2c_peripheral->base->sda_num = peripheral_config->sda_io_num;

    ESP_GOTO_ON_ERROR(i2c_common_set_pins(i2c_peripheral->base), err, TAG, "i2c peripheral set pins failed");

    // Setup interrupts
    int isr_flags = I2C_INTR_ALLOC_FLAG;
    if (peripheral_config->intr_priority)
    {
        isr_flags |= 1 << (peripheral_config->intr_priority);
    }
    i2c_ll_clear_intr_mask(hal->dev, I2C_INTR_ALL);
    i2c_ll_disable_intr_mask(hal->dev, I2C_INTR_ALL);
    
    ret = esp_intr_alloc_intrstatus(i2c_periph_signal[i2c_port_num].irq, isr_flags, (uint32_t)i2c_ll_get_interrupt_status_reg(hal->dev), I2C_INTR_PERIPHERAL, s_peripheral_isr_handle_default, i2c_peripheral, &i2c_peripheral->base->intr_handle);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "install i2c peripheral interrupt failed");

    
    // Clear all interrupt raw bits before enable, avoid previous bus data affects interrupt.
    portENTER_CRITICAL(&i2c_peripheral->base->spinlock);
    i2c_hal_slave_init(hal);

#if SOC_I2C_SLAVE_SUPPORT_I2CRAM_ACCESS
    if (i2c_slave->fifo_mode == I2C_SLAVE_NONFIFO)
    {
        i2c_ll_slave_set_fifo_mode(hal->dev, false);
        i2c_ll_enable_mem_access_nonfifo(hal->dev, true);
    }
    else
    {
        i2c_ll_slave_set_fifo_mode(hal->dev, true);
        i2c_ll_enable_mem_access_nonfifo(hal->dev, false);
    }
#endif

    // Default, we enable hardware filter
    I2C_CLOCK_SRC_ATOMIC()
    {
        i2c_ll_set_source_clk(hal->dev, peripheral_config->clk_source);
    }
    // bool addr_10bit_en = slave_config->addr_bit_len != I2C_ADDR_BIT_LEN_7;
    i2c_ll_set_slave_addr(hal->dev, peripheral_config->peripheral_addr, false);
#if SOC_I2C_SLAVE_SUPPORT_BROADCAST
    i2c_ll_slave_broadcast_enable(hal->dev, peripheral_config->flags.broadcast_en);
#endif
    i2c_ll_set_txfifo_empty_thr(hal->dev, 0);
    i2c_ll_set_rxfifo_full_thr(hal->dev, peripheral_config->register_addr_size);
    // set timing for data
    i2c_ll_set_sda_timing(hal->dev, 10, 10);
    i2c_ll_set_tout(hal->dev, I2C_PERIPHERAL_TIMEOUT_DEFAULT);

#if SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
    i2c_ll_slave_enable_scl_stretch(hal->dev, slave_config->flags.stretch_en);
#endif
    // i2c_ll_slave_tx_auto_start_en(hal->dev, true);

    i2c_ll_update(hal->dev);

    portEXIT_CRITICAL(&i2c_peripheral->base->spinlock);


    i2c_ll_rxfifo_rst(hal->dev);
    i2c_ll_txfifo_rst(hal->dev);

    i2c_ll_enable_intr_mask(hal->dev, I2C_INTR_PERIPHERAL);
    *ret_handle = i2c_peripheral;
    ESP_LOGI("I2C", "I2C peripheral initialized successfully");
    return ESP_OK;

err:
    if (i2c_peripheral)
    {
        i2c_peripheral_bus_destroy(i2c_peripheral);
    }
    return ret;
}

static esp_err_t i2c_peripheral_bus_destroy(i2c_peripheral_dev_handle_t i2c_peripheral)
{
    if (i2c_peripheral)
    {
        // Disable interrupts for peripheral mode
        i2c_ll_disable_intr_mask(i2c_peripheral->base->hal->dev, I2C_INTR_PERIPHERAL);

        // Release the I2C bus handle
        i2c_release_bus_handle(i2c_peripheral->base);
    }

    // Free the peripheral structure
    free(i2c_peripheral);

    return ESP_OK;
}

esp_err_t i2c_peripheral_register_event_callbacks(i2c_peripheral_dev_handle_t i2c_peripheral, const i2c_peripheral_event_callbacks_t *cbs, void *user_data)
{
    ESP_RETURN_ON_FALSE(i2c_peripheral != NULL, ESP_ERR_INVALID_ARG, TAG, "i2c peripheral handle not initialized");
    ESP_RETURN_ON_FALSE(cbs, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

#if CONFIG_I2C_ISR_IRAM_SAFE
#if SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
    if (cbs->on_stretch_occur)
    {
        ESP_RETURN_ON_FALSE(esp_ptr_in_iram(cbs->on_stretch_occur), ESP_ERR_INVALID_ARG, TAG, "i2c stretch occur callback not in IRAM");
    }
#endif // SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
    if (cbs->on_recv_done)
    {
        ESP_RETURN_ON_FALSE(esp_ptr_in_iram(cbs->on_recv_done), ESP_ERR_INVALID_ARG, TAG, "i2c receive done callback not in IRAM");
    }
    if (user_data)
    {
        ESP_RETURN_ON_FALSE(esp_ptr_internal(user_data), ESP_ERR_INVALID_ARG, TAG, "user context not in internal RAM");
    }
#endif // CONFIG_I2C_ISR_IRAM_SAFE

    memcpy(&(i2c_peripheral->callbacks), cbs, sizeof(cbs));
    i2c_peripheral->user_ctx = user_data;
    return ESP_OK;
}
QueueHandle_t i2c_peripheral_get_filled_queue_handle(i2c_peripheral_dev_handle_t i2c_peripheral)
{
    return i2c_peripheral->filledBufferQueue;
}
QueueHandle_t i2c_peripheral_return_queue_handle(i2c_peripheral_dev_handle_t i2c_peripheral)
{
    return i2c_peripheral->freeBufferQueue;
}
