#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include "driver/spi_master.h"
// #include "driver/can.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN1 GPIO_NUM_32 // I2C SDA, UART RX, SPI MOSI, PWM, Digital
#define PIN2 GPIO_NUM_33 // I2C SCL, SPI MISO, Analog, Digital
#define PIN3 GPIO_NUM_18 // SPI SCLK, CAN_H
#define PIN4 GPIO_NUM_19 // CAN_L, SPI SS

static const char *TAG = "SignalDetection";
spi_device_handle_t spi_device;

// Function prototypes
void init_uart();
void init_adc();
void init_gpio();
void init_spi();
// void init_can();

bool detect_i2c();
bool detect_pwm();
// bool detect_analog();
bool detect_uart();
bool detect_spi();
// bool detect_can();
bool detect_digital();

void app_main(void) {
    init_uart();
    init_adc();
    init_gpio();
    init_spi();
    // init_can();

    while (1) {
        if (detect_i2c()) {
            ESP_LOGI(TAG, "I2C signal detected");
        } else if (detect_pwm()) {
            ESP_LOGI(TAG, "PWM signal detected");
        } // else if (detect_analog()) {
           // ESP_LOGI(TAG, "Analog signal detected");}
         else if (detect_uart()) {
            ESP_LOGI(TAG, "UART signal detected");
        } else if (detect_spi()) {
            ESP_LOGI(TAG, "SPI signal detected");
        } // else if (detect_can()) {
          //  ESP_LOGI(TAG, "CAN signal detected"); }
        else if (detect_digital()) {
            ESP_LOGI(TAG, "Digital signal detected");
        } else {
            ESP_LOGI(TAG, "No valid signal detected");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, PIN1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
}

void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0); // Use ADC_ATTEN_DB_0 for 0-1V range
}

void init_gpio() {
    gpio_set_direction(PIN1, GPIO_MODE_INPUT);
    gpio_set_direction(PIN2, GPIO_MODE_INPUT);
    gpio_set_direction(PIN3, GPIO_MODE_INPUT);
    gpio_set_direction(PIN4, GPIO_MODE_INPUT);
}

void init_spi() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN2,
        .mosi_io_num = PIN1,
        .sclk_io_num = PIN3,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // Clock out at 1 MHz
        .mode = 0,                 // SPI mode 0
        .spics_io_num = PIN4,      // CS pin
        .queue_size = 7,           // We want to be able to queue 7 transactions at a time
    };
    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 1));
    // Attach the device to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_device));
}

// can is not included in the edpidf library so will be added later
// void init_can() {
//     can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(PIN3, PIN4, CAN_MODE_NORMAL);
//     can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
//     can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
//     can_driver_install(&g_config, &t_config, &f_config);
//     can_start();
// }

bool detect_i2c() {
    // Simplified I2C detection logic
    // Note: This is a basic check and may not fully detect I2C signals accurately
    bool sda = gpio_get_level(PIN1);
    bool scl = gpio_get_level(PIN2);
    return sda && scl;
}

bool detect_pwm() {
    // PWM detection logic
    // Use a timer or pulse width measurement
    int high_time = gpio_get_level(PIN1) == 1;
    int low_time = gpio_get_level(PIN1) == 0;
    return high_time > 0 && low_time > 0;
}

// will be added back when I have the components neccessary to not fry my board
// bool detect_analog() {
//     int adc_value = adc1_get_raw(ADC1_CHANNEL_6); // GPIO34
//     float voltage = adc_value * (3.3 / 4095.0);
//     return voltage > 0.1;
// }

bool detect_uart() {
    uint8_t data;
    if (uart_read_bytes(UART_NUM_1, &data, 1, 100 / portTICK_PERIOD_MS) > 0) {
        return true;
    }
    return false;
}

bool detect_spi() {
    // SPI detection logic
    // Use the initialized SPI bus and check for SPI communication
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA,
        .length = 8 * 4,
        .rxlength = 0,
        .rx_buffer = NULL
    };
    if (spi_device_transmit(spi_device, &trans) == ESP_OK) {
        return true;
    }
    return false;
}

// bool detect_can() {
//     can_message_t message;
//     if (can_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
//         return true;
//     }
//     return false;
// }

bool detect_digital() {
    return gpio_get_level(PIN1) == 1;
}
