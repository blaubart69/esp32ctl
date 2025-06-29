#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"


const uint8_t  host_digitalWriteZero        = 1;
const uint8_t  host_digitalWriteOne         = 2;
const uint8_t  host_digiRead                = 3;
const uint8_t  host_pinModeInput            = 4;
const uint8_t  host_pinModeInputPullup      = 5;
const uint8_t  host_pinModeInputPulldown    = 6;
const uint8_t  host_pinModeOutput           = 7;


const uint8_t client_digitalReadZero        = 1;
const uint8_t client_digitalReadOne         = 2;
const uint8_t client_error                  = 0xFF;

struct host_cmd_t {
    uint8_t cmd;
    uint8_t pin;
};

static QueueHandle_t blink_queue;

//----------------------------------------------------------------------------
static void vTaskBlinker(void* pvParameters )
//----------------------------------------------------------------------------
{
    #define ESP32_SUPER_MINI_BUILDIN_LED 8

    gpio_set_direction(ESP32_SUPER_MINI_BUILDIN_LED, GPIO_MODE_OUTPUT); // ESP32 super mini buildin LED
    gpio_set_level(ESP32_SUPER_MINI_BUILDIN_LED,1); // turn off

    QueueHandle_t blink_queue = (QueueHandle_t)pvParameters;
    for( ;; )
    {
        char message;
        if( xQueueReceive( blink_queue, &( message ), ( TickType_t ) pdMS_TO_TICKS(10000)) )  
        {
            gpio_set_level(ESP32_SUPER_MINI_BUILDIN_LED,0);

            TickType_t delay;
            if ( message == 'e' ) {
                delay = 2000;
            } 
            else if ( message == 'o' ) {
                delay = 50;
            }
            else {
                delay = 5000;
            }
            vTaskDelay(pdMS_TO_TICKS(delay));

            gpio_set_level(ESP32_SUPER_MINI_BUILDIN_LED,1);
        }
    }
}
//----------------------------------------------------------------------------
static struct host_cmd_t decode_command(const uint8_t encoded_cmd) 
//----------------------------------------------------------------------------
{
    struct host_cmd_t cmd = {
         .cmd = (uint8_t)(encoded_cmd >> 5)
        ,.pin = (uint8_t)(encoded_cmd & 0x1F)
    };

    return cmd;
}
//----------------------------------------------------------------------------
static uint8_t encode_result(const uint8_t cmd, const uint8_t pin) 
//----------------------------------------------------------------------------    
{
    return (uint8_t)(cmd << 5) | (uint8_t)( pin & 0x1F);
}
//----------------------------------------------------------------------------
static uint8_t get_client_response_type(const int level) 
//----------------------------------------------------------------------------    
{
    uint8_t result;
    if ( level == 0 ) {
        result = client_digitalReadZero;
    }
    else if ( level == 1 ){
        result = client_digitalReadOne;
    }
    else {
        result = client_error;
    }
    return result;
}
//----------------------------------------------------------------------------
static uint8_t handle_digitalRead(const uint8_t pin)
//----------------------------------------------------------------------------    
{
    int     level           = gpio_get_level(pin);
    uint8_t client_result   = get_client_response_type(level);
    uint8_t response        = encode_result(client_result, pin);
    return response;
}

//----------------------------------------------------------------------------
static bool is_valid_pin_for_esp32_super_mini(const uint8_t pin)
//----------------------------------------------------------------------------    
{
    return
           pin <= 10
        || pin == 20
        || pin == 21 ; 
}

//----------------------------------------------------------------------------
static bool handle_command(const uint8_t rawcmd, uint8_t* response)
//----------------------------------------------------------------------------    
{   
    struct host_cmd_t cmd = decode_command(rawcmd);

    if ( ! is_valid_pin_for_esp32_super_mini(cmd.pin) ) {
        return false;
    }

    *response = rawcmd;

    switch (cmd.cmd)
    {
        default: return false;

        case host_digitalWriteOne:        gpio_set_level(cmd.pin,1);                          break;
        case host_digitalWriteZero:       gpio_set_level(cmd.pin,0);                          break;
        case host_pinModeInput:           gpio_set_direction(cmd.pin, GPIO_MODE_INPUT);       break;
        
        case host_pinModeInputPullup:     gpio_set_direction(cmd.pin, GPIO_MODE_INPUT);  
                                          gpio_set_pull_mode(cmd.pin, GPIO_PULLUP_ENABLE);    break;
        
        case host_pinModeInputPulldown:   gpio_set_direction(cmd.pin, GPIO_MODE_INPUT);        
                                          gpio_set_pull_mode(cmd.pin, GPIO_PULLDOWN_ENABLE);  break;

        case host_pinModeOutput:          gpio_set_direction(cmd.pin, GPIO_MODE_OUTPUT);      break;
        case host_digiRead:               *response = handle_digitalRead(cmd.pin);            break;
    }

    return true;
}
//----------------------------------------------------------------------------
static void onCommand( const uint8_t buf[], const size_t len, uint8_t dataout[] )
//----------------------------------------------------------------------------
{
    for (int i=0; i<len; i++) {
        //uint8_t response;
        char blink;

        if ( ! handle_command(buf[i], &(dataout[i])) ) {
            dataout[i] = 0xFF;
            blink = 'e';
        } 
        else {
            blink = 'o';
        }
        if ( xQueueSend(blink_queue, (void*)&blink, 0) == errQUEUE_FULL ) {
            // we don't care
        }
    }
}

#define BUF_SIZE (128)
#define ECHO_TASK_STACK_SIZE (4096)

static void main_loop(void *arg)
{
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    /*
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE("usb_serial_jtag echo", "no memory for data");
        return;
    }

    uint8_t *dataout = (uint8_t *) malloc(BUF_SIZE);
    if (dataout == NULL) {
        ESP_LOGE("usb_serial_jtag echo", "no memory for dataout");
        return;
    }*/
    static uint8_t data_in [BUF_SIZE];
    static uint8_t data_out[BUF_SIZE];

    while (1) {
        const int len = usb_serial_jtag_read_bytes(data_in, BUF_SIZE, 2000 / portTICK_PERIOD_MS);
        if (len) {
            onCommand(data_in, len, data_out);
            usb_serial_jtag_write_bytes(data_out, len, 20 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    blink_queue = xQueueCreate(3, sizeof(char));

    xTaskCreate(vTaskBlinker, "Blinker", 4096, blink_queue, tskIDLE_PRIORITY, NULL);
    xTaskCreate(main_loop, "esp32Ctl", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}