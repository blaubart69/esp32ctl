#include <Arduino.h>

#include <functional>

/*
*   Let's try ONE byte.
*   we have  > 16 GPIO pin NUMBERS on ESP32 super mini 
*   we have <=  8 commands
*   1 byte
*     3 bits command        8 commands
*     5 bits pin number    32 pins
*/

enum HOST_CMD {
    digitalWriteHigh = 1,
    digitalWriteLow = 2,
    digiRead = 3,
    pinModeInput = 4,
    pinModeInputPullup = 5,
    pinModeInputPulldown = 6,
    pinModeOutput = 7,
};

enum CLIENT_RESPONSE {
    digitalReadHigh = 1,
    digitalReadLow = 2,
    error = 0xFF
};

typedef struct {
    uint8_t cmd;
    uint8_t pin;
} host_cmd_t;

//----------------------------------------------------------------------------
void vTaskBlinker(void* pvParameters )
//----------------------------------------------------------------------------
{
    #define STATUS_LED 8
    pinMode(STATUS_LED, OUTPUT);     // ESP32 super mini buildin LED

    QueueHandle_t blink_queue = (QueueHandle_t)pvParameters;
    for( ;; )
    {
        char message;
        if( xQueueReceive( blink_queue, &( message ), ( TickType_t ) pdMS_TO_TICKS(10000)) )  
        {
            digitalWrite(STATUS_LED, LOW);

            TickType_t delay;
            if ( message == 'e' ) {
                delay = pdMS_TO_TICKS(2000);
            } 
            else if ( message == 'o' ) {
                delay = pdMS_TO_TICKS(50);
            }
            vTaskDelay(delay);

            digitalWrite(STATUS_LED, HIGH);
        }
    }
}
//----------------------------------------------------------------------------
host_cmd_t decode_command(const uint8_t encoded_cmd) 
//----------------------------------------------------------------------------
{
    return host_cmd_t {
         .cmd = (uint8_t)(encoded_cmd >> 5)
        ,.pin = (uint8_t)(encoded_cmd & 0x1F)
    };
}
//----------------------------------------------------------------------------
uint8_t encode_result(const uint8_t cmd, const uint8_t pin) 
//----------------------------------------------------------------------------    
{
    return (uint8_t)(cmd << 5) | (uint8_t)( pin & 0x1F);
}
//----------------------------------------------------------------------------
CLIENT_RESPONSE get_client_response_type(int value) 
//----------------------------------------------------------------------------    
{
    CLIENT_RESPONSE result;
    if ( value == HIGH ) {
        result = CLIENT_RESPONSE::digitalReadHigh;
    }
    else if ( value == LOW ){
        result = CLIENT_RESPONSE::digitalReadLow;
    }
    else {
        result = CLIENT_RESPONSE::error;
    }
    return result;
}
//----------------------------------------------------------------------------
uint8_t handle_digitalRead(const uint8_t pin)
//----------------------------------------------------------------------------    
{
    uint8_t value           = digitalRead(pin);
    uint8_t client_result   = get_client_response_type(value);
    uint8_t response        = encode_result(client_result, pin);
    return response;
}
//----------------------------------------------------------------------------
bool handle_command(const uint8_t rawcmd, uint8_t* response)
//----------------------------------------------------------------------------    
{   
    host_cmd_t cmd = decode_command(rawcmd);

    switch (cmd.cmd)
    {
        default: return false;
        case HOST_CMD::digitalWriteHigh:           digitalWrite      (cmd.pin, HIGH);             *response = rawcmd;     break;
        case HOST_CMD::digitalWriteLow:            digitalWrite      (cmd.pin, LOW);              *response = rawcmd;     break;
        case HOST_CMD::pinModeInput:               pinMode           (cmd.pin, INPUT);            *response = rawcmd;     break;
        case HOST_CMD::pinModeInputPullup:         pinMode           (cmd.pin, INPUT_PULLUP);     *response = rawcmd;     break;
        case HOST_CMD::pinModeInputPulldown:       pinMode           (cmd.pin, INPUT_PULLDOWN);   *response = rawcmd;     break;
        case HOST_CMD::pinModeOutput:              pinMode           (cmd.pin, OUTPUT);           *response = rawcmd;     break;
        case HOST_CMD::digiRead: uint8_t tmpRead = handle_digitalRead(cmd.pin);                   *response = tmpRead;    break;
    }
    return true;
}

static QueueHandle_t blink_queue;

//----------------------------------------------------------------------------
void onRxEvent(
//----------------------------------------------------------------------------
  void*               event_handler_arg
  ,esp_event_base_t   event_base
  ,int32_t            event_id
  ,void*              event_data) 
{
    int available;
    while ( (available = Serial.available()) != 0 )
    {
        uint8_t rawcmd;
        if ( Serial.read(&rawcmd, 1) != 1 ) {
            // problem!
        }
        else {
            uint8_t response;
            char blink;

            if ( ! handle_command(rawcmd, &response) ) {
                Serial.write(CLIENT_RESPONSE::error);
                blink = 'e';
            } 
            else if ( Serial.write(response) != 1 ) {
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
}

//----------------------------------------------------------------------------
void setup()
//----------------------------------------------------------------------------
{
    Serial.begin(230400);

    blink_queue = xQueueCreate(3, sizeof(char));
    Serial.onEvent( ARDUINO_HW_CDC_RX_EVENT, onRxEvent );
    
    TaskHandle_t task_handle;
    xTaskCreate(
        vTaskBlinker
        , "Blinker"
        , 512   // stack
        , blink_queue
        , tskIDLE_PRIORITY
        , &task_handle );
}

// the loop function runs over and over again forever
void loop() {
    vTaskDelete(nullptr);
}