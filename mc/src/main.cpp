#include <Arduino.h>

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
CLIENT_RESPONSE get_client_response(int value) 
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
void handle_digitalRead(const uint8_t pin)
//----------------------------------------------------------------------------    
{
    uint8_t value           = digitalRead(pin);
    uint8_t client_result   = get_client_response(value);
    uint8_t reply           = encode_result(client_result, pin);
    if ( Serial.write(reply) != 1 ) {
        // problem!
    }
}
//----------------------------------------------------------------------------
bool handle_command(const host_cmd_t cmd)
//----------------------------------------------------------------------------    
{   
    switch (cmd.cmd)
    {
        default: return false;
        case HOST_CMD::digitalWriteHigh:       digitalWrite(cmd.pin, HIGH);                  break;
        case HOST_CMD::digitalWriteLow:        digitalWrite(cmd.pin, LOW);                   break;
        case HOST_CMD::pinModeInput:           pinMode     (cmd.pin, INPUT);                 break;
        case HOST_CMD::pinModeInputPullup:     pinMode     (cmd.pin, INPUT_PULLUP);          break;
        case HOST_CMD::pinModeInputPulldown:   pinMode     (cmd.pin, INPUT_PULLDOWN);        break;
        case HOST_CMD::pinModeOutput:          pinMode     (cmd.pin, OUTPUT);                break;
        case HOST_CMD::digiRead:               handle_digitalRead(cmd.pin);                  break;
    }
    return true;
}
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
        uint8_t data;
        if ( Serial.read(&data, 1) != 1 ) {
            // problem!
        }
        else {
            host_cmd_t cmd = decode_command(data);
            if ( ! handle_command(cmd) ) {
                Serial.write(CLIENT_RESPONSE::error);
            } 
            else if ( cmd.cmd == HOST_CMD::digiRead ) {
                // do not send an ACK
            }
            else {
                // send back the received data as an ACK
                Serial.write(data);      
            }
        }
    }
}
//----------------------------------------------------------------------------
void setup()
//----------------------------------------------------------------------------
{
    Serial.begin(230400);
    pinMode(8,OUTPUT); // build in LED
    Serial.onEvent( ARDUINO_HW_CDC_RX_EVENT, onRxEvent );
}

// the loop function runs over and over again forever
void loop() {
    vTaskDelete(nullptr);
}