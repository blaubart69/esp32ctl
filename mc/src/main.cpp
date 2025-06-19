#include <Arduino.h>

/*
*   3 bytes
*     1: API
*     2: port
*     3: value 
*
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

enum CLIENT_RESULT {
    digitalReadHigh = 1,
    digitalReadLow = 2
};


typedef struct host_cmd_t {
    uint8_t cmd;
    uint8_t pin;
};

host_cmd_t decode_command(const uint8_t encoded_cmd) {
    return host_cmd_t {
         .cmd = (uint8_t)(encoded_cmd >> 5)
        ,.pin = (uint8_t)(encoded_cmd & 0x1F)
    };
}

uint8_t encode_result(const uint8_t cmd, const uint8_t pin) {
    return (uint8_t)(cmd << 5) | (uint8_t)( pin & 0x1F);
}

uint8_t get_client_result(int value) {
    uint8_t result;
    if ( value == HIGH ) {
        result = CLIENT_RESULT::digitalReadHigh;
    }
    else if ( value == LOW ){
        result = CLIENT_RESULT::digitalReadLow;
    }
    else {
        result = 0xFF;
    }
    return result;
}

void handle_command(const host_cmd_t cmd) {
    switch (cmd.cmd)
    {
    case HOST_CMD::digitalWriteHigh:       digitalWrite(cmd.pin, HIGH);             break;
    case HOST_CMD::digitalWriteLow:        digitalWrite(cmd.pin, LOW);              break;
    case HOST_CMD::pinModeInput:           pinMode     (cmd.pin, INPUT);            break;
    case HOST_CMD::pinModeInputPullup:     pinMode     (cmd.pin, INPUT_PULLUP);     break;
    case HOST_CMD::pinModeInputPulldown:   pinMode     (cmd.pin, INPUT_PULLDOWN);   break;
    case HOST_CMD::digiRead:
            uint8_t value = digitalRead(cmd.pin);
            uint8_t client_result = get_client_result(value);
            uint8_t reply = encode_result(client_result, cmd.pin);
            if ( Serial.write(reply) != 1 ) {
                // problem!
            }
        break;
    default:
        // problem!
        break;
    }
}

void  onRxEvent(
  void*               event_handler_arg
  ,esp_event_base_t   event_base
  ,int32_t            event_id
  ,void*              event_data) 
{
    int available;
    while ( (available = Serial.available()) != 0 )
    {
        byte data;
        if ( Serial.read(&data, 1) != 1 ) {
            // problem!
        }
        else {
            host_cmd_t cmd = decode_command(data);
            handle_command(cmd);
        }
    }
}

void setup() {
  Serial.begin(230400);
  pinMode(8,OUTPUT); // build in LED
  Serial.onEvent( ARDUINO_HW_CDC_RX_EVENT, onRxEvent );
}

// the loop function runs over and over again forever
void loop() {
    vTaskDelete(nullptr);
}