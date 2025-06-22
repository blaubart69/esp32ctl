// C library headers
#include <cstdio>
#include <cstring>  // strerror
#include <cstdint>



// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <chrono>
#include <thread>

typedef struct {
    uint8_t cmd;
    uint8_t pin;
} host_cmd_t;

//----------------------------------------------------------------------------
host_cmd_t unpack(const uint8_t encoded_cmd) 
//----------------------------------------------------------------------------
{
    return host_cmd_t {
         .cmd = (uint8_t)(encoded_cmd >> 5)
        ,.pin = (uint8_t)(encoded_cmd & 0x1F)
    };
}
//----------------------------------------------------------------------------
uint8_t pack(const uint8_t cmd, const uint8_t pin) 
//----------------------------------------------------------------------------    
{
    return (uint8_t)(cmd << 5) | (uint8_t)( pin & 0x1F);
}

void msg_loop(int serial_port) {
    for(;;) {
        printf("cmd pin? ");
        int cmd, pin;
        if ( scanf("%u %u", &cmd,&pin) != 2 ) {
            printf("E: could not parse input\n");
            {
                // consume all garbage input
                int c;
                while ( (c = getchar()) != '\n' && c != EOF ) { }
            }
        }
        else {
            uint8_t payload = pack(cmd,pin);

            auto start = std::chrono::steady_clock::now();

            if ( write(serial_port, &payload, sizeof(payload)) != 1 ) {
                printf("E: write != 1\n");
            }
            else {
                uint8_t response;
                int num_bytes = read(serial_port, &response, sizeof(response));

                auto end = std::chrono::steady_clock::now();

                if (num_bytes != sizeof(response)) {
                    printf("W: read only %d bytes\n", num_bytes);
                }
                else {
                    auto roundtrip_ms = std::chrono::duration <double,std::milli> (end-start).count();

                    host_cmd_t resp = unpack(response);
                    printf("I: req: %02X -> response (roundtrip: %.3f ms): %02X => %d %d\n", payload, roundtrip_ms, response, resp.cmd, resp.pin);
                }
            }
        }
    }
}

void autofire(int serial_port) {
    const uint8_t payload = 0x60;
    uint8_t response;
    size_t read_err = 0;
    size_t success = 0;
    size_t write_err = 0;
    int l=0;
    for(;;l++) {

        //std::this_thread::sleep_for(std::chrono::milliseconds(10));

        int num_bytes;
        ssize_t written;
        if ( (written=write(serial_port, &payload, 1)) != 1 ) {
            perror("write");
            printf("Error %i from write: %s\n", errno, strerror(errno));
            write_err++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else if ( (num_bytes = read(serial_port, &response, sizeof(response))) != 1 ) {
            read_err++;
        }
        else {
            success++;
        }
        if ( l > 10000) {
            l=0;
            printf("success: %lu, read_err: %lu, write_err: %lu\n", success, read_err, write_err);
        }
    }
}

int main(int argc, char* argv[]) {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/esp32mini", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B230400);
  cfsetospeed(&tty, B230400);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  if ( argc == 2 && strcmp("a", argv[1]) == 0 ) {
    autofire(serial_port);
  }
  else {
    msg_loop(serial_port);
  }

  /*

  // Write to serial port
  unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
  write(serial_port, msg, sizeof(msg));

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [256];

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return 1;
  }

  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
    */

  close(serial_port);
  return 0; // success
};