#include "spi_master.h"
// #include "wait.h"
// #include "util.h"
#include <ctype.h>

// #define _delay_us nrf_delay_us
#define SHARPMEM_BIT_WRITECMD (0x01) // 0x80 in LSB format
#define SHARPMEM_BIT_VCOM (0x02)     // 0x40 in LSB format
#define SHARPMEM_BIT_CLEAR (0x04)    // 0x20 in LSB format

#define WIDTH 160
#define HEIGHT 68

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif
#ifndef _swap_uint16_t
#define _swap_uint16_t(a, b)                                                   \
  {                                                                            \
    uint16_t t = a;                                                            \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

uint16_t rotation = 0;
uint8_t *sharpmem_buffer;
uint8_t _sharpmem_vcom;

#define TOGGLE_VCOM                                                            \
  do {                                                                         \
    _sharpmem_vcom = _sharpmem_vcom ? 0x00 : SHARPMEM_BIT_VCOM;                \
  } while (0);


enum { SPI_MODE0, SPI_MODE1, SPI_MODE2, _SPI_MODE4 };
#define SPI_BITORDER_LSBFIRST 0
int _dataOrder = 0;
uint32_t _freq = 2000000;
uint8_t _dataMode = SPI_MODE0;
int8_t _mosi = SPI_MOSI_PIN, _sck = SPI_SCK_PIN;
int8_t _miso = -1;

void SHARP_begin(void) {
    // gpio_write_pin_high(SPI_SCK_PIN);
    gpio_set_pin_output(OLED_CS_PIN);
    gpio_write_pin_high(OLED_CS_PIN);

    gpio_set_pin_output(_sck);
    gpio_write_pin_low(_sck);

    gpio_set_pin_output(_mosi);
    gpio_write_pin_high(_mosi);
    // spi_start();

    gpio_write_pin_low(OLED_CS_PIN);
    _sharpmem_vcom = SHARPMEM_BIT_VCOM;
    sharpmem_buffer = (uint8_t *)malloc((WIDTH * HEIGHT) / 8);
}

void software_spi_transmit(uint8_t* buffer, size_t len) {
    //
  // SOFTWARE SPI
  //
  uint8_t startbit;
  if (_dataOrder == SPI_BITORDER_LSBFIRST) {
    startbit = 0x1;
  } else {
    startbit = 0x80;
  }

  bool towrite, lastmosi = !(buffer[0] & startbit);
  uint8_t bitdelay_us = (1000000 / _freq) / 2;
    bitdelay_us = 1;

  for (size_t i = 0; i < len; i++) {
    uint8_t reply = 0;
    uint8_t send = buffer[i];

    /*
    Serial.print("\tSending software SPI byte 0x");
    Serial.print(send, HEX);
    Serial.print(" -> 0x");
    */

    // Serial.print(send, HEX);
    for (uint8_t b = startbit; b != 0;
         b = (_dataOrder == SPI_BITORDER_LSBFIRST) ? b << 1 : b >> 1) {

      if (bitdelay_us) {
              chThdSleepMicroseconds(bitdelay_us);
      }

      if (_dataMode == SPI_MODE0 || _dataMode == SPI_MODE2) {
        towrite = send & b;
        if ((_mosi != -1) && (lastmosi != towrite)) {
            if (towrite) gpio_write_pin_high(_mosi);
                    else gpio_write_pin_low(_mosi);
          lastmosi = towrite;
        }

       gpio_write_pin_high(_sck); // digitalWrite(_sck, HIGH);

        if (bitdelay_us) { chThdSleepMicroseconds(bitdelay_us); }

        if (_miso != -1) {
          if (gpio_read_pin(_miso)) {
            reply |= b;
          }
        }

       gpio_write_pin_low(_sck);// digitalWrite(_sck, LOW);
        if (bitdelay_us) { chThdSleepMicroseconds(bitdelay_us); } // try
      } else { // if (_dataMode == SPI_MODE1 || _dataMode == SPI_MODE3)

        gpio_write_pin_high(_sck);

        if (bitdelay_us) { chThdSleepMicroseconds(bitdelay_us); }

        if (_mosi != -1) {
                    if (send&b) gpio_write_pin_high(_mosi);
                    else gpio_write_pin_low(_mosi);
        }

      gpio_write_pin_low(_sck);

        if (_miso != -1) {
          if (gpio_read_pin(_miso)) {
            reply |= b;
          }
        }
      }
      if (_miso != -1) {
        buffer[i] = reply;
      }
    }
  }
}

void software_spi_write(uint8_t v) {
    software_spi_transmit(&v, 1);
}

int16_t _width = WIDTH, _height = HEIGHT;
#define DIVISOR 48
// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t PROGMEM set[] = {1, 2, 4, 8, 16, 32, 64, 128},
                             clr[] = {(uint8_t)~1,  (uint8_t)~2,  (uint8_t)~4,
                                      (uint8_t)~8,  (uint8_t)~16, (uint8_t)~32,
                                      (uint8_t)~64, (uint8_t)~128};


void drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
    return;

  switch (rotation) {
  case 1:
    _swap_int16_t(x, y);
    x = WIDTH - 1 - x;
    break;
  case 2:
    x = WIDTH - 1 - x;
    y = HEIGHT - 1 - y;
    break;
  case 3:
    _swap_int16_t(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  if (color) {
    sharpmem_buffer[(y * WIDTH + x) / 8] |= pgm_read_byte(&set[x & 7]);
  } else {
    sharpmem_buffer[(y * WIDTH + x) / 8] &= pgm_read_byte(&clr[x & 7]);
  }
}


void SHARP_clear(void) {
     memset(sharpmem_buffer, 0xff, (WIDTH * HEIGHT) / 8);

    // spi_start(OLED_CS_PIN, true, 0, DIVISOR);
  // Send the clear screen command rather than doing a HW refresh (quicker)
  gpio_write_pin_high(OLED_CS_PIN);
              chThdSleepMicroseconds(1);
  uint8_t clear_data[2] = {(uint8_t)(_sharpmem_vcom | SHARPMEM_BIT_CLEAR),
                           0x00};
  software_spi_transmit(clear_data, 2);

  TOGGLE_VCOM;
              chThdSleepMicroseconds(1);
  gpio_write_pin_low(OLED_CS_PIN);
    // spi_stop();
}

void SHARP_refesh(void) {
    uint16_t i, currentline;

    // spi_start(OLED_CS_PIN, true, 0, DIVISOR);
  // Send the write command
  gpio_write_pin_high(OLED_CS_PIN);

    software_spi_write(_sharpmem_vcom | SHARPMEM_BIT_WRITECMD);
  TOGGLE_VCOM;
              chThdSleepMicroseconds(1);

  uint8_t bytes_per_line = WIDTH / 8;
  uint16_t totalbytes = (WIDTH * HEIGHT) / 8;

  for (i = 0; i < totalbytes; i += bytes_per_line) {
    uint8_t line[bytes_per_line + 2];

    // Send address byte
    currentline = ((i + 1) / (WIDTH / 8)) + 1;
    line[0] = currentline;
    // copy over this line
    memcpy(line + 1, sharpmem_buffer + i, bytes_per_line);
    // Send end of line
    line[bytes_per_line + 1] = 0x00;
    // send it!
    software_spi_transmit(line, bytes_per_line + 2);
  }

              chThdSleepMicroseconds(1);
  // Send another trailing 8 bits for the last line
  software_spi_write(0x00);
              chThdSleepMicroseconds(1);
  gpio_write_pin_low(OLED_CS_PIN);
    // spi_stop();
}

void fuck(void) {
    // spi_init();
    // gpio_set_pin_output(OLED_CS_PIN);
    // gpio_set_pin_output(_mosi);
    // gpio_set_pin_output(_sck);
    // gpio_write_pin_low(_sck);
    // gpio_write_pin_low(OLED_CS_PIN);
    SHARP_begin();
    // SHARP_clear();
    for (int i = 0; i < WIDTH; i++) {
        for (int j = 0; j < HEIGHT; j++) {
            if ((i/10 + j/10) % 2) {
            drawPixel(i, j, 0); // black
            } else {
            drawPixel(i, j, 1); // white
            }
        }
    }
    SHARP_refesh();
}

