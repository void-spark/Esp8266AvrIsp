#ifndef _ESP_AVRISP_H
#define _ESP_AVRISP_H

#include <Arduino.h>
#include <ESP8266WiFi.h>

// uncomment if you use an n-mos to level-shift the reset line
// #define AVRISP_ACTIVE_HIGH_RESET

// SPI clock frequency in Hz
#define AVRISP_SPI_FREQ   125e3

// programmer states
typedef enum {
    AVRISP_STATE_IDLE = 0,    // no active TCP session
    AVRISP_STATE_PENDING,     // TCP connected, pending SPI activation
    AVRISP_STATE_ACTIVE       // programmer is active and owns the SPI bus
} AVRISPState_t;

// stk500 parameters
typedef struct {
    uint8_t devicecode;
    uint8_t revision;
    uint8_t progtype;
    uint8_t parmode;
    uint8_t polling;
    uint8_t selftimed;
    uint8_t lockbytes;
    uint8_t fusebytes;
    int flashpoll;
    int eeprompoll;
    int pagesize;
    int eepromsize;
    int flashsize;
} AVRISP_parameter_t;


class ESP_AVRISP {
public:
    ESP_AVRISP(uint16_t port, uint8_t reset_pin, uint32_t spi_freq=AVRISP_SPI_FREQ, bool reset_state=false, bool reset_activehigh=false);

    void begin();

    // set the SPI clock frequency
    void setSpiFrequency(uint32_t);

    // control the state of the RESET pin of the target
    // see AVRISP_ACTIVE_HIGH_RESET
    void setReset(bool);

    // check for pending clients if IDLE, check for disconnect otherwise
    // returns the updated state
    AVRISPState_t update();

    // transition to ACTIVE if PENDING
    // serve STK500 commands from buffer if ACTIVE
    // returns the updated state
    AVRISPState_t serve();

protected:

    inline void _reject_incoming(void);     // reject any incoming tcp connections

    void avrisp(void);           // handle incoming STK500 commands

    uint8_t getch(void);        // retrieve a character from the remote end
    void putch(uint8_t value);
    uint8_t spi_transaction(uint8_t, uint8_t, uint8_t, uint8_t);
    void empty_reply(void);
    void breply(uint8_t);

    void get_parameter(uint8_t);
    void set_parameters(void);
    int addr_page(int);
    void flash(uint8_t, int, uint8_t);
    void write_flash(int);
    uint8_t write_flash_pages(int length);
    uint8_t write_eeprom(int length);
    uint8_t write_eeprom_chunk(int start, int length);
    void commit(int addr);
    void program_page();
    uint8_t flash_read(uint8_t hilo, int addr);
    void flash_read_page(int length);
    void eeprom_read_page(int length);
    void read_page();
    void read_signature();

    void universal(void);

    void fill(int);             // fill the buffer with n bytes
    void start_pmode(void);     // enter program mode
    void end_pmode(void);       // exit program mode

    uint32_t _spi_freq;
    WiFiServer _server;
    WiFiClient _client;
    AVRISPState_t _state;
    uint8_t _reset_pin;
    bool _reset_state;
    bool _reset_activehigh;

    // programmer settings, set by remote end
    AVRISP_parameter_t param;
    // page buffer
    uint8_t buff[256];

    int error = 0;

    // Are we in program mode, if so we want to stop program mode when the client disconnects.
    bool pmode = 0;

    // address for reading and writing, set by 'U' command
    int here;
};


#endif // _ESP_AVRISP_H
