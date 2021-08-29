#ifndef _ESP_AVRISP_H
#define _ESP_AVRISP_H

#include <Arduino.h>

// uncomment if you use an n-mos to level-shift the reset line
// #define AVRISP_ACTIVE_HIGH_RESET

// SPI clock frequency in Hz
#define AVRISP_SPI_FREQ   125e3

typedef enum {
    AVRISP_RES_OK = 0,   // Everything went ok
    AVRISP_RES_ERR,      // An error occured
    AVRISP_RES_END       // Last cmd ended programming, connection can be closed.
} AVRISPResult_t;


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
    ESP_AVRISP(uint8_t reset_pin, uint32_t spi_freq=AVRISP_SPI_FREQ, bool reset_state=false, bool reset_activehigh=false);

    // control the state of the RESET pin of the target
    // see AVRISP_ACTIVE_HIGH_RESET
    void setReset(bool);

    // Handle one incoming STK500 command
    AVRISPResult_t handleCmd(Stream& client);           

    // exit program mode
    void end_pmode(void);            

protected:

    void set_parameters(void);
    int addr_page(int);
    void flash(uint8_t, int, uint8_t);
    AVRISPResult_t write_flash(Stream& client, int length);
    uint8_t write_flash_pages(int length);
    uint8_t write_eeprom(Stream& client, int length);
    uint8_t write_eeprom_chunk(Stream& client, int start, int length);
    void commit(int addr);
    AVRISPResult_t program_page(Stream& client);
    void flash_read_page(Stream& client,int length);
    void eeprom_read_page(Stream& client,int length);
    AVRISPResult_t read_page(Stream& client);

    AVRISPResult_t universal(Stream& client);

    void fill(Stream& client, int);  // fill the buffer with n bytes
    void start_pmode(void);          // enter program mode

    uint32_t _spi_freq;

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
