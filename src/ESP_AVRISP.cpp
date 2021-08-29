#include <Arduino.h>
#include <SPI.h>
#include "ESP_AVRISP.h"
#include "command.h"

#include <ESP8266WiFi.h>

// #define AVRISP_DEBUG(fmt, ...)     os_printf("[AVRP] " fmt "\r\n", ##__VA_ARGS__ )
#define AVRISP_DEBUG(...)

#define AVRISP_HWVER 2
#define AVRISP_SWMAJ 1
#define AVRISP_SWMIN 18
#define AVRISP_PTIME 10

#define EECHUNK (32)

#define beget16(addr) (*addr * 256 + *(addr+1))

ESP_AVRISP::ESP_AVRISP(uint16_t port, uint8_t reset_pin, uint32_t spi_freq, bool reset_state, bool reset_activehigh):
    _spi_freq(spi_freq), _server(WiFiServer(port)), _state(AVRISP_STATE_IDLE), _reset_pin(reset_pin),
    _reset_state(reset_state), _reset_activehigh(reset_activehigh)     
{
    pinMode(_reset_pin, OUTPUT);

    // Start with the default reset state.
    setReset(_reset_state);
}

void ESP_AVRISP::begin() {
    _server.begin();
}

void ESP_AVRISP::setSpiFrequency(uint32_t freq) {
    _spi_freq = freq;
    if (_state == AVRISP_STATE_ACTIVE) {
        SPI.setFrequency(freq);
    }
}

void ESP_AVRISP::setReset(bool value) {
    digitalWrite(_reset_pin, value == _reset_activehigh);
}

AVRISPState_t ESP_AVRISP::update() {
    switch (_state) {
        case AVRISP_STATE_IDLE: {
            // Check if a client is connecting.
            if (_server.hasClient()) {
                // Accept client connection
                _client = _server.available();
                _client.setNoDelay(true);
                ip4_addr_t lip;
                lip.addr = _client.remoteIP();
                AVRISP_DEBUG("client connect %d.%d.%d.%d:%d", IP2STR(&lip), _client.remotePort());
                _client.setTimeout(100); // for getch()
                _state = AVRISP_STATE_PENDING;
                // Reject any other clients trying to connect.
                _reject_incoming();
            }
            break;
        }
        case AVRISP_STATE_PENDING:
        case AVRISP_STATE_ACTIVE: {
            // Check for client disconnect
            if (!_client.connected()) {
                // Stop client
                _client.stop();
                AVRISP_DEBUG("client disconnect");
                // Reset chip communication
                end_pmode();
                _state = AVRISP_STATE_IDLE;
            } else {
                // Reject any other clients trying to connect.
                _reject_incoming();
            }
            break;
        }
    }
    return _state;
}

AVRISPState_t ESP_AVRISP::serve() {
    switch (update()) {
        case AVRISP_STATE_IDLE:
            // should not be called when idle, error?
            break;
        case AVRISP_STATE_PENDING: {
            _state = AVRISP_STATE_ACTIVE;
        // fallthrough
        }
        case AVRISP_STATE_ACTIVE: {
            while (_client.available()) {
                avrisp();
            }
            return update();
        }
    }
    return _state;
}

inline void ESP_AVRISP::_reject_incoming(void) {
    while (_server.hasClient()) _server.available().stop();
}

uint8_t ESP_AVRISP::getch() {
    while (!_client.available()) yield();
    uint8_t b = (uint8_t)_client.read();
    // AVRISP_DEBUG("< %02x", b);
    return b;
}

void ESP_AVRISP::putch(uint8_t value) {
    _client.write(value);
}

void ESP_AVRISP::fill(int n) {
    // AVRISP_DEBUG("fill(%u)", n);
    for (int x = 0; x < n; x++) {
        buff[x] = getch();
    }
}

uint8_t ESP_AVRISP::spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    uint8_t n;
    SPI.transfer(a);
    n = SPI.transfer(b);
    n = SPI.transfer(c);
    return SPI.transfer(d);
}

void ESP_AVRISP::empty_reply() {
    if (Sync_CRC_EOP == getch()) {
        putch(Resp_STK_INSYNC);
        putch(Resp_STK_OK);
    } else {
        error++;
        putch(Resp_STK_NOSYNC);
    }
}

void ESP_AVRISP::breply(uint8_t b) {
    if (Sync_CRC_EOP == getch()) {
        uint8_t resp[3];
        resp[0] = Resp_STK_INSYNC;
        resp[1] = b;
        resp[2] = Resp_STK_OK;
        _client.write(resp, 3);
    } else {
        error++;
        putch(Resp_STK_NOSYNC);
    }
}

void ESP_AVRISP::get_parameter(uint8_t c) {
    switch (c) {
    case 0x80:
        breply(AVRISP_HWVER);
        break;
    case 0x81:
        breply(AVRISP_SWMAJ);
        break;
    case 0x82:
        breply(AVRISP_SWMIN);
        break;
    case 0x93:
        breply('S'); // serial programmer
        break;
    default:
        breply(0);
    }
}

void ESP_AVRISP::set_parameters() {
    // call this after reading paramter packet into buff[]
    param.devicecode = buff[0];
    param.revision   = buff[1];
    param.progtype   = buff[2];
    param.parmode    = buff[3];
    param.polling    = buff[4];
    param.selftimed  = buff[5];
    param.lockbytes  = buff[6];
    param.fusebytes  = buff[7];
    param.flashpoll  = buff[8];
    // ignore buff[9] (= buff[8])
    // following are 16 bits (big endian)
    param.eeprompoll = beget16(&buff[10]);
    param.pagesize   = beget16(&buff[12]);
    param.eepromsize = beget16(&buff[14]);

    // 32 bits flashsize (big endian)
    param.flashsize = buff[16] * 0x01000000
                    + buff[17] * 0x00010000
                    + buff[18] * 0x00000100
                    + buff[19];
}

void ESP_AVRISP::start_pmode() {

    // Set up SPI
    SPI.begin();
    SPI.setFrequency(_spi_freq);
    SPI.setHwCs(false);

    if(!_reset_state) {
        // Send a reset inactive pulse, so start with reset active.
        // Only need this if reset is inactive by default.
        setReset(true); 
        delay(30);
    }

    // Send pulse, for attiny85 at least 2.5μs + 2 clocks
    setReset(false);
    delayMicroseconds(10);
    setReset(true);

    // Wait before first command, at least 20ms according to spec.
    delay(30);

    // Send Programming Enable
    spi_transaction(0xAC, 0x53, 0x00, 0x00);
    pmode = 1;
}

void ESP_AVRISP::end_pmode() {
    // Only need revert things if we actually started programming mode.
    if (pmode) {
        SPI.end();

        // Go back to default reset state
        setReset(_reset_state);

        pmode = 0;
    }
}

void ESP_AVRISP::universal() {
    uint8_t ch;

    fill(4);
    ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
    breply(ch);
}

void ESP_AVRISP::flash(uint8_t hilo, int addr, uint8_t data) {
    spi_transaction(0x40 + 8 * hilo,
                    addr >> 8 & 0xFF,
                    addr & 0xFF,
                    data);
}

void ESP_AVRISP::commit(int addr) {
    spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
    delay(AVRISP_PTIME);
}

//#define _addr_page(x) (here & 0xFFFFE0)
int ESP_AVRISP::addr_page(int addr) {
    if (param.pagesize == 32)  return addr & 0xFFFFFFF0;
    if (param.pagesize == 64)  return addr & 0xFFFFFFE0;
    if (param.pagesize == 128) return addr & 0xFFFFFFC0;
    if (param.pagesize == 256) return addr & 0xFFFFFF80;
    AVRISP_DEBUG("unknown page size: %d", param.pagesize);
    return addr;
}


void ESP_AVRISP::write_flash(int length) {
    fill(length);

    if (Sync_CRC_EOP == getch()) {
        putch(Resp_STK_INSYNC);
        putch(write_flash_pages(length));
    } else {
      error++;
      putch(Resp_STK_NOSYNC);
    }
}

uint8_t ESP_AVRISP::write_flash_pages(int length) {
    int x = 0;
    int page = addr_page(here);
    while (x < length) {
        yield();
        if (page != addr_page(here)) {
            commit(page);
            page = addr_page(here);
        }
        flash(LOW, here, buff[x++]);
        flash(HIGH, here, buff[x++]);
        here++;
    }
    commit(page);
    return Resp_STK_OK;
}

uint8_t ESP_AVRISP::write_eeprom(int length) {
    // here is a word address, get the byte address
    int start = here * 2;
    int remaining = length;
    if (length > param.eepromsize) {
        error++;
        return Resp_STK_FAILED;
    }
    while (remaining > EECHUNK) {
        write_eeprom_chunk(start, EECHUNK);
        start += EECHUNK;
        remaining -= EECHUNK;
    }
    write_eeprom_chunk(start, remaining);
    return Resp_STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t ESP_AVRISP::write_eeprom_chunk(int start, int length) {
    // this writes byte-by-byte,
    // page writing may be faster (4 bytes at a time)
    fill(length);
    // prog_lamp(LOW);
    for (int x = 0; x < length; x++) {
        int addr = start + x;
        spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
        delay(45);
    }
    // prog_lamp(HIGH);
    return Resp_STK_OK;
}

void ESP_AVRISP::program_page() {
    char result = (char) Resp_STK_FAILED;
    int length = 256 * getch();
    length += getch();
    char memtype = getch();
    // flash memory @here, (length) bytes
    if (memtype == 'F') {
        write_flash(length);
        return;
    }

    if (memtype == 'E') {
        result = write_eeprom(length);
        if (Sync_CRC_EOP == getch()) {
            putch(Resp_STK_INSYNC);
            putch(result);
        } else {
            error++;
            putch(Resp_STK_NOSYNC);
        }
        return;
    }
    putch(Resp_STK_FAILED);
  return;

}

uint8_t ESP_AVRISP::flash_read(uint8_t hilo, int addr) {
    return spi_transaction(0x20 + hilo * 8,
                           (addr >> 8) & 0xFF,
                           addr & 0xFF,
                           0);
}

void ESP_AVRISP::flash_read_page(int length) {
    uint8_t *data = (uint8_t *) malloc(length + 1);
    for (int x = 0; x < length; x += 2) {
        *(data + x) = flash_read(LOW, here);
        *(data + x + 1) = flash_read(HIGH, here);
        here++;
    }
    *(data + length) = Resp_STK_OK;
    _client.write((const uint8_t *)data, (size_t)(length + 1));
    free(data);
    return;
}

void ESP_AVRISP::eeprom_read_page(int length) {
    // here again we have a word address
    uint8_t *data = (uint8_t *) malloc(length + 1);
    int start = here * 2;
    for (int x = 0; x < length; x++) {
        int addr = start + x;
        uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
        *(data + x) = ee;
    }
    *(data + length) = Resp_STK_OK;
    _client.write((const uint8_t *)data, (size_t)(length + 1));
    free(data);
    return;
}

void ESP_AVRISP::read_page() {
    int length = 256 * getch();
    length += getch();
    char memtype = getch();
    if (Sync_CRC_EOP != getch()) {
        error++;
        putch(Resp_STK_NOSYNC);
        return;
    }
    putch(Resp_STK_INSYNC);
    if (memtype == 'F') flash_read_page(length);
    if (memtype == 'E') eeprom_read_page(length);
    return;
}

void ESP_AVRISP::read_signature() {
    if (Sync_CRC_EOP != getch()) {
        error++;
        putch(Resp_STK_NOSYNC);
        return;
    }
    putch(Resp_STK_INSYNC);

    uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
    putch(high);
    uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
    putch(middle);
    uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
    putch(low);
    putch(Resp_STK_OK);
}

// We implement a subset of the STK500 (not v2) commands.
void ESP_AVRISP::avrisp() {
    const uint8_t cmd = getch();
    // AVRISP_DEBUG("CMD 0x%02x", ch);
    switch (cmd) {
    case Cmnd_STK_GET_SYNC:
        error = 0;
        empty_reply();
        break;

    case Cmnd_STK_GET_SIGN_ON:
        if (getch() == Sync_CRC_EOP) {
            putch(Resp_STK_INSYNC);
            // Use a different message from STK500 ("AVR STK"),
            // I guess to allow callers to know we're different.
            // As far as I know avrdude doesn't even use this
            _client.write("AVR ISP");
            putch(Resp_STK_OK);
        }
        break;

    case Cmnd_STK_GET_PARAMETER:
        get_parameter(getch());
        break;

    case Cmnd_STK_SET_DEVICE:
        fill(20);
        set_parameters();
        empty_reply();
        break;

    case Cmnd_STK_SET_DEVICE_EXT:   // ignored
        fill(5);
        empty_reply();
        break;

    case Cmnd_STK_ENTER_PROGMODE:
        start_pmode();
        empty_reply();
        break;

    case Cmnd_STK_LOAD_ADDRESS:
        here = getch();
        here += 256 * getch();
        // AVRISP_DEBUG("here=0x%04x", here);
        empty_reply();
        break;

    // XXX: not implemented!
    case Cmnd_STK_PROG_FLASH:
        {
            const uint8_t low = getch();
            const uint8_t high = getch();
            empty_reply();
        }
        break;

    // XXX: not implemented!
        case Cmnd_STK_PROG_DATA:
        {
            const uint8_t data = getch();
            empty_reply();
        }
        break;

    case Cmnd_STK_PROG_PAGE:
        program_page();
        break;

    case Cmnd_STK_READ_PAGE:
        read_page();
        break;

    case Cmnd_STK_UNIVERSAL:
        universal();
        break;

    case Cmnd_STK_LEAVE_PROGMODE:
        error = 0;
        end_pmode();
        empty_reply();
        delay(5);
        // if (_client && _client.connected())
        _client.stop();
        // AVRISP_DEBUG("left progmode");

        break;

    case Cmnd_STK_READ_SIGN:
        read_signature();
        break;

    // expecting a command, not Sync_CRC_EOP
    // this is how we can get back in sync
    case Sync_CRC_EOP:       // 0x20, space
        error++;
        putch(Resp_STK_NOSYNC);
        break;

    // Anything else we will return STK_UNKNOWN
    default:
        AVRISP_DEBUG("Unexpected command: 0x%02x", ch);
        error++;
        if (Sync_CRC_EOP == getch()) {
            putch(Resp_STK_UNKNOWN);
        } else {
            putch(Resp_STK_NOSYNC);
        }
  }
}
