#include <Arduino.h>
#include <SPI.h>
#include "ESP_AVRISP.h"
#include "command.h"

// #define AVRISP_DEBUG(fmt, ...)     os_printf("[AVRP] " fmt "\r\n", ##__VA_ARGS__ )
#define AVRISP_DEBUG(...)

#define AVRISP_HWVER 2
#define AVRISP_SWMAJ 1
#define AVRISP_SWMIN 18
#define AVRISP_PTIME 10

#define EECHUNK (32)

#define beget16(addr) (*addr * 256 + *(addr+1))

ESP_AVRISP::ESP_AVRISP(uint8_t reset_pin, uint32_t spi_freq, bool reset_state, bool reset_activehigh):
    _spi_freq(spi_freq), _reset_pin(reset_pin), _reset_state(reset_state), _reset_activehigh(reset_activehigh) {
    pinMode(_reset_pin, OUTPUT);

    // Start with the default reset state.
    setReset(_reset_state);
}

void ESP_AVRISP::setReset(bool value) {
    digitalWrite(_reset_pin, value == _reset_activehigh);
}

// retrieve a character from the remote end
static uint8_t getch(Stream& client) {
    while (!client.available()) yield();
    uint8_t b = (uint8_t)client.read();
    // AVRISP_DEBUG("< %02x", b);
    return b;
}

void ESP_AVRISP::fill(Stream& client, int n) {
    // AVRISP_DEBUG("fill(%u)", n);
    for (int x = 0; x < n; x++) {
        buff[x] = getch(client);
    }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    uint8_t n;
    SPI.transfer(a);
    n = SPI.transfer(b);
    n = SPI.transfer(c);
    return SPI.transfer(d);
}

AVRISPResult_t checkSync(Stream& client) {
    if (Sync_CRC_EOP == getch(client)) {
        client.write(Resp_STK_INSYNC);
        return AVRISP_RES_OK;
    } else {
        client.write(Resp_STK_NOSYNC);
        return AVRISP_RES_ERR;
    }
}

AVRISPResult_t empty_reply(Stream& client) {
    AVRISPResult_t result = checkSync(client);
    if(result == AVRISP_RES_OK) {
        client.write(Resp_STK_OK);
    }
    return result;
}

AVRISPResult_t byte_reply(Stream& client, uint8_t b) {
    AVRISPResult_t result = checkSync(client);
    if(result == AVRISP_RES_OK) {
        client.write(b);
        client.write(Resp_STK_OK);
    }
    return result;
}

AVRISPResult_t buffer_reply(Stream& client, const uint8_t* buffer, size_t size) {
    AVRISPResult_t result = checkSync(client);
    if(result == AVRISP_RES_OK) {
        client.write(buffer, size);
        client.write(Resp_STK_OK);
    }
    return result;
}

AVRISPResult_t get_parameter(Stream& client) {
    uint8_t c = getch(client);
    switch (c) {
    case 0x80:
        return byte_reply(client, AVRISP_HWVER);
    case 0x81:
        return byte_reply(client, AVRISP_SWMAJ);
    case 0x82:
        return byte_reply(client, AVRISP_SWMIN);
    case 0x93:
        return byte_reply(client, 'S'); // serial programmer
    default:
        return byte_reply(client, 0);
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

AVRISPResult_t ESP_AVRISP::universal(Stream& client) {
    uint8_t ch;

    fill(client, 4);
    ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
    return byte_reply(client, ch);
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


AVRISPResult_t ESP_AVRISP::write_flash(Stream& client, int length) {
    fill(client, length);
    AVRISPResult_t result = checkSync(client);
    if(result == AVRISP_RES_OK) {
        client.write(write_flash_pages(length));
    }
    return result;
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

uint8_t ESP_AVRISP::write_eeprom(Stream& client, int length) {
    // here is a word address, get the byte address
    int start = here * 2;
    int remaining = length;
    if (length > param.eepromsize) {
        error++;
        return Resp_STK_FAILED;
    }
    while (remaining > EECHUNK) {
        write_eeprom_chunk(client, start, EECHUNK);
        start += EECHUNK;
        remaining -= EECHUNK;
    }
    write_eeprom_chunk(client, start, remaining);
    return Resp_STK_OK;
}

// write (length) bytes, (start) is a byte address
uint8_t ESP_AVRISP::write_eeprom_chunk(Stream& client, int start, int length) {
    // this writes byte-by-byte,
    // page writing may be faster (4 bytes at a time)
    fill(client, length);
    // prog_lamp(LOW);
    for (int x = 0; x < length; x++) {
        int addr = start + x;
        spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
        delay(45);
    }
    // prog_lamp(HIGH);
    return Resp_STK_OK;
}

AVRISPResult_t ESP_AVRISP::program_page(Stream& client) {
    char result = (char) Resp_STK_FAILED;
    int length = 256 * getch(client);
    length += getch(client);
    char memtype = getch(client);
    // flash memory @here, (length) bytes
    if (memtype == 'F') {
        return write_flash(client, length);
    }

    if (memtype == 'E') {
        result = write_eeprom(client, length);
        AVRISPResult_t resultCode = checkSync(client);
        if(resultCode == AVRISP_RES_OK) {
            client.write(result);
        }
        return resultCode;
    }

    client.write(Resp_STK_FAILED);
    return AVRISP_RES_ERR;
}

uint8_t flash_read(uint8_t hilo, int addr) {
    return spi_transaction(0x20 + hilo * 8,
                           (addr >> 8) & 0xFF,
                           addr & 0xFF,
                           0);
}

void ESP_AVRISP::flash_read_page(Stream& client, int length) {
    uint8_t *data = (uint8_t *) malloc(length + 1);
    for (int x = 0; x < length; x += 2) {
        *(data + x) = flash_read(LOW, here);
        *(data + x + 1) = flash_read(HIGH, here);
        here++;
    }
    *(data + length) = Resp_STK_OK;
    client.write((const uint8_t *)data, (size_t)(length + 1));
    free(data);
    return;
}

void ESP_AVRISP::eeprom_read_page(Stream& client, int length) {
    // here again we have a word address
    uint8_t *data = (uint8_t *) malloc(length + 1);
    int start = here * 2;
    for (int x = 0; x < length; x++) {
        int addr = start + x;
        uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
        *(data + x) = ee;
    }
    *(data + length) = Resp_STK_OK;
    client.write((const uint8_t *)data, (size_t)(length + 1));
    free(data);
    return;
}

AVRISPResult_t ESP_AVRISP::read_page(Stream& client) {
    int length = 256 * getch(client);
    length += getch(client);
    char memtype = getch(client);

    AVRISPResult_t result = checkSync(client);
    if(result == AVRISP_RES_OK) {
        if (memtype == 'F') flash_read_page(client, length);
        if (memtype == 'E') eeprom_read_page(client, length);
    }
    return result;
}

AVRISPResult_t read_signature(Stream& client) {
    uint8_t result[3] = {};
    result[0] = spi_transaction(0x30, 0x00, 0x00, 0x00);
    result[1] = spi_transaction(0x30, 0x00, 0x01, 0x00);
    result[2] = spi_transaction(0x30, 0x00, 0x02, 0x00);

    return buffer_reply(client, result, 3);
}

AVRISPResult_t sign_on(Stream& client) {
    AVRISPResult_t result = checkSync(client);
    if(result == AVRISP_RES_OK) {
        // Use a different message from STK500 ("AVR STK"),
        // I guess to allow callers to know we're different.
        // As far as I know avrdude doesn't even use this
        client.write("AVR ISP");
        client.write(Resp_STK_OK);
    }
    return result;
}

// We implement a subset of the STK500 (not v2) commands.
AVRISPResult_t ESP_AVRISP::handleCmd(Stream& client) {
    const uint8_t cmd = getch(client);
    // AVRISP_DEBUG("CMD 0x%02x", ch);
    switch (cmd) {
    case Cmnd_STK_GET_SYNC:
        error = 0;
        if(empty_reply(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_GET_SIGN_ON:
        if(sign_on(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_GET_PARAMETER:
        if(get_parameter(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_SET_DEVICE:
        fill(client, 20);
        set_parameters();
        if(empty_reply(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_SET_DEVICE_EXT:   // ignored
        fill(client, 5);
        if(empty_reply(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_ENTER_PROGMODE:
        start_pmode();
        if(empty_reply(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_LOAD_ADDRESS:
        here = getch(client);
        here += 256 * getch(client);
        // AVRISP_DEBUG("here=0x%04x", here);
        if(empty_reply(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    // XXX: not implemented!
    case Cmnd_STK_PROG_FLASH:
        {
            const uint8_t low = getch(client);
            const uint8_t high = getch(client);
            if(empty_reply(client) != AVRISP_RES_OK) {
                error++;
            }
        }
        break;

    // XXX: not implemented!
        case Cmnd_STK_PROG_DATA:
        {
            const uint8_t data = getch(client);
            if(empty_reply(client) != AVRISP_RES_OK) {
                error++;
            }
        }
        break;

    case Cmnd_STK_PROG_PAGE:
        if(program_page(client) != AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_READ_PAGE:
        read_page(client);
        break;

    case Cmnd_STK_UNIVERSAL:
        if(universal(client)!= AVRISP_RES_OK) {
            error++;
        }
        break;

    case Cmnd_STK_LEAVE_PROGMODE:
        error = 0;
        end_pmode();
        if(empty_reply(client) != AVRISP_RES_OK) {
            error++;
        }
        client.flush();
        return AVRISP_RES_END;

    case Cmnd_STK_READ_SIGN:
        if(read_signature(client) != AVRISP_RES_OK) {
            error++;
        }        
        break;

    // expecting a command, not Sync_CRC_EOP
    // this is how we can get back in sync
    case Sync_CRC_EOP:
        error++;
        client.write(Resp_STK_NOSYNC);
        break;

    // Anything else we will return STK_UNKNOWN
    default:
        AVRISP_DEBUG("Unexpected command: 0x%02x", ch);
        error++;
        if (Sync_CRC_EOP == getch(client)) {
            client.write(Resp_STK_UNKNOWN);
        } else {
            client.write(Resp_STK_NOSYNC);
        }
  }
  return AVRISP_RES_OK;
}
