#include <SPI.h>
#include <ESP_AVRISP.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

const char* host = "esp-avrisp";
const char* ssid = "Milkrun";
const char* pass = "55382636751623425906";
const uint16_t port = 328;
const uint8_t reset_pin = 5;

typedef enum {
    AVRISP_STATE_IDLE = 0,    // no active TCP session
    AVRISP_STATE_ACTIVE       // TCP connected
} AVRISPState_t;

AVRISPState_t _state = AVRISP_STATE_IDLE;

WiFiServer _server(port);
WiFiClient _client = {};

ESP_AVRISP avrprog(reset_pin);

void setup() {
    Serial.begin(115200);
    Serial.println("");
    Serial.println("Arduino AVR-ISP over TCP");
    avrprog.setReset(false); // let the AVR run

    WiFi.begin(ssid, pass);
    while (WiFi.waitForConnectResult() != WL_CONNECTED);

    MDNS.begin(host);
    MDNS.addService("avrisp", "tcp", port);

    IPAddress local_ip = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(local_ip);
    Serial.println("Use your avrdude:");
    Serial.print("avrdude -c arduino -p <device> -P net:");
    Serial.print(local_ip);
    Serial.print(":");
    Serial.print(port);
    Serial.println(" -t # or -U ...");

    // listen for avrdudes
    _server.begin();
}

// reject any incoming tcp connections
void _reject_incoming() {
    while (_server.hasClient()) _server.available().stop();
}

void loop() {

    // Handle client status
    switch (_state) {
        case AVRISP_STATE_IDLE: {
            // Check if a client is connecting.
            if (_server.hasClient()) {
                // Accept client connection
                _client = _server.available();
                _client.setNoDelay(true);
                ip4_addr_t lip;
                lip.addr = _client.remoteIP();
                Serial.printf("client connect %d.%d.%d.%d:%d", IP2STR(&lip), _client.remotePort());
                Serial.println();
                _client.setTimeout(100); // for getch()
                _state = AVRISP_STATE_ACTIVE;
                // Reject any other clients trying to connect.
                _reject_incoming();
            }
            break;
        }
        case AVRISP_STATE_ACTIVE: {
            // Check for client disconnect
            if (!_client.connected()) {
                // Stop client
                _client.stop();
                Serial.println("client disconnect");
                // Reset chip communication
                avrprog.end_pmode();
                _state = AVRISP_STATE_IDLE;
            } else {
                // Reject any other clients trying to connect.
                _reject_incoming();
            }
            break;
        }
    }    

    // Serve the client
    if (_state == AVRISP_STATE_ACTIVE) {
        while (_client.available()) {
            if(avrprog.handleCmd(_client) == AVRISP_RES_END) {
                _client.stop();
            }
        }
    }
}
