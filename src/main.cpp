
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include <Arduino.h>

#include <WiFi.h>
#include "esp_wifi.h"

static String _protocol_to_string (const uint8_t p) {
    return "802.11" + String (p & WIFI_PROTOCOL_11B ? "b" : "") + (p & WIFI_PROTOCOL_11G ? "g" : "") + (p & WIFI_PROTOCOL_11N ? "n" : "") + (p & WIFI_PROTOCOL_LR ? "l" : "") + (p & WIFI_PROTOCOL_11AX ? "ax" : "");
}
static String _bandwidth_to_string (const uint8_t b) {
    return String ((b >= 1 && b <= 5) ? std::min (160, 20 * (1 << (b - 1))) : 0) + "Mhz";
}

void startWiFi (const char *ssid, const char *pass) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT ();
    cfg.ampdu_tx_enable = 1;
    cfg.ampdu_rx_enable = 1;
    cfg.nano_enable = 0;
    ESP_ERROR_CHECK (esp_wifi_init (&cfg));
#ifdef ARDUINO_VARIANT
    if (strcmp (ARDUINO_VARIANT, "esp32c6") == 0 || strcmp (ARDUINO_VARIANT, "XIAO_ESP32C6") == 0) {
        Serial.printf ("Wifi configure external antenna\n");
        pinMode (GPIO_NUM_3, OUTPUT);
        digitalWrite (GPIO_NUM_3, LOW);    // Activate RF switch control
        delay (100);
        pinMode (GPIO_NUM_14, OUTPUT);
        digitalWrite (GPIO_NUM_14, HIGH);    // Use external antenna
    }
#endif
    WiFi.setHostname ("SpeedTest");
    WiFi.setAutoReconnect (true);
    WiFi.useStaticBuffers (true);
    WiFi.setSleep (false);
    WiFi.setTxPower (WIFI_POWER_21dBm);    // Maximum
    esp_wifi_set_ps (WIFI_PS_NONE);
    WiFi.mode (WIFI_MODE_STA);
    //
    Serial.printf ("Wifi connecting (%s) ...", ssid);
    WiFi.begin (ssid, pass);
    while (WiFi.status () != WL_CONNECTED)
        delay (500), Serial.printf (".");
    Serial.printf (" connected (%d dbm) ...", WiFi.RSSI ());
    while (WiFi.localIP () == IPAddress (0, 0, 0, 0))
        delay (500), Serial.printf (".");
    Serial.printf (" allocated (%s)\n", WiFi.localIP ().toString ().c_str ());
    //
    uint8_t protocol;
    wifi_bandwidth_t bandwidth;
    uint8_t channel;
    wifi_second_chan_t second;
    wifi_country_t country;
    Serial.printf ("Wifi ssid=%s, bssid=%s, protocol=%s, bandwidth=%s, channel=%s, country=%s, txpower=%d, rssi=%d\n",
                   WiFi.SSID ().c_str (),
                   WiFi.BSSIDstr ().c_str (),
                   String (esp_wifi_get_protocol (WIFI_IF_STA, &protocol) == ESP_OK ? _protocol_to_string (protocol) : "n/a").c_str (),
                   String (esp_wifi_get_bandwidth (WIFI_IF_STA, &bandwidth) == ESP_OK ? _bandwidth_to_string (bandwidth) : "n/a").c_str (),
                   String (esp_wifi_get_channel (&channel, &second) == ESP_OK ? String (channel) + (second == WIFI_SECOND_CHAN_ABOVE ? "/+" : (second == WIFI_SECOND_CHAN_BELOW ? "/-" : "")) : "n/a").c_str (),
                   String (esp_wifi_get_country (&country) == ESP_OK ? String (country.cc, 3) : "n/a").c_str (),
                   WiFi.getTxPower (),
                   WiFi.RSSI ());
}

void startTime () {
    Serial.println ("Time ...");
    configTime (0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    while (time (nullptr) < 3600 * 2)
        delay (500);
    Serial.println ("Time OK");
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include "SpeedTest.hpp"
#include "Secrets.hpp"

#define SPEEDTEST_SERVER "speedtest.local:8080"

void setup () {
    delay (5 * 1000);
    Serial.begin (115200);
    Serial.println ("UP");
    startWiFi (WIFI_SSID, WIFI_PASS);
    startTime ();
    //
    // speedTest (SPEEDTEST_SERVER);
    speedTest ();
}

void loop () {
    delay (5 * 1000);
    Serial.println ("***");
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
