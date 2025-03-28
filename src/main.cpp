
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

// #define USE_ETHERNET
// #define USE_WIFI

#if ! defined(USE_ETHERNET) && ! defined(USE_WIFI)
#define USE_WIFI
#endif

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <esp_pthread.h>

void __esp32_specific_init () {
    esp_pthread_cfg_t cfg;
    esp_pthread_get_cfg (&cfg);
    cfg.stack_size = 4096;
    if (esp_pthread_set_cfg (&cfg) != ESP_OK)
        Serial.println ("could not esp_pthread_set_cfg");
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

void startTime () {
    Serial.println ("Time ...");
    configTime (0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    while (time (nullptr) < 3600 * 2)
        delay (500);
    Serial.println ("Time OK");
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#ifdef USE_WIFI

#include <WiFi.h>
#include <esp_wifi.h>

static String _protocol_to_string (const uint8_t p) {
    struct ProtocolInfo {
        uint8_t flag;
        String name;
    };
    static const ProtocolInfo protocols [] = {
#ifdef WIFI_PROTOCOL_11B
        {  WIFI_PROTOCOL_11B,  "b" },
#endif
#ifdef WIFI_PROTOCOL_11G
        {  WIFI_PROTOCOL_11G,  "g" },
#endif
#ifdef WIFI_PROTOCOL_11N
        {  WIFI_PROTOCOL_11N,  "n" },
#endif
#ifdef WIFI_PROTOCOL_LR
        {   WIFI_PROTOCOL_LR,  "l" },
#endif
#ifdef WIFI_PROTOCOL_11A
        {  WIFI_PROTOCOL_11A,  "a" },
#endif
#ifdef WIFI_PROTOCOL_11AC
        { WIFI_PROTOCOL_11AC, "ac" },
#endif
#ifdef WIFI_PROTOCOL_11AX
        { WIFI_PROTOCOL_11AX, "ax" },
#endif
    };
    String r;
    for (const auto &protocol : protocols)
        if (p & protocol.flag)
            r += (r.isEmpty () ? "" : "/") + protocol.name;
    return "802.11" + r;
}
static String _bandwidth_to_string (const uint8_t b) {
    return String ((b >= 1 && b <= 5) ? std::min (160, 20 * (1 << (b - 1))) : 0) + "Mhz";
}
static String _channel_to_string (const uint8_t c, const wifi_second_chan_t s) {
    return String (c) + (s == WIFI_SECOND_CHAN_ABOVE ? "/+" : (s == WIFI_SECOND_CHAN_BELOW ? "/-" : ""));
}
static String _country_to_string (const wifi_country_t c) {
    String r (c.cc, 3);
    r.trim ();
    return r;
}

void startWiFi (const char *ssid, const char *pass) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT ();
    cfg.ampdu_tx_enable = 1;
    cfg.ampdu_rx_enable = 1;
    cfg.nano_enable = 0;
    cfg.nvs_enable = 0;
    ESP_ERROR_CHECK (esp_wifi_init (&cfg));
#if defined(ARDUINO_VARIANT) && defined(WIFI_ANTENNA_EXTERNAL)
    if (String (ARDUINO_VARIANT) == "esp32c6" || String (ARDUINO_VARIANT) == "XIAO_ESP32C6") {
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
    // WiFi.setTxPower (WIFI_POWER_8_5dBm);    // XXX ?!? for AUTH_EXPIRE ... flash access problem ...  https://github.com/espressif/arduino-esp32/issues/2144
    WiFi.setTxPower (WIFI_POWER_21dBm);    // Maximum
    esp_wifi_set_ps (WIFI_PS_NONE);
    WiFi.mode (WIFI_MODE_STA);
    //
    Serial.printf ("Wifi connecting (%s) ...", ssid);
    WiFi.begin (ssid, pass);
    while (WiFi.status () != WL_CONNECTED)
        delay (500), Serial.printf (".");
    Serial.printf (" connected (%d dbm) ...", WiFi.RSSI ());
    while (WiFi.localIP () == INADDR_NONE)
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
                   String (esp_wifi_get_channel (&channel, &second) == ESP_OK ? _channel_to_string (channel, second) : "n/a").c_str (),
                   String (esp_wifi_get_country (&country) == ESP_OK ? _country_to_string (country) : "n/a").c_str (),
                   WiFi.getTxPower (),
                   WiFi.RSSI ());
}

#endif

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#ifdef USE_ETHERNET

#include <ETH.h>

#if defined(WAVESHARE_ESP32_S3_ETH)
#define ETH_MISO_PIN 12
#define ETH_MOSI_PIN 11
#define ETH_SCLK_PIN 13
#define ETH_CS_PIN   14
#define ETH_INT_PIN  10
#define ETH_RST_PIN  9
#define ETH_ADDR     1
#else
#error "Ethernet board has not been specified"
#endif

void startEthernet () {
    if (! ETH.begin (ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, ETH_RST_PIN, SPI3_HOST, ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN)) {
        Serial.println ("ETH start Failed!");
        return;
    }
    Serial.printf ("Ethernet connecting ...");
    while (! ETH.connected ())
        delay (500), Serial.printf (".");
    Serial.printf (" connected (%d Mbps) ...", ETH.linkSpeed ());
    while (ETH.localIP () == INADDR_NONE)
        delay (500), Serial.printf (".");
    Serial.printf (" allocated (%s)\n", ETH.localIP ().toString ().c_str ());
}

#endif

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#ifdef USE_WIFI
#if ! defined(WIFI_SSID) || ! defined(WIFI_PASS)
#include "Secrets.hpp"
#endif
#endif

void startNetwork () {
#if defined(USE_WIFI)
    startWiFi (WIFI_SSID, WIFI_PASS);
#elif defined(USE_ETHERNET)
    startEthernet ();
#endif
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include "SpeedTest.hpp"

// #define SPEEDTEST_SERVER "speedtest.local:8080"
// #define SPEEDTEST_PROFILE SpeedTestProfile::SLOW
const SpeedTestConfig speedTestConfig {
#ifdef SPEEDTEST_SERVER
    {  SpeedTestConfigType::SERVER,  SPEEDTEST_SERVER },
#endif
#ifdef SPEEDTEST_PROFILE
    { SpeedTestConfigType::PROFILE, SPEEDTEST_PROFILE },
#endif
};

void setup () {
    delay (5 * 1000);
    Serial.begin (115200);
    Serial.println ("UP");
    __esp32_specific_init ();
    startNetwork ();
    startTime ();
    speedTest (speedTestConfig);
}

void loop () {
    delay (5 * 1000);
    Serial.println ("***");
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
