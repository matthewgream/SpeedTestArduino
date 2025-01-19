
#pragma once

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <WiFiClient.h>

#include <cstdlib>
#include <functional>
#include <vector>

// -----------------------------------------------------------------------------------------------

struct ClientInfo {
    String address;
    String isp;
    float lat;
    float lon;
};

struct ServerInfo {
    String url;
    String name;
    String country;
    String country_code;
    String host;
    String sponsor;
    int id;
    float lat;
    float lon;
    float distance;
    long latency;
    float version;
};

struct TestConfig {
    size_t start_size;
    size_t max_size;
    size_t incr_size;
    size_t buff_size;
    long min_test_time_ms;
    int concurrency;
    String label;
};

// -----------------------------------------------------------------------------------------------

#define SPEED_TEST_USER_AGENT           "ESP32-SpeedTest/1.0 (Arduino; ESP32) LibWiFiClient/1.0"
#define SPEED_TEST_SERVER_LIST_URL      "https://www.speedtest.net/speedtest-servers.php"
#define SPEED_TEST_IP_INFO_API_URL      "https://api.ipapi.is/"
#define SPEED_TEST_MIN_SERVER_VERSION   2.3
#define SPEED_TEST_LATENCY_SAMPLE_SIZE  80
#define SPEED_TEST_LATENCY_EVAL_SIZE    20
#define SPEED_TEST_TIMEOUT_READ_DEFAULT 30
#define SPEED_TEST_SERVER_SELECT_SAMPLE 10

// -----------------------------------------------------------------------------------------------

class SpeedTestClient {
public:
    explicit SpeedTestClient (const String &server);
    ~SpeedTestClient ();

    bool connect ();
    bool ping (long &millisec);
    bool upload (const size_t total_size, const size_t buff_size, long &millisec);
    bool download (const size_t total_size, const size_t buff_size, long &millisec);
    float version () const;
    const std::pair<String, uint16_t> hostport () const;
    void close ();

private:
    String mServer;
    float mServerVersion;
    WiFiClient mClient;

    bool read (uint8_t *buffer, const size_t length, const int timeout = SPEED_TEST_TIMEOUT_READ_DEFAULT);
    bool write (const uint8_t *buffer, const size_t length);
    bool readLine (String &buffer, const int timeout = SPEED_TEST_TIMEOUT_READ_DEFAULT);
    bool writeLine (const String &buffer);
};

// -----------------------------------------------------------------------------------------------

class SpeedTest  {
public:
    typedef bool (SpeedTestClient::*opFn) (const size_t total_size, const size_t buff_size, long &millisec);
    using cbFn = std::function<void (bool)>;

    explicit SpeedTest (float minServerVersion);

    const ClientInfo &clientInfo () const { return mClientInfo; }
    bool identifyClient (const String &url = SPEED_TEST_IP_INFO_API_URL);

    void insertServer (const ServerInfo &server);
    size_t numServers () const { return mServerList.size (); }
    bool fetchServers (const String &url = SPEED_TEST_SERVER_LIST_URL);
    void sortServersByDistance (const ClientInfo &info);
    const ServerInfo selectBestServer (const int sample_size = SPEED_TEST_SERVER_SELECT_SAMPLE, const cbFn &cb = nullptr);

    bool downloadSpeed (const String &server, const TestConfig &config, double &result, const cbFn &cb = nullptr);
    bool uploadSpeed (const String &server, const TestConfig &config, double &result, const cbFn &cb = nullptr);
    bool latencyAndJitter (const String &server, long &latency, long &jitter, const int sample = SPEED_TEST_LATENCY_SAMPLE_SIZE);

private:
    double execute (const String &server, const TestConfig &config, const opFn &fnc, const cbFn &cb = nullptr);

    ClientInfo mClientInfo {};
    std::vector<ServerInfo> mServerList {};
    float mMinSupportedServer;
};

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

extern int speedTest (const String &server = String ());

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
