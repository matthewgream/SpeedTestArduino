
//
// Original by Francesco Laurita on 5/29/16.
// Current by mgream 2025
//

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include "SpeedTest.hpp"

#include <HTTPClient.h>

#define xxx_isAlpha isAlpha
#undef isAlpha
#include <TinyXML.h>
#undef isAlpha
#define isAlpha xxx_isAlpha
#include <ArduinoJson.h>

#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

static inline constexpr float EARTH_RADIUS_KM = 6371.0;

template <typename T>
static inline constexpr T deg2rad (T n) {
    return (n * M_PI / 180);
}
template <typename T>
static inline constexpr T harversine (std::pair<T, T> n1, std::pair<T, T> n2) {
    const T lat1r = deg2rad (n1.first), lon1r = deg2rad (n1.second);
    const T lat2r = deg2rad (n2.first), lon2r = deg2rad (n2.second);
    const T u = std::sin ((lat2r - lat1r) / 2), v = std::sin ((lon2r - lon1r) / 2);
    return 2.0 * EARTH_RADIUS_KM * std::asin (std::sqrt (u * u + std::cos (lat1r) * std::cos (lat2r) * v * v));
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

SpeedTestClient::SpeedTestClient (const String &server) :
    mServer (server),
    mServerVersion (-1.0) { }

SpeedTestClient::~SpeedTestClient () {
    close ();
}

bool SpeedTestClient::connect () {
    if (! mClient.connected ()) {
        auto hostp = hostport ();
        if (! mClient.connect (hostp.first.c_str (), hostp.second))
            return false;
        mClient.setNoDelay (true);
    }
    String reply;
    int offset;
    if (! writeLine ("HI"))
        return false;
    if (! readLine (reply) || ! reply.startsWith ("HELLO ") || (offset = reply.indexOf (' ')) < 0)
        return false;
    mServerVersion = reply.substring (offset + 1, reply.indexOf (' ', offset + 1)).toFloat ();
    return true;
}

void SpeedTestClient::close () {
    if (mClient.connected ()) {
        writeLine ("QUIT");
        mClient.stop ();
    }
}

bool SpeedTestClient::ping (long &millisec) {
    if (! mClient.connected ())
        return false;
    const auto start = std::chrono::steady_clock::now ();
    if (! writeLine (String ("PING ") + String (start.time_since_epoch ().count ())))
        return false;
    String reply;
    if (! readLine (reply) || ! reply.startsWith ("PONG "))
        return false;
    millisec = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - start).count ();
    return true;
}

bool SpeedTestClient::download (const size_t total_size, const size_t buff_size, long &millisec) {
    if (! writeLine (String ("DOWNLOAD ") + String (total_size)))
        return false;
    uint8_t *buff = new uint8_t [buff_size];
    if (buff == nullptr)
        return false;
    const auto start = std::chrono::steady_clock::now ();
    size_t recv_size = 0;
    while (recv_size < total_size) {
        const size_t left_size = std::min (buff_size, total_size - recv_size);
        if (! read (buff, left_size))
            goto download_fail;
        recv_size += left_size;
    }
    millisec = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - start).count ();
    delete [] buff;
    return true;
download_fail:
    delete [] buff;
    return false;
}

bool SpeedTestClient::upload (const size_t total_size, const size_t buff_size, long &millisec) {
    String reply;
    int offset;
    const String command = String ("UPLOAD ") + String (total_size) + "\n";
    if (! writeLine (command))
        return false;
    uint8_t *buff = new uint8_t [buff_size];
    for (size_t i = 0; i < buff_size; i++)
        buff [i] = static_cast<uint8_t> (rand () % 256);
    const auto start = std::chrono::steady_clock::now ();
    size_t send_size = command.length ();
    while (send_size < total_size) {
        const size_t left_size = total_size - send_size;
        if (left_size > buff_size) {
            if (! write (buff, buff_size))
                goto upload_fail;
            send_size += buff_size;
        } else {
            buff [left_size - 1] = '\n';
            if (! write (buff, left_size))
                goto upload_fail;
            send_size += left_size;
        }
    }
    if (! readLine (reply) || ! reply.startsWith ("OK ") || (offset = reply.indexOf (' ')) < 0) {
        Serial.printf ("upload faulty response: %s\n", reply.c_str ());
        goto upload_fail;
    }
    millisec = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - start).count ();
    delete [] buff;
    return reply.substring (offset + 1, reply.indexOf (' ', offset + 1)).toInt () == total_size;
upload_fail:
    delete [] buff;
    return false;
}

float SpeedTestClient::version () const {
    return mServerVersion;
}

const std::pair<String, uint16_t> SpeedTestClient::hostport () const {
    const int colon = mServer.indexOf (':');
    return std::pair<String, uint16_t> (mServer.substring (0, colon), static_cast<uint16_t> (mServer.substring (colon + 1).toInt ()));
}

bool SpeedTestClient::read (uint8_t *buffer, const size_t length, const int timeout) {
    if (! mClient.connected ())
        return false;
    const long t = millis () + (timeout * 1000);
    size_t obtained = 0;
    while (obtained < length) {
        int n;
        while ((n = mClient.read (&buffer [obtained], length - obtained)) <= 0) {
            if (millis () > t)
                return false;
            delay (5);
        }
        obtained += n;
    }
    return true;
}

bool SpeedTestClient::write (const uint8_t *buffer, const size_t length) {
    if (! mClient.connected ())
        return false;
    return length > 0 && mClient.write (buffer, length) == length;
}

bool SpeedTestClient::readLine (String &buffer, const int timeout) {
    buffer.clear ();
    if (! mClient.connected ())
        return false;
    const long t = millis () + (timeout * 1000);
    while (true) {
        int n;
        while ((n = mClient.read ()) == -1) {
            if (millis () > t)
                return false;
            delay (5);
        }
        const char c = static_cast<char> (n);
        if (c == '\n' || c == '\r')
            break;
        buffer += c;
    }
    return true;
}

bool SpeedTestClient::writeLine (const String &buffer) {
    if (! mClient.connected ())
        return false;
    if (buffer.length () == 0)
        return false;
    if (mClient.write (buffer.c_str ()) != buffer.length ())
        return false;
    if (buffer.indexOf ('\n') < 0 && mClient.write ("\n") != 1)
        return false;
    return true;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

SpeedTest::SpeedTest (float minServerVersion) : mMinSupportedServer (minServerVersion) {
}

bool SpeedTest::identifyClient (const String &url) {
    HTTPClient client;
    client.setUserAgent (SPEED_TEST_USER_AGENT);
    client.setFollowRedirects (HTTPC_FORCE_FOLLOW_REDIRECTS);
    client.begin (url);
    int code;
    if ((code = client.GET ()) != HTTP_CODE_OK) {
        Serial.printf ("clientInfo: HTTP GET error-code=%d\n", code);
        return false;
    }
    JsonDocument json;
    const DeserializationError error = deserializeJson (json, client.getStream ());
    client.end ();
    if (error)
        return false;
    try {
        mClientInfo.address = json ["ip"].as<String> ();
        mClientInfo.isp = json ["company"]["name"].as<String> ();
        mClientInfo.lat = json ["location"]["latitude"].as<String> ().toFloat ();
        mClientInfo.lon = json ["location"]["longitude"].as<String> ().toFloat ();
    } catch (...) {
        return false;
    }
    return true;
}

//

void SpeedTest::insertServer (const ServerInfo &server) {
    mServerList.push_back (server);
}
bool SpeedTest::fetchServers (const String &url) {
    HTTPClient client;
    client.setUserAgent (SPEED_TEST_USER_AGENT);
    client.setFollowRedirects (HTTPC_FORCE_FOLLOW_REDIRECTS);
    client.begin (url);
    int code;
    if ((code = client.GET ()) != HTTP_CODE_OK) {
        Serial.printf ("fetchServers: HTTP GET error-code=%d\n", code);
        client.end ();
        return false;
    }
    TinyXML xml;
    uint8_t buffer [256];
    static SpeedTest *__xml_parse_speedTest;
    static ServerInfo __xml_parse_serverInfo;
    __xml_parse_serverInfo = ServerInfo ();
    __xml_parse_speedTest = this;
    xml.init (buffer, sizeof (buffer), [] (uint8_t flags, char *name, uint16_t nameLen, char *data, uint16_t dataLen) {
        if (name != nullptr && data != nullptr) {
            if (strcasecmp (name, "url") == 0) {
                if (! __xml_parse_serverInfo.url.isEmpty ())
                    __xml_parse_speedTest->insertServer (__xml_parse_serverInfo);
                __xml_parse_serverInfo = ServerInfo { .url = String (data) };
            } else if (strcasecmp (name, "lat") == 0)
                __xml_parse_serverInfo.lat = String (data).toFloat ();
            else if (strcasecmp (name, "lon") == 0)
                __xml_parse_serverInfo.lon = String (data).toFloat ();
            else if (strcasecmp (name, "name") == 0)
                __xml_parse_serverInfo.name = String (data);
            else if (strcasecmp (name, "country") == 0)
                __xml_parse_serverInfo.country = String (data);
            else if (strcasecmp (name, "cc") == 0)
                __xml_parse_serverInfo.country_code = String (data);
            else if (strcasecmp (name, "host") == 0)
                __xml_parse_serverInfo.host = String (data);
            else if (strcasecmp (name, "id") == 0)
                __xml_parse_serverInfo.id = String (data).toInt ();
            else if (strcasecmp (name, "sponsor") == 0)
                __xml_parse_serverInfo.sponsor = String (data);
        }
    });
    auto &stream = client.getStream ();
    while (stream.available ())
        xml.processChar (stream.read ());
    client.end ();
    if (! __xml_parse_serverInfo.url.isEmpty ())
        insertServer (__xml_parse_serverInfo);
    return true;
}

void SpeedTest::sortServersByDistance (const ClientInfo &ipInfo) {
    for (auto &server : mServerList) {
        server.version = 0.0;
        server.latency = LONG_MAX;
        server.distance = harversine (std::make_pair (ipInfo.lat, ipInfo.lon), std::make_pair (server.lat, server.lon));
        auto client = SpeedTestClient (server.host);
        if (client.connect ()) {
            server.version = client.version ();
            long latency = 0;
            for (int i = 0; i < SPEED_TEST_LATENCY_EVAL_SIZE; i++)
                if (client.ping (latency) && latency < server.latency)
                    server.latency = latency;
            client.close ();
        }
        Serial.printf ("Server: %s %s by %s [distance %f km, version %f, latency %lu ms]\n", server.name.c_str (), server.host.c_str (), server.sponsor.c_str (), server.distance, server.version, server.latency);
    }
    std::sort (mServerList.begin (), mServerList.end (), [] (const ServerInfo &a, const ServerInfo &b) -> bool {
        return a.distance < b.distance;
    });
}

const ServerInfo SpeedTest::selectBestServer (const int sample_size, const cbFn &cb) {
    int samples_found = 0;
    ServerInfo bestServer = mServerList [0];
    long latency = INT_MAX;
    for (auto &server : mServerList) {
        if (server.version == 0.0) {
            if (cb)
                cb (false);
            continue;
        }
        if (server.version < mMinSupportedServer)
            continue;
        if (server.latency < latency) {
            latency = server.latency;
            bestServer = server;
        }
        if (cb)
            cb (true);
        if (++samples_found > sample_size)
            break;
    }
    return bestServer;
}

//

bool SpeedTest::downloadSpeed (const String &server, const TestConfig &config, double &result, const cbFn &cb) {
    result = execute (server, config, &SpeedTestClient::download, cb);
    return true;
}

bool SpeedTest::uploadSpeed (const String &server, const TestConfig &config, double &result, const cbFn &cb) {
    result = execute (server, config, &SpeedTestClient::upload, cb);
    return true;
}

bool SpeedTest::latencyAndJitter (const String &server, long &latency, long &jitter, const int sample) {
    auto client = SpeedTestClient (server);
    long previous_latency = LONG_MAX, minimum_latency = LONG_MAX;
    double current_jitter = 0;
    if (! client.connect ())
        return false;
    for (int i = 0; i < sample; i++) {
        long current_latency = 0;
        if (! client.ping (current_latency))
            return false;
        if (current_latency < minimum_latency)
            minimum_latency = current_latency;
        if (previous_latency == LONG_MAX)
            previous_latency = current_latency;
        else
            current_jitter += std::abs (previous_latency - current_latency);
    }
    client.close ();
    latency = minimum_latency;
    jitter = (long) std::floor (current_jitter / sample);
    return true;
}

double SpeedTest::execute (const String &server, const TestConfig &config, const opFn &op, const cbFn &cb) {
    std::vector<std::thread> workers;
    double overall_speed = 0;
    std::mutex mtx;
    for (int i = 0; i < config.concurrency; i++)
        workers.push_back (std::thread ([&server, &overall_speed, &op, &config, &mtx, cb] () {
            const size_t max_size = config.max_size;
            const size_t incr_size = config.incr_size;
            size_t curr_size = config.start_size;
            auto client = SpeedTestClient (server);
            if (client.connect ()) {
                auto start = std::chrono::steady_clock::now ();
                std::vector<double> partial_results;
                while (curr_size < max_size) {
                    long op_time = 0;
                    bool result = false;
                    if ((client.*op) (curr_size, config.buff_size, op_time)) {
                        partial_results.push_back ((curr_size * 8) / (static_cast<double> (op_time) / 1000));
                        result = true;
                    }
                    if (cb)
                        cb (result);
                    curr_size += incr_size;
                    if (std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - start).count () > config.min_test_time_ms)
                        break;
                }
                client.close ();
                std::sort (partial_results.begin (), partial_results.end ());
                size_t skip = 0, drop = 0, iter = 0;
                if (partial_results.size () >= 10) {
                    skip = partial_results.size () / 4;
                    drop = 2;
                }
                double real_sum = 0;
                for (auto it = partial_results.begin () + skip; it != partial_results.end () - drop; ++it) {
                    iter++;
                    real_sum += (*it);
                }
                mtx.lock ();
                overall_speed += (real_sum / iter);
                mtx.unlock ();
            } else {
                if (cb)
                    cb (false);
            }
        }));
    for (auto &worker : workers)
        worker.join ();
    return overall_speed / 1000 / 1000;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

const TestConfig preflightConfigDownload = {
    600000,     // start_size
    2000000,    // max_size
    125000,     // inc_size
    4096,       // buff_size
    10000,      // min_test_time_ms
    2,          // Concurrency
    "Preflight check"
};
const TestConfig slowConfigDownload = {
    100000,    // start_size
    500000,    // max_size
    10000,     // inc_size
    1024,      // buff_size
    20000,     // min_test_time_ms
    2,         // Concurrency
    "Very-slow-line line type detected: profile selected slowband"
};
const TestConfig slowConfigUpload = {
    50000,    // start_size
    80000,    // max_size
    1000,     // inc_size
    1024,     // buff_size
    20000,    // min_test_time_ms
    2,        // Concurrency
    "Very-slow-line line type detected: profile selected slowband"
};
const TestConfig narrowConfigDownload = {
    1000000,      // start_size
    100000000,    // max_size
    750000,       // inc_size
    4096,         // buff_size
    20000,        // min_test_time_ms
    2,            // Concurrency
    "Buffering-lover line type detected: profile selected narrowband"
};
const TestConfig narrowConfigUpload = {
    1000000,      // start_size
    100000000,    // max_size
    550000,       // inc_size
    4096,         // buff_size
    20000,        // min_test_time_ms
    2,            // Concurrency
    "Buffering-lover line type detected: profile selected narrowband"
};
const TestConfig broadbandConfigDownload = {
    1000000,      // start_size
    100000000,    // max_size
    750000,       // inc_size
    65536,        // buff_size
    20000,        // min_test_time_ms
    32,           // concurrency
    "Broadband line type detected: profile selected broadband"

};
const TestConfig broadbandConfigUpload = {
    1000000,     // start_size
    70000000,    // max_size
    250000,      // inc_size
    65536,       // buff_size
    20000,       // min_test_time_ms
    8,           // concurrency
    "Broadband line type detected: profile selected broadband"
};
const TestConfig fibreConfigDownload = {
    5000000,      // start_size
    120000000,    // max_size
    950000,       // inc_size
    65536,        // buff_size
    20000,        // min_test_time_ms
    32,           // concurrency
    "Fibre / Lan line type detected: profile selected fibre"
};
const TestConfig fibreConfigUpload = {
    1000000,     // start_size
    70000000,    // max_size
    250000,      // inc_size
    65536,       // buff_size
    20000,       // min_test_time_ms
    12,          // concurrency
    "Fibre / Lan line type detected: profile selected fibre"
};
void testConfigSelector (const double preSpeed, TestConfig &uploadConfig, TestConfig &downloadConfig) {
    uploadConfig = slowConfigUpload, downloadConfig = slowConfigDownload;
    if (preSpeed > 4 && preSpeed <= 30)
        downloadConfig = narrowConfigDownload, uploadConfig = narrowConfigUpload;
    else if (preSpeed > 30 && preSpeed < 150)
        downloadConfig = broadbandConfigDownload, uploadConfig = broadbandConfigUpload;
    else if (preSpeed >= 150)
        downloadConfig = fibreConfigDownload, uploadConfig = fibreConfigUpload;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

int speedTest (const String &server_specified) {

    auto sp = SpeedTest (SPEED_TEST_MIN_SERVER_VERSION);

    //

    if (! sp.identifyClient ()) {
        Serial.printf ("Unable to retrieve Client info. Try again later\n");
        return EXIT_FAILURE;
    }
    const auto clientInfo = sp.clientInfo ();
    Serial.printf ("Client: Address \"%s\", Provider \"%s\", Location (%f, %f)\n", clientInfo.address.c_str (), clientInfo.isp.c_str (), clientInfo.lat, clientInfo.lon);

    //

    String server;
    if (server_specified.isEmpty ()) {
        if (! sp.fetchServers ()) {
            Serial.printf ("Unable to retrieve Server list. Try again later\n");
            return EXIT_FAILURE;
        }
        Serial.printf ("%d Servers online\n", sp.numServers ());
        sp.sortServersByDistance (clientInfo);
        const auto serverInfo = sp.selectBestServer (10, [] (bool success) {
            Serial.printf ("%c", success ? '.' : '*');
        });
        Serial.printf ("\n");
        Serial.printf ("Server (closest): %s %s by %s (%f km distance)\n", serverInfo.name.c_str (), serverInfo.host.c_str (), serverInfo.sponsor.c_str (), serverInfo.distance);
        server = serverInfo.host;
    } else {
        sp.insertServer ({ .host = server_specified });
        Serial.printf ("Server (specified): %s\n", server_specified.c_str ());
        server = server_specified;
    }

    //

    long latency = 0, jitter = 0;
    if (sp.latencyAndJitter (server, latency, jitter))
        Serial.printf ("Latency: %lu ms, Jitter: %lu ms\n", latency, jitter);
    else
        Serial.printf ("Latency: unavailable, Jitter: unavailable\n");

    //

    Serial.printf ("Determining line type (%d): ", preflightConfigDownload.concurrency);
    double preSpeed = 0;
    if (sp.downloadSpeed (server, preflightConfigDownload, preSpeed, [] (bool success) {
            Serial.printf ("%c", success ? '.' : '*');
        })) {
        Serial.printf ("\n");
        Serial.printf ("Prespeed: %.2f Mbit/s\n", preSpeed);
    } else {
        Serial.printf ("\n");
        Serial.printf ("Prespeed test failed\n");
        return EXIT_FAILURE;
    }
    TestConfig uploadConfig, downloadConfig;
    testConfigSelector (preSpeed, uploadConfig, downloadConfig);
    Serial.printf ("%s\n", downloadConfig.label.c_str ());

    //

    Serial.printf ("Testing download speed (%d) ", downloadConfig.concurrency);
    double downloadSpeed = 0;
    if (sp.downloadSpeed (server, downloadConfig, downloadSpeed, [] (bool success) {
            Serial.printf ("%c", success ? '.' : '*');
        })) {
        Serial.printf ("\n");
        Serial.printf ("Download: %.2f Mbit/s\n", downloadSpeed);
    } else {
        Serial.printf ("\n");
        Serial.printf ("Download test failed\n");
        return EXIT_FAILURE;
    }

    //

    Serial.printf ("Testing upload speed (%d) ", uploadConfig.concurrency);
    double uploadSpeed = 0;
    if (sp.uploadSpeed (server, uploadConfig, uploadSpeed, [] (bool success) {
            Serial.printf ("%c", success ? '.' : '*');
        })) {
        Serial.printf ("\n");
        Serial.printf ("Upload: %.2f Mbit/s\n", uploadSpeed);
    } else {
        Serial.printf ("\n");
        Serial.printf ("Upload test failed\n");
        return EXIT_FAILURE;
    }

    //

    return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
