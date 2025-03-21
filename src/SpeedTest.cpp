
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
static inline constexpr T deg2rad (const T n) {
    return (n * M_PI / 180);
}
template <typename T>
static inline constexpr T harversine (const std::pair<T, T> n1, const std::pair<T, T> n2) {
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
    auto buff = std::unique_ptr<uint8_t []> (new uint8_t [buff_size]);
    if (! buff)
        return false;
    const auto start = std::chrono::steady_clock::now ();
    size_t recv_size = 0;
    while (recv_size < total_size) {
        const size_t left_size = std::min (buff_size, total_size - recv_size);
        if (! read (buff.get (), left_size))
            return false;
        recv_size += left_size;
    }
    millisec = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - start).count ();
    return true;
}

bool SpeedTestClient::upload (const size_t total_size, const size_t buff_size, long &millisec) {
    String reply;
    int offset;
    const String command = String ("UPLOAD ") + String (total_size) + "\n";
    if (! writeLine (command))
        return false;
    auto buff = std::unique_ptr<uint8_t []> (new uint8_t [buff_size]);
    for (size_t i = 0; i < buff_size; i++)
        buff [i] = static_cast<uint8_t> (rand () % 256);
    const auto start = std::chrono::steady_clock::now ();
    size_t send_size = command.length ();
    while (send_size < total_size) {
        const size_t left_size = total_size - send_size;
        if (left_size > buff_size) {
            if (! write (buff.get (), buff_size))
                return false;
            send_size += buff_size;
        } else {
            buff [left_size - 1] = '\n';
            if (! write (buff.get (), left_size))
                return false;
            send_size += left_size;
        }
    }
    if (! readLine (reply) || ! reply.startsWith ("OK ") || (offset = reply.indexOf (' ')) < 0) {
        Serial.printf ("upload faulty response: %s\n", reply.c_str ());
        return false;
    }
    millisec = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - start).count ();
    return reply.substring (offset + 1, reply.indexOf (' ', offset + 1)).toInt () == total_size;
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
        mStats.bytesReceived += n;
    }
    return true;
}

bool SpeedTestClient::write (const uint8_t *buffer, const size_t length) {
    if (! mClient.connected ())
        return false;
    if (length <= 0)
        return false;
    const size_t n = mClient.write (buffer, length);
    if (n > 0)
        mStats.bytesTransmitted += n;
    return n == length;
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
        mStats.bytesReceived += n;
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
    const size_t n = mClient.write (buffer.c_str ());
    if (n > 0)
        mStats.bytesTransmitted += n;
    if (n != buffer.length ())
        return false;
    if (buffer.indexOf ('\n') < 0 && mClient.write ("\n") != 1)
        return false;
    return true;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

SpeedTest::SpeedTest (const float minServerVersion) :
    mMinSupportedServer (minServerVersion) {
}

bool SpeedTest::identifyClient (const String &url) {
    HTTPClient client;
    client.setUserAgent (SPEED_TEST_USER_AGENT);
    client.setFollowRedirects (HTTPC_FORCE_FOLLOW_REDIRECTS);
    client.begin (url);
    int code;
    if ((code = client.GET ()) != HTTP_CODE_OK) {
        Serial.printf ("identifyClient: HTTP GET error-code=%d\n", code);
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
        return false;
    }
    TinyXML xml;
    uint8_t buffer [256];
    static SpeedTest *__xml_parse_speedTest;
    static ServerInfo __xml_parse_serverInfo;
    __xml_parse_serverInfo = ServerInfo ();
    __xml_parse_speedTest = this;
    xml.init (buffer, sizeof (buffer), [] (uint8_t flags, char *name, uint16_t nameLen, char *data, uint16_t dataLen) {
        if (name == nullptr || data == nullptr)
            return;
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
    long latency = LONG_MAX;
    for (const auto &server : mServerList) {
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

bool SpeedTest::latency (const String &server, long &latency, long &jitter, const int sample_size, const cbFn &cb) {
    auto client = SpeedTestClient (server);
    long previous_latency = LONG_MAX, minimum_latency = LONG_MAX;
    double current_jitter = 0;
    if (! client.connect ())
        return false;
    for (int i = 0; i < sample_size; i++) {
        bool result = false;
        long current_latency = 0;
        if (client.ping (current_latency)) {
            if (current_latency < minimum_latency)
                minimum_latency = current_latency;
            if (previous_latency == LONG_MAX)
                previous_latency = current_latency;
            else
                current_jitter += std::abs (previous_latency - current_latency);
            result = true;
        }
        if (cb)
            cb (result);
    }
    client.close ();
    latency = minimum_latency;
    jitter = (long) std::floor (current_jitter / sample_size);
    return true;
}

double SpeedTest::execute (const String &server, const TestConfig &config, const opFn &op, const cbFn &cb) {
    std::vector<std::thread> workers;
    double overall_speed = 0;
    std::mutex mtx;
    for (int i = 0; i < config.concurrency; i++)
        workers.push_back (std::thread ([this, &server, &overall_speed, &op, &config, &mtx, cb] () {
            const size_t max_size = config.max_size;
            const size_t incr_size = config.incr_size;
            size_t curr_size = config.start_size;
            auto client = SpeedTestClient (server);
            if (client.connect ()) {
                auto start = std::chrono::steady_clock::now ();
                std::vector<double> partial_results;
                while (curr_size < max_size && std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () - start).count () < config.min_test_time_ms) {
                    long op_time = 0;
                    bool result = false;
                    if ((client.*op) (curr_size, config.buff_size, op_time)) {
                        partial_results.push_back ((curr_size * 8) / (static_cast<double> (op_time) / 1000));
                        result = true;
                    }
                    if (cb)
                        cb (result);
                    curr_size += incr_size;
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

                const std::lock_guard<std::mutex> lock (mtx);
                overall_speed += (real_sum / iter);
                this->mStats += client.stats ();
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

const TestConfig preflightConfig = {
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

bool testConfigSelector (const String &profile, TestConfig &uploadConfig, TestConfig &downloadConfig) {
    if (profile == SpeedTestProfile::PREFLIGHT)
        uploadConfig = preflightConfig, downloadConfig = preflightConfig;
    else if (profile == SpeedTestProfile::SLOW)
        uploadConfig = slowConfigUpload, downloadConfig = slowConfigDownload;
    else if (profile == SpeedTestProfile::NARROWBAND)
        downloadConfig = narrowConfigDownload, uploadConfig = narrowConfigUpload;
    else if (profile == SpeedTestProfile::BROADBAND)
        downloadConfig = broadbandConfigDownload, uploadConfig = broadbandConfigUpload;
    else if (profile == SpeedTestProfile::FIBRE)
        downloadConfig = fibreConfigDownload, uploadConfig = fibreConfigUpload;
    else
        return false;
    return true;
}
String testProfileSelector (const double preSpeed) {
    if (preSpeed <= 4)
        return SpeedTestProfile::SLOW;
    if (preSpeed > 4 && preSpeed <= 30)
        return SpeedTestProfile::NARROWBAND;
    else if (preSpeed > 30 && preSpeed < 150)
        return SpeedTestProfile::BROADBAND;
    else if (preSpeed >= 150)
        return SpeedTestProfile::FIBRE;
    /*NOT_REACHED*/
    return {};
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

bool speedTest (const SpeedTestConfig &config) {

    auto progressCallback = [] (bool success) {
        Serial.printf ("%c", success ? '.' : '*');
    };

    String configOptions;
    for (auto &[name, value] : config)
        configOptions += (configOptions.isEmpty () ? "" : ", ") + name + "=" + value;
    Serial.printf ("Config: %s\n", configOptions.c_str ());

    auto sp = SpeedTest (SPEED_TEST_MIN_SERVER_VERSION);

    //

    if (! sp.identifyClient (SPEED_TEST_IP_INFO_API_URL)) {
        Serial.printf ("Unable to retrieve Client info. Try again later\n");
        return false;
    }
    const auto clientInfo = sp.clientInfo ();
    Serial.printf ("Client: Address \"%s\", Provider \"%s\", Location (%f, %f)\n", clientInfo.address.c_str (), clientInfo.isp.c_str (), clientInfo.lat, clientInfo.lon);

    //

    String server;
    if (config.find ("server") == config.end ()) {
        if (! sp.fetchServers (SPEED_TEST_SERVER_LIST_URL)) {
            Serial.printf ("Unable to retrieve Server list. Try again later\n");
            return false;
        }
        Serial.printf ("%d Servers online\n", sp.numServers ());
        sp.sortServersByDistance (clientInfo);
        const auto serverInfo = sp.selectBestServer (SPEED_TEST_SERVER_SELECT_SAMPLE, progressCallback);
        Serial.printf ("\nServer (closest): %s %s by %s (%f km distance)\n", serverInfo.name.c_str (), serverInfo.host.c_str (), serverInfo.sponsor.c_str (), serverInfo.distance);
        server = serverInfo.host;
    } else {
        const auto server_specified = config.at ("server");
        sp.insertServer ({ .host = server_specified });
        Serial.printf ("Server (specified): %s\n", server_specified.c_str ());
        server = server_specified;
    }

    //

    long latency = 0, jitter = 0;
    Serial.printf ("Testing latency/jitter ");
    if (! sp.latency (server, latency, jitter, SPEED_TEST_LATENCY_SAMPLE_SIZE, progressCallback)) {
        Serial.printf ("Latency/Jitter: test failed\n");
        return false;
    }
    Serial.printf ("\nLatency: %lu ms, Jitter: %lu ms\n", latency, jitter);

    //

    TestConfig uploadConfig, downloadConfig;
    if (config.find ("profile") == config.end ()) {
        double preflightSpeed = 0;
        Serial.printf ("Determining line type (%d): ", preflightConfig.concurrency);
        if (! sp.downloadSpeed (server, preflightConfig, preflightSpeed, progressCallback)) {
            Serial.printf ("\nPreflight test failed\n");
            return false;
        }
        const String profile = testProfileSelector (preflightSpeed);
        Serial.printf ("\nPreflight: %.2f Mbit/s, qualifies as %s\n", preflightSpeed, profile.c_str ());
        if (! testConfigSelector (profile, uploadConfig, downloadConfig))
            return false;
    } else {
        const auto profile_specified = config.at ("profile");
        Serial.printf ("Profile (specified): %s\n", profile_specified.c_str ());
        if (! testConfigSelector (profile_specified, uploadConfig, downloadConfig))
            return false;
    }
    Serial.printf ("%s\n", downloadConfig.label.c_str ());

    //

    double downloadSpeed = 0;
    Serial.printf ("Testing download speed (%d) ", downloadConfig.concurrency);
    if (! sp.downloadSpeed (server, downloadConfig, downloadSpeed, progressCallback)) {
        Serial.printf ("\nDownload test failed\n");
        return false;
    }
    Serial.printf ("\nDownload: %.2f Mbit/s\n", downloadSpeed);

    //

    double uploadSpeed = 0;
    Serial.printf ("Testing upload speed (%d) ", uploadConfig.concurrency);
    if (! sp.uploadSpeed (server, uploadConfig, uploadSpeed, progressCallback)) {
        Serial.printf ("\nUpload test failed\n");
        return false;
    }
    Serial.printf ("\nUpload: %.2f Mbit/s\n", uploadSpeed);

    const auto stats = sp.stats ();
    Serial.printf ("Stats: %.2fMb/%.2fMb transmitted/received\n", stats.bytesTransmitted / 1024.0 / 1024.0, stats.bytesReceived / 1024.0 / 1024.0);

    //

    return true;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
