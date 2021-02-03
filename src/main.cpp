#include <Arduino.h>

// Web Updater
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>

// Post to InfluxDB
#include <ESP8266HTTPClient.h>

// Infrastructure
#include <NTPClient.h>
#include <Syslog.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

// Lora
#include <rfm95.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

#define DIO0_PIN D8
#define DIO5_PIN D0
#define NSS_PIN D3
#define DB_LED_PIN D4
#define RF_LED_PIN D1

#define DB_LED_ON LOW
#define DB_LED_OFF HIGH
#define RF_LED_ON HIGH
#define RF_LED_OFF LOW

#define WEBSERVER_PORT 80

#define PAYLOAD_MAGIC 0x04ec38ed

ESP8266WebServer web_server(WEBSERVER_PORT);

ESP8266HTTPUpdateServer esp_updater;

// Post to InfluxDB
WiFiClient client;
HTTPClient http;
int influx_status = 0;

const uint32_t ok_interval = 5000;
const uint32_t err_interval = 1000;

uint32_t breathe_interval = ok_interval; // ms for one led breathe cycle

WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, NTP_SERVER);
static char start_time[30] = "";

WiFiUDP logUDP;
Syslog syslog(logUDP, SYSLOG_PROTO_IETF);

typedef struct payload {
  uint16_t magic;
  uint16_t mVcc;
  uint16_t mVbat;
  uint16_t mVaccu;
  uint32_t check;
} payload_t;

payload_t payload = {0};              // last received payload
signal_t signal = {0};                // last received signal quality
uint32_t last_counter_reset = 0;      // millis() of last counter reset
volatile uint32_t counter_events = 0; // events of current interval so far

ICACHE_RAM_ATTR void event() { counter_events++; }

// Post data to InfluxDB
void post_data() {
  static const char uri[] = "/write?db=" INFLUX_DB "&precision=s";

  char fmt[] =
      "payload,dev=" HOSTNAME ",ver=%s vcc=%.3f,vbat=%.3f,vaccu=%.3f,snr=%d,rssi=%d\n";
  char msg[sizeof(fmt) + 20 + 3 * 10];
  snprintf(msg, sizeof(msg), fmt, VERSION, payload.mVcc / 1000.0,
           payload.mVbat / 1000.0, payload.mVaccu / 1000.0, signal.snr,
           signal.rssi);
  http.begin(client, INFLUX_SERVER, INFLUX_PORT, uri);
  http.setUserAgent(PROGNAME);
  influx_status = http.POST(msg);
  String payload = http.getString();
  http.end();
  if (influx_status < 200 || influx_status > 299) {
    breathe_interval = err_interval;
    syslog.logf(LOG_ERR, "Post %s:%d%s status %d response '%s'", INFLUX_SERVER,
                INFLUX_PORT, uri, influx_status, payload.c_str());
  } else {
    breathe_interval = ok_interval;
  };
}

// Define web pages for update, reset or for event infos
void setup_webserver() {
  web_server.on("/payload", []() {
    static const char fmt[] = "{\n"
                              " \"meta\": {\n"
                              "  \"device\": \"" HOSTNAME "\",\n"
                              "  \"program\": \"" PROGNAME "\",\n"
                              "  \"version\": \"" VERSION "\",\n"
                              "  \"frequency\": 868500000,\n"
                              "  \"spreading\": 7,\n"
                              "  \"bandwidth\": 125,\n"
                              "  \"codingrate\": 0.8,\n"
                              "  \"started\": \"%s\",\n"
                              "  \"measured\": \"%s\"\n"
                              " },\n"
                              " \"voltage\": {\n"
                              "  \"vcc\": %u,\n"
                              "  \"vbat\": %u,\n"
                              "  \"vaccu\": %u\n"
                              " },\n"
                              " \"signal\": {\n"
                              "  \"snr\": %u,\n"
                              "  \"rssi\": %u\n"
                              " }\n"
                              "}\n";
    static char msg[sizeof(fmt) + 2 * 20 + 5 * 10];
    static char iso_time[30];
    strftime(iso_time, sizeof(iso_time), "%FT%T%Z",
             localtime(&events.measured));
    snprintf(msg, sizeof(msg), fmt, start_time, iso_time, payload.vcc,
             payload.vbat, payload.vacu, signal.snr, signal.rssi);
    web_server.send(200, "application/json", msg);
  });

  // Call this page to reset the ESP
  web_server.on("/reset", HTTP_POST, []() {
    syslog.log(LOG_NOTICE, "RESET");
    web_server.send(200, "text/html",
                    "<html>\n"
                    " <head>\n"
                    "  <title>" PROGNAME " v" VERSION "</title>\n"
                    "  <meta http-equiv=\"refresh\" content=\"7; url=/\"> \n"
                    " </head>\n"
                    " <body>Resetting...</body>\n"
                    "</html>\n");
    delay(200);
    ESP.restart();
  });

  // Standard page
  static const char fmt[] =
      "<html>\n"
      " <head>\n"
      "  <title>" PROGNAME " v" VERSION "</title>\n"
      "  <meta http-equiv=\"expires\" content=\"5\">\n"
      " </head>\n"
      " <body>\n"
      "  <h1>" PROGNAME " v" VERSION "</h1>\n"
      "  <table><tr>\n"
      "   <td><form action=\"payload\">\n"
      "    <input type=\"submit\" name=\"payload\" value=\"Payload as JSON\" />\n"
      "   </form></td>\n"
      "   <td><form action=\"reset\" method=\"post\">\n"
      "    <input type=\"submit\" name=\"reset\" value=\"Reset\" />\n"
      "   </form></td>\n"
      "  </tr></table>\n"
      "  <div>Post firmware image to /update<div>\n"
      "  <div>Influx status: %d<div>\n"
      " </body>\n"
      "</html>\n";
  static char page[sizeof(fmt) + 10] = "";

  // Index page
  web_server.on("/", []() {
    snprintf(page, sizeof(page), fmt, influx_status);
    web_server.send(200, "text/html", page);
  });

  // Catch all page
  web_server.onNotFound([]() {
    snprintf(page, sizeof(page), fmt, influx_status);
    web_server.send(404, "text/html", page);
  });

  web_server.begin();

  MDNS.addService("http", "tcp", WEBSERVER_PORT);
  syslog.logf(LOG_NOTICE, "Serving HTTP on port %d", WEBSERVER_PORT);
}

void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);

  pinMode(DB_LED_PIN, OUTPUT);
  digitalWrite(DB_LED_PIN, DB_LED_ON);
  analogWriteRange(PWMRANGE);

  Serial.begin(SERIAL_SPEED);
  Serial.println("\nStarting " PROGNAME " v" VERSION " " __DATE__ " " __TIME__);

  // Syslog setup
  syslog.server(SYSLOG_SERVER, SYSLOG_PORT);
  syslog.deviceHostname(HOSTNAME);
  syslog.appName("Joba1");
  syslog.defaultPriority(LOG_KERN);

  digitalWrite(DB_LED_PIN, DB_LED_OFF);

  WiFiManager wm;
  // wm.resetSettings();
  if (!wm.autoConnect()) {
    Serial.println("Failed to connect WLAN");
    for (int i = 0; i < 1000; i += 200) {
      digitalWrite(DB_LED_PIN, DB_LED_ON);
      delay(100);
      digitalWrite(DB_LED_PIN, DB_LED_OFF);
      delay(100);
    }
    ESP.restart();
    while (true)
      ;
  }

  digitalWrite(DB_LED_PIN, DB_LED_ON);
  char msg[80];
  snprintf(msg, sizeof(msg), "%s Version %s, WLAN IP is %s", PROGNAME, VERSION,
           WiFi.localIP().toString().c_str());
  Serial.printf(msg);
  syslog.logf(LOG_NOTICE, msg);

  ntp.begin();

  MDNS.begin(HOSTNAME);

  esp_updater.setup(&web_server);
  setup_webserver();

  last_counter_reset = millis();
  attachInterrupt(digitalPinToInterrupt(DIO0_PIN), event, RISING);
}

bool check_ntptime() {
  static bool have_time = false;
  if (!have_time && ntp.getEpochTime()) {
    have_time = true;
    time_t now = time(NULL);
    strftime(start_time, sizeof(start_time), "%FT%T%Z", localtime(&now));
    syslog.logf(LOG_NOTICE, "Booted at %s", start_time);
  }
  return have_time;
}

void on_interval_elapsed(uint32_t elapsed, uint32_t counts) {
  if( counts ) {
    uint8_t len = rfm95_getfifo(dev, buf, sizeof(buf));
    if( len == sizeof(payload) && payload.magic == PAYLOAD_MAGIC && verify_checksum(payload) ) {
      syslog.logf(LOG_INFO, "New payload after %u seconds: "
                  "Events CPM: 5s=%2u,  1m=%2u,  10m=%2u,  "
                  "1h=%2u,  1d=%2u,  RAW: "
                  "5s=%2u,  1m=%2u,  10m=%3u,  1h=%4u,  1d=%5u",
                  elapsed / 1000, );
      post_data();
    }
  } else {
    syslog.logf(LOG_NOTICE, "No payload since %u seconds", elapsed / 1000);
  }
}

void check_events() {
  static const uint32_t max_interval = 600*1000; // ms to wait for RF Rx

  uint32_t now = millis();
  uint32_t elapsed = now - last_counter_reset;
  if (counter_events || elapsed >= max_interval) {
    uint32_t events = counter_events;

    counter_events = 0;
    last_counter_reset += counter_interval;

    // either Rx happened or log interval elapsed
    on_interval_elapsed(elapsed, events);
  }
}

void breathe() {
  static uint32_t start = 0;
  static uint32_t max_duty = PWMRANGE / 2; // limit max brightness
  static uint32_t prev_duty = 0;

  uint32_t now = millis();
  uint32_t elapsed = now - start;
  if (elapsed > breathe_interval) {
    start = now;
    elapsed -= breathe_interval;
  }

  uint32_t duty = max_duty * elapsed * 2 / breathe_interval;
  if (duty > max_duty) {
    duty = 2 * max_duty - duty;
  }

  duty = duty * duty / max_duty;

  if (duty != prev_duty) {
    prev_duty = duty;
    analogWrite(DB_LED_PIN, PWMRANGE - duty);
  }
}

void loop() {
  ntp.update();
  if (check_ntptime()) {
    breathe();
  }
  check_events();
  web_server.handleClient();
  delay(1);
}
