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
#include <SPI.h>
#include <rfm95.h>
#include <payload.h>

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

payload_t payload = {0}; // last received payload
time_t payload_received;
signal_t signal = {0};                // last received signal quality
uint32_t last_message = 0;            // millis() of last counter reset
volatile uint32_t counter_events = 0; // events of current interval so far
rfm95_t rfm95_dev;

ICACHE_RAM_ATTR void event() { counter_events++; }

// Post data to InfluxDB
void post_data() {
  static const char uri[] = "/write?db=" INFLUX_DB "&precision=s";

  char fmt[] = "payload,gw='" HOSTNAME "',ver=%s,dev=0x%08x"
               " vcc=%.3f,vbat=%.3f,vaccu=%.3f,snr=%d,rssi=%d,celsius=%.1f\n";
  char msg[sizeof(fmt) + 20 + 3 * 10];
  snprintf(msg, sizeof(msg), fmt, VERSION, payload.id, payload.mVcc / 1000.0,
           payload.mVbat / 1000.0, payload.mVaccu / 1000.0, signal.snr,
           signal.rssi, payload.dCelsius / 10.0);
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
                              "  \"started\": \"%s\"\n"
                              " },\n"
                              " \"source\": {\n"
                              "  \"measured\": \"%s\",\n"
                              "  \"device\": %u\n"
                              " },\n"
                              " \"voltage\": {\n"
                              "  \"vcc\": %.3f,\n"
                              "  \"vbat\": %.3f,\n"
                              "  \"vaccu\": %.3f\n"
                              " },\n"
                              " \"signal\": {\n"
                              "  \"snr\": %u,\n"
                              "  \"rssi\": %u\n"
                              " }\n"
                              "}\n";
    static char msg[sizeof(fmt) + 2 * 20 + 5 * 10];
    static char iso_time[30];
    strftime(iso_time, sizeof(iso_time), "%FT%T%Z",
             localtime(&payload_received));
    snprintf(msg, sizeof(msg), fmt, start_time, iso_time, payload.id, payload.mVcc / 1000.0,
             payload.mVbat / 1000.0, payload.mVaccu / 1000.0, signal.snr,
             signal.rssi);
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
      "    <input type=\"submit\" name=\"payload\" value=\"Payload as JSON\" "
      "/>\n"
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

int8_t duplexSpi(uint8_t dev, uint8_t addr, uint8_t *data, uint16_t len) {
  digitalWrite(NSS_PIN, LOW);
  SPI.transfer(addr);
  SPI.transfer(data, len);
  digitalWrite(NSS_PIN, HIGH);
  return 0;
}

uint8_t readPin(uint8_t dev) { return (uint8_t)digitalRead(dev); }

void rfm_setup(uint32_t seed) {
  uint8_t rfm95_ver = 0;

  pinMode(DIO0_PIN, INPUT);
  pinMode(DIO5_PIN, INPUT);
  pinMode(NSS_PIN, OUTPUT);

  rfm95_dev.nss_pin_id = NSS_PIN;
  rfm95_dev.dio0_pin_id = DIO0_PIN;
  rfm95_dev.dio5_pin_id = DIO5_PIN;
  rfm95_dev.spi_write = duplexSpi;
  rfm95_dev.spi_read = duplexSpi;
  rfm95_dev.delay = (rfm95_delay_fptr_t)delay;
  rfm95_dev.pin_read = readPin;

  while (rfm95_ver != 0x12) {
    rfm95_ver = rfm95_init(&rfm95_dev, seed);
  }
  rfm95_recv(&rfm95_dev);
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

  pinMode(RF_LED_PIN, OUTPUT);
  digitalWrite(RF_LED_PIN, RF_LED_ON);

  char msg[80];
  snprintf(msg, sizeof(msg), "%s Version %s, WLAN IP is %s", PROGNAME, VERSION,
           WiFi.localIP().toString().c_str());
  Serial.printf(msg);
  syslog.logf(LOG_NOTICE, msg);

  ntp.begin();

  MDNS.begin(HOSTNAME);

  esp_updater.setup(&web_server);
  setup_webserver();

  SPI.begin();
  rfm_setup(analogRead(A0) + WiFi.RSSI());
  last_message = millis();
  attachInterrupt(digitalPinToInterrupt(DIO0_PIN), event, RISING);
  digitalWrite(RF_LED_PIN, RF_LED_OFF);
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

bool handle_event() {
  bool valid = false;
  uint8_t buf[0xff];
  signal_t sig;
  uint8_t len = rfm95_fifo(&rfm95_dev, buf, sizeof(buf), &sig);
  // rfm95_recv(&rfm95_dev); // TODO: needed?
  digitalWrite(RF_LED_PIN, RF_LED_ON);
  if (len == sizeof(payload)) {
    payload_t *p = (payload_t *)buf;
    if (p->magic == PAYLOAD_MAGIC && payload_checksum(p) == p->check) {
      payload_received = time(NULL);
      payload = *p;
      signal = sig;
      valid = true;
      syslog.logf(
          LOG_INFO, "Received from 0x%08x: mVcc=%u, mVbat=%u, mVaccu=%u, snr=%d, rssi=%d, dcelsius=%u",
          payload.id, payload.mVcc, payload.mVbat, payload.mVaccu, signal.snr, signal.rssi, payload.dCelsius);
      post_data();
    } else {
      syslog.logf(LOG_DEBUG,
                  "Invalid from 0x%08x: mVcc=%u, mVbat=%u, mVaccu=%u, snr=%d, "
                  "rssi=%d, dcelsius=%u",
                  payload.id, payload.mVcc, payload.mVbat, payload.mVaccu,
                  signal.snr, signal.rssi, payload.dCelsius);
    }
  } else {
    if( len == 4 ) {
      syslog.logf(LOG_DEBUG, "Received old packet %u: %u mV",
                  buf[3] * 256 + buf[2], buf[1] * 256 + buf[0]);
    }
    else {
      syslog.logf(LOG_DEBUG, "Received packet with wrong length %u", len);
    }
  }
  digitalWrite(RF_LED_PIN, RF_LED_OFF);
  return valid;
}

void check_events() {
  static const uint32_t max_interval = 600 * 1000; // ms to wait for RF Rx

  uint32_t now = millis();
  bool valid = false;
  if (counter_events) {
    counter_events = 0;
    valid = handle_event();
    if (valid) {
      last_message = now;
    }
  }

  if (!valid) {
    uint32_t elapsed = now - last_message;
    if (elapsed >= max_interval) {
      last_message += max_interval;
      syslog.logf(LOG_NOTICE, "No payload since %u seconds", elapsed / 1000);
    }
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
