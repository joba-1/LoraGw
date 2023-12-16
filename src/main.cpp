#include <Arduino.h>

// This is in Arduino.h, but I keep getting not defined???
#define PWMRANGE 1023

// Web Updater
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// Post to InfluxDB
#include <ESP8266HTTPClient.h>

// Publish via MQTT
#include <PubSubClient.h>

WiFiClient wifiMqtt;
PubSubClient mqtt(wifiMqtt);

// Infrastructure
#include <NTPClient.h>
#include <Syslog.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

// Lora
#include <SPI.h>
#include <rfm95.h>
#include <payload.h>

unsigned packets_total = 0;
unsigned packets_bme = 0;
unsigned packets_adc = 0;
unsigned packets_invalid = 0;

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
static char valid_time[30] = "";      // last received valid packet
signal_t signal = {0};                // last received signal quality
uint32_t last_message = 0;            // millis() of last counter reset
volatile uint32_t counter_events = 0; // events of current interval so far
rfm95_t rfm95_dev;


// Interrupt handler on received packages
IRAM_ATTR void event() { 
  counter_events++; 
}


// Post data to InfluxDB
void post_data() {
  static const char uri[] = "/write?db=" INFLUX_DB "&precision=s";

  char fmtBme[] = "payload,gw='" HOSTNAME "',ver=%s,dev=0x%08x"
                  " vcc=%.3f,vbat=%.3f,hpa=%.3f,humidity=%.3f,snr=%d,rssi=%d,celsius=%.1f\n";
  char fmtAdc[] = "payload,gw='" HOSTNAME "',ver=%s,dev=0x%08x"
                  " vcc=%.3f,vbat=%.3f,vaccu=%.3f,snr=%d,rssi=%d,celsius=%.1f\n";
  char msg[sizeof(fmtBme) + 20 + 3 * 10];
  payload_bme_t *bme;
  payload_adc_t *adc;

  if( payload.magic == PAYLOAD_MAGIC_ADC) {
    adc = (payload_adc_t *)&payload.data;
    snprintf(msg, sizeof(msg), fmtAdc, VERSION, payload.id, adc->mVcc / 1000.0,
             adc->mVbat / 1000.0, adc->mVaccu / 1000.0, signal.snr,
             signal.rssi, adc->dCelsius / 10.0);
  }
  else {
    bme = (payload_bme_t *)&payload.data;
    snprintf(msg, sizeof(msg), fmtBme, VERSION "b", payload.id, bme->mVcc / 1000.0,
             bme->mVbat / 1000.0, bme->paPressure / 100.0, bme->mpHumi / 1000.0, 
             signal.snr, signal.rssi, bme->ctCelsius / 100.0);
  }
  http.begin(client, INFLUX_SERVER, INFLUX_PORT, uri);
  http.setUserAgent(PROGNAME);
  influx_status = http.POST(msg);
  String resp = http.getString();
  http.end();
  if (influx_status < 200 || influx_status > 299) {
    breathe_interval = err_interval;
    syslog.logf(LOG_ERR, "Post %s:%d%s status %d response '%s'", INFLUX_SERVER,
                INFLUX_PORT, uri, influx_status, resp.c_str());
  } else {
    breathe_interval = ok_interval;
  };
}


// Publish packet to mqtt broker
void publish() {
   static const char fmtGen[] = "{\n"
     " \"meta\":{\"gateway\":\"" HOSTNAME "\",\"version\":\"" VERSION "\",\"device\":\"%08x\",\"started\":\"%s\"},\n"
     " \"packets\":{\"last\":\"%s\",\"total\":%u,\"bme\":%u,\"adc\":%u,\"invalid\":%u},\n"
     " \"signal\":{\"snr\":%d,\"rssi\":%d},\n"
     " \"data\":{\"vcc\":%.3f,\"vbat\":%.3f,";
  static const char fmtBme[] = "\"hpa\":%.3f,\"humidity\":%.3f,\"celsius\":%.1f}\n}";
  static const char fmtAdc[] = "\"vaccu\":%.3f}\n}";

  static char gen[sizeof(fmtGen) + 130];
  static char dat[sizeof(fmtBme) + 20];
  static char msg[sizeof(gen) + sizeof(dat)];

  if( payload.magic == PAYLOAD_MAGIC_ADC) {
    payload_adc_t *adc = (payload_adc_t *)&payload.data;
    snprintf(gen, sizeof(gen), fmtGen, payload.id, start_time, valid_time, 
      packets_total, packets_bme, packets_adc, packets_invalid,
      signal.snr, signal.rssi, adc->mVcc / 1000.0, adc->mVbat / 1000.0);
    snprintf(dat, sizeof(dat), fmtAdc, adc->mVaccu / 1000.0);
  }
  else {
    payload_bme_t *bme = (payload_bme_t *)&payload.data;
    snprintf(gen, sizeof(gen), fmtGen, payload.id, start_time, valid_time,
      packets_total, packets_bme, packets_adc, packets_invalid,
      signal.snr, signal.rssi, bme->mVcc / 1000.0, bme->mVbat / 1000.0);
    snprintf(dat, sizeof(dat), fmtBme, bme->paPressure / 100.0, bme->mpHumi / 1000.0, bme->ctCelsius / 100.0);
  }

  snprintf(msg, sizeof(msg), "%s%s", gen, dat);
  snprintf(gen, sizeof(gen), MQTT_TOPIC "/device/%08x", payload.id);

  if( !mqtt.publish(gen, msg) ) {
    syslog.logf(LOG_ERR, "Publish %s: '%s' to %s:%d failed", gen, msg, MQTT_BROKER, MQTT_PORT);
  }
}


// Maintain MQTT connection
void handle_mqtt() {
  static const int32_t interval = 5000;  // if disconnected try reconnect this often in ms
  static uint32_t prev = -interval;      // first connect attempt without delay
  static char msg[128];

  if (mqtt.connected()) {
    mqtt.loop();
  }
  else {
    uint32_t now = millis();
    if (now - prev > interval) {
      prev = now;

      if (mqtt.connect(HOSTNAME, MQTT_TOPIC "/LWT", 0, true, "Offline")
      && mqtt.publish(MQTT_TOPIC "/LWT", "Online", true)
      && mqtt.publish(MQTT_TOPIC "/Version", VERSION, true) ) {
        snprintf(msg, sizeof(msg), "Connected to MQTT broker %s:%d using topic %s", MQTT_BROKER, MQTT_PORT, MQTT_TOPIC);
        syslog.log(LOG_NOTICE, msg);
      }
      else {
        int error = mqtt.state();
        mqtt.disconnect();
        snprintf(msg, sizeof(msg), "Connect to MQTT broker %s:%d failed with code %d", MQTT_BROKER, MQTT_PORT, error);
        syslog.log(LOG_ERR, msg);
      }
    }
  }
}


// Define web pages for update, reset or for event infos
void setup_webserver() {
  web_server.on("/payload", []() {
    static const char fmtAdc[] = "  \"vcc\": %.3f,\n"
                                 "  \"vbat\": %.3f,\n"
                                 "  \"vaccu\": %.3f\n";
    static const char fmtBme[] = "  \"vcc\": %.3f,\n"
                                 "  \"vbat\": %.3f,\n"
                                 "  \"hpa\": %.3f,\n"
                                 "  \"humi\": %.3f,\n"
                                 "  \"celsius\": %.3f\n";
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
                              " \"data\": {\n"
                              "%s"
                              " },\n"
                              " \"signal\": {\n"
                              "  \"snr\": %d,\n"
                              "  \"rssi\": %d\n"
                              " }\n"
                              "}\n";
    static char msg[sizeof(fmt) + sizeof(fmtBme) + 2 * 20 + 5 * 10];
    static char data[sizeof(fmtBme) + 20];
    payload_bme_t *bme;
    payload_adc_t *adc;
    if( payload.magic == PAYLOAD_MAGIC_ADC) {
      adc = (payload_adc_t *)&payload.data;
      snprintf(data, sizeof(data), fmtAdc, adc->mVcc / 1000.0, 
               adc->mVbat / 1000.0, adc->mVaccu / 1000.0);
    }
    else {
      bme = (payload_bme_t *)&payload.data;
      snprintf(data, sizeof(data), fmtBme, bme->mVcc / 1000.0, bme->mVbat / 1000.0, 
               bme->paPressure / 100.0, bme->mpHumi / 1000.0, bme->ctCelsius / 100.0);
    }
    snprintf(msg, sizeof(msg), fmt, start_time, valid_time, payload.id, data,
             signal.snr, signal.rssi);
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
    "<!doctype html>\n"
    "<html lang=\"en\">\n"
    " <head>\n"
    "  <style> th, td { padding: 5px; } </style>\n"
    "  <meta charset=\"utf-8\">\n"
    "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
    "  <meta http-equiv=\"expires\" content=\"60\">\n"
    "  <link href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQAgMAAABinRfyAAAADFBMVEUqYbutnpTMuq/70SQgIef5AAAAVUlEQVQIHWOAAPkvDAyM3+Y7MLA7NV5g4GVqKGCQYWowYTBhapBhMGB04GE4/0X+M8Pxi+6XGS67XzzO8FH+iz/Dl/q/8gx/2S/UM/y/wP6f4T8QAAB3Bx3jhPJqfQAAAABJRU5ErkJggg==\" rel=\"icon\" type=\"image/x-icon\" />\n"
    "  <title>" PROGNAME " v" VERSION "</title>\n"
    " </head>\n"
    " <body>\n"
    "  <h1>" PROGNAME " v" VERSION "</h1>\n"
    "  <table>\n"
    "   <tr><td><form action=\"payload\">\n"
    "    <input type=\"submit\" name=\"payload\" value=\"Payload as JSON\" />\n"
    "   </form></td>\n"
    "   <td><form action=\"reset\" method=\"post\">\n"
    "    <input type=\"submit\" name=\"reset\" value=\"Reset\" />\n"
    "   </form></td></tr>\n"
    "   <tr><td>Post firmware image to</td><td><a href=\"http://" HOSTNAME "/update\" >http://" HOSTNAME "/update</a></td></tr>\n"
    "   <tr><td>Influx status</td><td>%d</td></tr>\n"
    "   <tr><td>MQTT status</td><td>%s</td></tr>\n"
    "   <tr><td>Started</td><td>%s</td></tr>\n"
    "   <tr><td>Last receive</td><td>%s</td></tr>\n"
    "   <tr><td>Last reload</td><td>%s</td></tr>\n"
    "   <tr><td><small>Author</small></td><td><a href=\"https://github.com/joba-1/LoraGw\" target=\"_blank\"><small>Joachim Banzhaf</small></a></td></tr>\n"
    "  </table>\n"
    " </body>\n"
    "</html>\n";
  static char page[sizeof(fmt) + 110] = "";

  // Index page
  web_server.on("/", []() {
    static char curr_time[30];
    time_t now;
    time(&now);
    strftime(curr_time, sizeof(curr_time), "%FT%T%Z", localtime(&now));
    const char *connected = mqtt.connected() ? "connected" : "disconnected";
    snprintf(page, sizeof(page), fmt, influx_status, connected, start_time, valid_time, curr_time);
    web_server.send(200, "text/html", page);
  });

  // Catch all page
  web_server.onNotFound([]() {
    static char curr_time[30];
    time_t now;
    time(&now);
    strftime(curr_time, sizeof(curr_time), "%FT%T%Z", localtime(&now));
    const char *connected = mqtt.connected() ? "connected" : "disconnected";
    snprintf(page, sizeof(page), fmt, influx_status, connected, start_time, valid_time, curr_time);
    web_server.send(404, "text/html", page);
  });

  web_server.begin();

  syslog.logf(LOG_NOTICE, "Serving HTTP on port %d", WEBSERVER_PORT);
}


int8_t duplexSpi(uint8_t dev, uint8_t addr, uint8_t *data, uint16_t len) {
  digitalWrite(NSS_PIN, LOW);
  SPI.transfer(addr);
  SPI.transfer(data, len);
  digitalWrite(NSS_PIN, HIGH);
  return 0;
}


uint8_t readPin(uint8_t dev) { 
  return (uint8_t)digitalRead(dev); 
}


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

  esp_updater.setup(&web_server);
  setup_webserver();

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  SPI.begin();
  rfm_setup(analogRead(A0) + WiFi.RSSI());
  last_message = millis();
  attachInterrupt(digitalPinToInterrupt(DIO0_PIN), event, RISING);
  digitalWrite(RF_LED_PIN, RF_LED_OFF);
}


bool check_ntptime() {
  static bool have_time = false;
  if (!have_time && ntp.getEpochTime() > (2000UL - 1970) * 365 * 24 * 60 * 60) {
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
  digitalWrite(RF_LED_PIN, RF_LED_ON);
  payload_t *p = (payload_t *)buf;
  payload_bme_t *bme;
  payload_adc_t *adc;

  packets_total++;
  if ((len == sizeof(payload))
   || (len == sizeof(payload) - sizeof(payload_bme_t) + sizeof(payload_adc_t))) {
    if ((p->magic == PAYLOAD_MAGIC_BME || p->magic == PAYLOAD_MAGIC_ADC) 
     && payload_is_valid(p)) {
      payload = *p;
      signal = sig;
      valid = true;
      time_t now;
      time(&now);
      strftime(valid_time, sizeof(valid_time), "%FT%T%Z", localtime(&now));
      if(p->magic == PAYLOAD_MAGIC_BME) {
        packets_bme++;
        bme = (payload_bme_t *)&p->data;
        syslog.logf(
          LOG_INFO, "Received from 0x%08x: mVcc=%d, mVbat=%d, pascal=%u, mpHumidity=%u, snr=%d, rssi=%d, ctCelsius=%d",
          payload.id, bme->mVcc, bme->mVbat, bme->paPressure, bme->mpHumi, signal.snr, signal.rssi, bme->ctCelsius);
      }
      else {
        packets_adc++;
        adc = (payload_adc_t *)&p->data;
        syslog.logf(
          LOG_INFO, "Received from 0x%08x: mVcc=%d, mVbat=%d, mVaccu=%d, snr=%d, rssi=%d, dCelsius=%d",
          payload.id, adc->mVcc, adc->mVbat, adc->mVaccu, signal.snr, signal.rssi, adc->dCelsius);
      }
      post_data();
      publish();
    } 
    else {
      packets_invalid++;
      syslog.logf(
        LOG_DEBUG, "Invalid payload from 0x%08x: magic=%x, snr=%d, rssi=%d",
        p->id, p->magic, signal.snr, signal.rssi);
    }
  } 
  else {
    packets_invalid++;
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
  static uint32_t silent_intervals = 0;

  uint32_t now = millis();
  bool valid = false;
  if (counter_events) {
    counter_events = 0;
    valid = handle_event();
    if (valid) {
      last_message = now;
      silent_intervals = 0;
    }
  }

  if (!valid) {
    uint32_t elapsed = now - last_message;
    if (elapsed >= max_interval) {
      last_message += max_interval;
      uint32_t silent_seconds = ((silent_intervals * max_interval) + elapsed) / 1000;
      syslog.logf(LOG_NOTICE, "No payload since %u seconds", silent_seconds);
      silent_intervals++;
      if( silent_intervals > 6 ) {
        syslog.logf(LOG_NOTICE, "Reboot in 1s due to no payloads");
        for (int i = 0; i < 1000; i += 100) {
          digitalWrite(DB_LED_PIN, DB_LED_ON);
          delay(50);
          digitalWrite(DB_LED_PIN, DB_LED_OFF);
          delay(50);
        }
        ESP.restart();
        while (true)
          ;
      }
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
  handle_mqtt();
  delay(1);
}
