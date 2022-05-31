#ifndef PAYLOAD_H
#define PAYLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PAYLOAD_MAGIC_ADC 0x04ec38ed
#define PAYLOAD_MAGIC_BME 0x04ec38f1

typedef struct payload_adc {
  int16_t mVcc;
  int16_t mVbat;
  int16_t mVaccu;
  int16_t dCelsius;
} payload_adc_t;

typedef struct payload_bme {
  int16_t mVcc;
  int16_t mVbat;
  uint32_t mpHumi;
  uint32_t paPressure;
  int16_t ctCelsius;
} payload_bme_t;

typedef struct payload {
  uint32_t magic;
  uint32_t id;
  payload_bme_t data; // biggest data type
  uint32_t check;
} payload_t;

uint32_t payload_checksum(payload_t *p);
int payload_is_valid(payload_t *p);
void payload_make_valid(payload_t *p);

#ifdef __cplusplus
}
#endif

#endif
