#ifndef PAYLOAD_H
#define PAYLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PAYLOAD_MAGIC 0x04ec38ed

typedef struct payload {
  uint32_t magic;
  uint32_t id;
  uint16_t mVcc;
  uint16_t mVbat;
  uint16_t mVaccu;
  uint16_t dCelsius;
  uint32_t check;
} payload_t;

uint32_t payload_checksum(payload_t *p);
int payload_is_valid(payload_t *p);
void payload_make_valid(payload_t *p);

#ifdef __cplusplus
}
#endif

#endif
