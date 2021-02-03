#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdint.h>

typedef struct payload {
  uint16_t magic;
  uint16_t mVcc;
  uint16_t mVbat;
  uint16_t mVaccu;
  uint16_t check;
} payload_t;

uint16_t checksum(payload_t *p);

#endif
