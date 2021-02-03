#include <payload.h>

// checksum for payload
uint16_t checksum(payload_t *p) {
  return p->magic + p->mVaccu + p->mVbat + p->mVcc;
}
