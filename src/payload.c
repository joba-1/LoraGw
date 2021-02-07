#include <payload.h>

// checksum for payload
uint32_t payload_checksum(payload_t *p) {
  return p->magic + p->id + p->mVaccu + p->mVbat + p->mVcc + p->dCelsius;
}

int payload_is_valid(payload_t *p) { return payload_checksum(p) == p->check; }

void payload_make_valid(payload_t *p) {
  p->magic = PAYLOAD_MAGIC;
  p->check = payload_checksum(p);
}
