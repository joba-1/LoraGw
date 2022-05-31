#include <payload.h>

// checksum for payload
uint32_t payload_checksum(payload_t *p) {
  payload_bme_t *bme;
  payload_adc_t *adc;

  uint32_t sum = p->magic + p->id;
  if( p->magic == PAYLOAD_MAGIC_ADC) {
      adc = (payload_adc_t *)&p->data;
      sum += adc->mVaccu + adc->mVbat + adc->mVcc + adc->dCelsius;
  }
  else {
      bme = (payload_bme_t *)&p->data;
      sum += bme->mVcc + bme->mVbat + bme->mpHumi + bme->paPressure + bme->ctCelsius;
  }
  return sum;
}

int payload_is_valid(payload_t *p) {
  uint8_t *check = (uint8_t *)&p->check;
  if(p->magic == PAYLOAD_MAGIC_ADC) {
    check += sizeof(payload_adc_t) - sizeof(payload_bme_t);
  } 
  return payload_checksum(p) == *(uint32_t *)check;
}

void payload_make_valid(payload_t *p) {
  if(p->magic != PAYLOAD_MAGIC_BME) {
    p->magic = PAYLOAD_MAGIC_ADC;
  }
  p->check = payload_checksum(p);
}
