
int16_t getTemp(uint16_t adc);
uint16_t getBoost(uint16_t adc);
uint16_t getOilTemp(uint16_t adc, uint8_t in_volt);
uint8_t calcThrtlPos(uint16_t adc);
int16_t trunc(int16_t temp);
uint16_t getSpeed(uint16_t timer);
uint16_t getVoltValue(uint16_t adc);
uint16_t getMAFValue(uint16_t adc);