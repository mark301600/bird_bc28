#ifndef __NuEdu_Basic01_EEPROM_H__
#define __NuEdu_Basic01_EEPROM_H__
extern void I2C_EEPROM_Init(void);
extern void I2C_EEPROM_Write(uint16_t u16Address, uint8_t u8Data);
extern uint8_t I2C_EEPROM_Read(uint16_t u16Address);
extern void EEPROM_Write(uint32_t u16Address, char *DataBuffer,uint32_t count);
extern uint8_t EEPROM_Read(uint32_t u16Address, char *DataBuffer,uint32_t count);
#endif

