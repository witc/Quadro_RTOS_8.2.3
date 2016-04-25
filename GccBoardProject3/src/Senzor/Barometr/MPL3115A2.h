/*
 * MPL3115A2.h
 *
 * Created: 19.04.2016 16:29:52
 *  Author: J
 */ 


#ifndef MPL3115A2_H_
#define MPL3115A2_H_

uint8_t Baro_send(unsigned char Adress, unsigned char *Data, unsigned char Length);
uint8_t Baro_read(unsigned char Adress, unsigned char *Data, unsigned char Length);
uint8_t Baro_get_preasure(uint8_t* data);
void Baro_init(void);
void toggleOneShot(void);
void Baro_Calibrate( float *init_altitude );


 #define MPL3115A2_ADDRESS                       (0x60)    // 1100000
 
  #define MPL3115A2_REGISTER_STATUS               (0x00)
  #define MPL3115A2_REGISTER_STATUS_TDR 0x02
  #define MPL3115A2_REGISTER_STATUS_PDR 0x04
  #define MPL3115A2_REGISTER_STATUS_PTDR 0x08

  #define MPL3115A2_REGISTER_PRESSURE_MSB         (0x01)
  #define MPL3115A2_REGISTER_PRESSURE_CSB         (0x02)
  #define MPL3115A2_REGISTER_PRESSURE_LSB         (0x03)

  #define MPL3115A2_REGISTER_TEMP_MSB             (0x04)
  #define MPL3115A2_REGISTER_TEMP_LSB             (0x05)

  #define MPL3115A2_REGISTER_DR_STATUS            (0x06)

  #define MPL3115A2_OUT_P_DELTA_MSB               (0x07)
  #define MPL3115A2_OUT_P_DELTA_CSB               (0x08)
  #define MPL3115A2_OUT_P_DELTA_LSB               (0x09)

  #define MPL3115A2_OUT_T_DELTA_MSB               (0x0A)
  #define MPL3115A2_OUT_T_DELTA_LSB               (0x0B)

  #define MPL3115A2_WHOAMI                        (0x0C)

  #define MPL3115A2_PT_DATA_CFG					0x13
  #define MPL3115A2_PT_DATA_CFG_TDEFE			0x01
  #define MPL3115A2_PT_DATA_CFG_PDEFE			0x02
  #define MPL3115A2_PT_DATA_CFG_DREM			0x04

  #define MPL3115A2_CTRL_REG1                     (0x26)
  #define MPL3115A2_CTRL_REG1_SBYB					0x01
  #define MPL3115A2_CTRL_REG1_OST					0x02
  #define MPL3115A2_CTRL_REG1_RST					0x04
  #define MPL3115A2_CTRL_REG1_OS1					0x00
  #define MPL3115A2_CTRL_REG1_OS2					0x08
  #define MPL3115A2_CTRL_REG1_OS4					0x10
  #define MPL3115A2_CTRL_REG1_OS8					0x18
  #define MPL3115A2_CTRL_REG1_OS16					0x20
  #define MPL3115A2_CTRL_REG1_OS32					0x28
  #define MPL3115A2_CTRL_REG1_OS64					0x30
  #define MPL3115A2_CTRL_REG1_OS128					0x38
  #define MPL3115A2_CTRL_REG1_RAW					0x40
  #define MPL3115A2_CTRL_REG1_ALT					0x80
  #define MPL3115A2_CTRL_REG1_BAR					0x00
  #define MPL3115A2_CTRL_REG2                     (0x27)
  #define MPL3115A2_CTRL_REG3                     (0x28)
  #define MPL3115A2_CTRL_REG4                     (0x29)
  #define MPL3115A2_CTRL_REG5                     (0x2A)

  #define MPL3115A2_REGISTER_STARTCONVERSION      (0x12)

#define STATUS     0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB  0x02
#define OUT_P_LSB  0x03
#define OUT_T_MSB  0x04
#define OUT_T_LSB  0x05
#define DR_STATUS  0x06
#define OUT_P_DELTA_MSB  0x07
#define OUT_P_DELTA_CSB  0x08
#define OUT_P_DELTA_LSB  0x09
#define OUT_T_DELTA_MSB  0x0A
#define OUT_T_DELTA_LSB  0x0B
#define WHO_AM_I   0x0C
#define F_STATUS   0x0D
#define F_DATA     0x0E
#define F_SETUP    0x0F
#define TIME_DLY   0x10
#define SYSMOD     0x11
#define INT_SOURCE 0x12
#define PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14
#define BAR_IN_LSB 0x15
#define P_TGT_MSB  0x16
#define P_TGT_LSB  0x17
#define T_TGT      0x18
#define P_WND_MSB  0x19
#define P_WND_LSB  0x1A
#define T_WND      0x1B
#define P_MIN_MSB  0x1C
#define P_MIN_CSB  0x1D
#define P_MIN_LSB  0x1E
#define T_MIN_MSB  0x1F
#define T_MIN_LSB  0x20
#define P_MAX_MSB  0x21
#define P_MAX_CSB  0x22
#define P_MAX_LSB  0x23
#define T_MAX_MSB  0x24
#define T_MAX_LSB  0x25
#define CTRL_REG1  0x26
#define CTRL_REG2  0x27
#define CTRL_REG3  0x28
#define CTRL_REG4  0x29
#define CTRL_REG5  0x2A
#define OFF_P      0x2B
#define OFF_T      0x2C
#define OFF_H      0x2D


#endif /* MPL3115A2_H_ */