#ifndef __I2C_OPTS_H__
#define __I2C_OPTS_H__


#define AT24C02_ADDR_RESERVE 			(1010 << 4)
#define AT24C02_ADDR_WRITE(SET_ADDR)	(AT24C02_ADDR_RESERVE + (SET_ADDR << 1) + 0x01)
#define AT24C02_ADDR_READ(SET_ADDR)		((AT24C02_ADDR_RESERVE + (SET_ADDR << 1)) + 0x00)

#define SCL_H	(PBout(6) = 1)
#define SCL_L	(PBout(6) = 0)
   
#define SDA_H	(PBout(7) = 1)
#define SDA_L	(PBout(7) = 0)

#define SCL_READ	PBin(6)
#define SDA_READ	PBin(7)


#endif






