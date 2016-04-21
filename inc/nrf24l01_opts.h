#ifndef __NRF24L01_OPTS_H__
#define __NRF24L01_OPTS_H__


#include "common_include.h"
#include "data_type.h"
#include "cfg_gpio.h"

#define OUT_PUT_MSB(byte) ((byte & 0x80) > 0x00) ? 1 : 0

#define NRF24L01_CE_I		PAout(2)
#define NRF24L01_CSN_I		PAout(4)
#define NRF24L01_SCK_I		PAout(5)
#define NRF24L01_MISO_I		PAin(6)
#define NRF24L01_MOSI_I		PAout(7)
#define NRF24L01_IRQ_I		PAin(1)

#define NRF24L01_CE_II		PBout(2)
#define NRF24L01_CSN_II		PBout(12)
#define NRF24L01_SCK_II		PBout(13)
#define NRF24L01_MISO_II	PBin(14)
#define NRF24L01_MOSI_II	PBout(15)
#define NRF24L01_IRQ_II		PBin(1)

////////// 相关命令的宏定义如下:

#define READ_REG_CMD		0x00 // Define read command to register
#define WRITE_REG_CMD		0x20 // Define write command to register
#define RD_RX_PLOAD 		0x61 // Define RX payload register address
#define WR_TX_PLOAD 		0xA0 // Define TX payload register address
#define FLUSH_TX 			0xE1 // Define flush TX register command
#define FLUSH_RX 			0xE2 // Define flush RX register command
#define REUSE_TX_PL 		0xE3 // Define reuse TX payload register command
#define NOP 				0xFF // Define No Operation, might be used to read status register


/////////// NRF24L01 相关寄存器地址的宏定义
#define CONFIG 				0x00 // 'Config' register address
#define EN_AA 				0x01 // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR 			0x02 // 'Enabled RX addresses' register address
#define SETUP_AW 			0x03 // 'Setup address width' register address
#define SETUP_RETR 			0x04 // 'Setup Auto. Retrans' register address
#define RF_CH 				0x05 // 'RF channel' register address
#define RF_SETUP 			0x06 // 'RF setup' register address
#define STATUS 				0x07 // 'Status' register address
#define OBSERVE_TX 			0x08 // 'Observe TX' register address
#define CD 					0x09 // 'Carrier Detect' register address
#define RX_ADDR_P0 			0x0A // 'RX address pipe0' register address
#define RX_ADDR_P1 			0x0B // 'RX address pipe1' register address
#define RX_ADDR_P2 			0x0C // 'RX address pipe2' register address
#define RX_ADDR_P3 			0x0D // 'RX address pipe3' register address
#define RX_ADDR_P4 			0x0E // 'RX address pipe4' register address
#define RX_ADDR_P5 			0x0F // 'RX address pipe5' register address
#define TX_ADDR 			0x10 // 'TX address' register address
#define RX_PW_P0 			0x11 // 'RX payload width, pipe0' register address
#define RX_PW_P1 			0x12 // 'RX payload width, pipe1' register address
#define RX_PW_P2 			0x13 // 'RX payload width, pipe2' register address
#define RX_PW_P3 			0x14 // 'RX payload width, pipe3' register address
#define RX_PW_P4 			0x15 // 'RX payload width, pipe4' register address
#define RX_PW_P5 			0x16 // 'RX payload width, pipe5' register address
#define FIFO_STATUS 		0x17 // 'FIFO Status Register' register address


#define TX_ADR_WIDTH    5   //5字节的地址宽度
#define RX_ADR_WIDTH    5   //5字节的地址宽度
#define TX_PLOAD_WIDTH  32  //20字节的用户数据宽度
#define RX_PLOAD_WIDTH  32  //20字节的用户数据宽度



void NRF24L01_GPIO_INIT(void);
void NRF24L01_TEST(void);


#endif






