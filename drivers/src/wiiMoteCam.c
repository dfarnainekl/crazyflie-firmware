/*
 * wiiMoteCam.c
 *
 *  Created on: Feb 22, 2015
 *      Author: Franky333
 */

#define DEBUG_MODULE "WIIMOTECAM"

#include "stm32fxxx.h"
// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"
#include "i2cdev.h"

#include "wiiMoteCam.h"

//initializes the wmc with basic (unknown) settings
uint8_t wmc_init_basic()
{
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x30, 0x01)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x30, 0x08)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x06, 0x90)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x08, 0xC0)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x1A, 0x40)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x33, 0x33)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	DEBUG_PRINT("I2C connection [OK].\n");
	return 1;
}

//initializes the wmc with settings as defined in wiiMoteCam.h
uint8_t wmc_init()
{
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x30, 0x01)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x00,0x02)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x01,0x00)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x02,0x01)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x03,0x71)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x04,0x01)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x05,0x00)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x06,WMC_MAXSIZE)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x07,0x00)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x08,WMC_GAIN)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x1A,WMC_GAINLIMIT)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x1A,WMC_MINSIZE)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x33,0x03)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, 0x30,0x08)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	DEBUG_PRINT("I2C connection [OK].\n");
	return 1;
}

//reads all four blobs from the wmc, isVisible flag is updated every time, x y s x_angle and y_angle only when blob is visible
uint8_t wmc_readBlobs(struct WmcBlob (*WMCBlob)[4])
{
	if(!i2cdevWriteByte(I2C1_DEV, WMC_ADR, I2CDEV_NO_MEM_ADDR, 0x37)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	uint8_t buf[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	if(!i2cdevRead(I2C1_DEV, WMC_ADR, I2CDEV_NO_MEM_ADDR, 12, buf)) { DEBUG_PRINT("I2C connection [FAIL].\n"); return 0; }

	struct WmcBlob wmcBlobs[4];

	wmcBlobs[0].x=buf[0]+((buf[2] & 0x30)<<4);
	wmcBlobs[0].y=buf[1]+((buf[2] & 0xC0) <<2);
	wmcBlobs[0].s=(buf[2] & 0x0F);

	wmcBlobs[1].x=buf[3]+((buf[5] & 0x30) <<4);
	wmcBlobs[1].y=buf[4]+((buf[5] & 0xC0) <<2);
	wmcBlobs[1].s=(buf[5] & 0x0F);

	wmcBlobs[2].x=buf[6]+((buf[8] & 0x30) <<4);
	wmcBlobs[2].y=buf[7]+((buf[8] & 0xC0) <<2);
	wmcBlobs[2].s=(buf[8] & 0x0F);

	wmcBlobs[3].x=buf[9]+((buf[11] & 0x30) <<4);
	wmcBlobs[3].y=buf[10]+((buf[11] & 0xC0) <<2);
	wmcBlobs[3].s=(buf[11] & 0x0F);

	uint8_t i;
	for(i=0; i<4; i++)
	{
		if(wmcBlobs[i].y <= 767) //blob is visible/valid
		{
			(*WMCBlob)[i].isVisible = 1;
			(*WMCBlob)[i].s = wmcBlobs[i].s;
			(*WMCBlob)[i].x = wmcBlobs[i].x;
			(*WMCBlob)[i].y = wmcBlobs[i].y;
			(*WMCBlob)[i].x_angle = ((float)wmcBlobs[i].x - ((float)WMC_RES_X - 1) / 2) * (float)WMC_X_TO_ANGLE_FACTOR;
			(*WMCBlob)[i].y_angle = ((float)wmcBlobs[i].y - ((float)WMC_RES_Y - 1) / 2) * (float)WMC_Y_TO_ANGLE_FACTOR;
		}
		else (*WMCBlob)[i].isVisible = 0;
	}

	return 1;
}
