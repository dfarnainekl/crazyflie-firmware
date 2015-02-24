/*
 * wiiMoteCam.h
 *
 *  Created on: Feb 22, 2015
 *      Author: Franky333
 */

#ifndef DRIVERS_INTERFACE_WIIMOTECAM_H_
#define DRIVERS_INTERFACE_WIIMOTECAM_H_


#define WMC_ADR 0x58
#define WMC_RES_X 1024
#define WMC_RES_Y 768

/*
http://makezine.com/2008/11/22/hacking-the-wiimote-ir-ca/
The author defined 5 sensitivity levels, and there are four parameters (p0, p1, p2, p3) that are adjusted for each level.
Here are the settings:

Level 1: p0 = 0×72, p1 = 0×20, p2 = 0x1F, p3 = 0×03
Level 2: p0 = 0xC8, p1 = 0×36, p2 = 0×35, p3 = 0×03
Level 3: p0 = 0xAA, p1 = 0×64, p2 = 0×63, p3 = 0×03
Level 4: p0 = 0×96, p1 = 0xB4, p2 = 0xB3, p3 = 0×04
Level 5: p0 = 0×96, p1 = 0xFE, p2 = 0xFE, p3 = 0×05

Quoting the Wiimote Wiki IR sensor page, these parameters correspond to:
p0: MAXSIZE: Maximum blob size. Wii uses values from 0×62 to 0xc8
p1: GAIN: Sensor Gain. Smaller values = higher gain
p2: GAINLIMIT: Sensor Gain Limit. Must be less than GAIN for camera to function. No other effect?
p3: MINSIZE: Minimum blob size. Wii uses values from 3 to 5
 */
#define WMC_MAXSIZE 0x72	//wiimote: 0x62-0xc8
#define WMC_GAIN 0x20		//smaller value-->higher gain
#define WMC_GAINLIMIT 0x1F	//must be less than gain (no effect?)
#define WMC_MINSIZE 0x03	//wiimote: 0x03-0x05

#define WMC_X_TO_ANGLE_FACTOR 0.039100684
#define WMC_Y_TO_ANGLE_FACTOR 0.039100684


struct WmcDot //raw data from wmc
{
    uint16_t x;
    uint16_t y;
    uint8_t s;
};

struct WmcDotAngle //x and y in degree, s raw from wmc
{
    float x;
    float y;
    uint8_t s;
};

uint8_t wmc_init_basic();
uint8_t wmc_init();
uint8_t wmc_readBlobs(struct WmcDot (*WMCDot)[4]);
uint8_t wmc_blobValid(struct WmcDot (*WMCDot));
void wmc_xyToAngle(struct WmcDot (*WMCDot), struct WmcDotAngle (*WMCDotAngle));


#endif /* DRIVERS_INTERFACE_WIIMOTECAM_H_ */
