/*
 * gp2y0a60sz0f.h
 *
 *  Created on: 22.03.2015
 *      Author: Franky333
 */

#ifndef DRIVERS_INTERFACE_GP2Y0A60SZ0F_H_
#define DRIVERS_INTERFACE_GP2Y0A60SZ0F_H_


uint8_t gp2y0a60sz0f_init();
uint16_t gp2y0a60sz0f_getValue();
float gp2y0a60sz0f_valueToDistance(uint16_t value);


#endif /* DRIVERS_INTERFACE_GP2Y0A60SZ0F_H_ */
