/*
 * xpt2046.h
 *
 *  Created on: May 10, 2025
 *      Author: Lenovo
 */

#ifndef INC_XPT2046_H_
#define INC_XPT2046_H_

#include "main.h"
#include <stdbool.h>

/*** Redefine if necessary ***/

// Warning! Use SPI bus with < 2.5 Mbit speed, better ~650 Kbit to be safe.
#define XPT2046_SPI_PORT hspi1
//extern SPI_HandleTypeDef XPT2046_SPI_PORT;

#define XPT2046_IRQ_Pin       TP_IRQ_Pin
#define XPT2046_IRQ_GPIO_Port TP_IRQ_GPIO_Port
#define XPT2046_CS_Pin        TP_CS_Pin
#define XPT2046_CS_GPIO_Port  TP_CS_GPIO_Port

// change depending on screen orientation
#define XPT2046_SCALE_X 480
#define XPT2046_SCALE_Y 320
//Values obtained in "rotated to landscape" (axes swapped)
#define XPT2046_MIN_RAW_X -31288
#define XPT2046_MAX_RAW_X -1816
#define XPT2046_MIN_RAW_Y -31032
#define XPT2046_MAX_RAW_Y -1235

#define READ_X 0xD0
#define READ_Y 0x90

//#define XPT2046_MIN_RAW_X 2000
//#define XPT2046_MAX_RAW_X 30000
//#define XPT2046_MIN_RAW_Y 1500
//#define XPT2046_MAX_RAW_Y 29000

//#define XTP2046_CALI_DIFF	2500

// call before initializing any SPI devices
void XPT2046_TouchUnselect(void);
void XPT2046_TouchSelect(void);
void XPT2046_TouchReadChannel(uint8_t chan, uint8_t * raw_data);
bool XPT2046_TouchPressed(void);
bool XPT2046_TouchGetCoordinates(int16_t* x, int16_t* y);
bool XPT2046_TouchGetRawCoordinates(int16_t* raw_, int16_t* raw_y);
bool XPT2046_TouchCalibration(void);

void SPI_Prescaler_Touch(void);
void SPI_Prescaler_LCD(void);


#endif /* INC_XPT2046_H_ */
