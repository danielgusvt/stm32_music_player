/*****************************************************************************
 * | File      	:	LCD_Driver.c
 * | Author      :   Waveshare team
 * | Function    :	ILI9486 Drive function
 * | Info        :
 *   Image scanning
 *      Please use progressive scanning to generate images or fonts
 *----------------
 * |	This version:   V1.0
 * | Date        :   2018-01-11
 * | Info        :   Basic version
 *
 ******************************************************************************/

/**************************Intermediate driver layer**************************/
#include "ILI9486.h"

LCD_DIS sLCD_DIS;

void PWM_SetValue(uint16_t value) {
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/*******************************************************************************
 function:
 Hardware reset
 *******************************************************************************/
void Driver_Delay_ms(unsigned long xms) {
	HAL_Delay(xms);

}

void Driver_Delay_us(int xus) {
	for (int j = xus; j > 0; j--)
		;
}

void LCD_Reset(void) {
	LCD_RST_1
	;
	Driver_Delay_ms(500);
	LCD_RST_0
	;
	Driver_Delay_ms(500);
	LCD_RST_1
	;
	Driver_Delay_ms(500);
}

static void LCD_SetBackLight(uint16_t value) {
	PWM_SetValue(value);
}

/*******************************************************************************
 function:
 Write register address and data
 *******************************************************************************/
void LCD_WriteReg(uint8_t Reg) {
	LCD_DC_0
	;
	LCD_CS_0
	;
	HAL_SPI_Transmit(&hspi1, &Reg, 1U, 100U);
	LCD_CS_1
	;
}

void LCD_WriteData(uint8_t Data) {
	LCD_DC_1
	;
	LCD_CS_0
	;
	HAL_SPI_Transmit(&hspi1, &Data, 1U, 100U);
	LCD_CS_1
	;
}

/*******************************************************************************
 function:
 Write register data
 *******************************************************************************/
void LCD_Write_AllData(uint16_t Data, uint32_t DataLen) {
	uint32_t i;
	LCD_DC_1;
	LCD_CS_0;
	for (i = 0; i < DataLen; i++) {
		uint8_t data[2];
		data[0] = Data >> 8;
		data[1] = Data;

		HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 2, 1000);
	}
	LCD_CS_1;
}

/*******************************************************************************
 function:
 Common register initialization
 *******************************************************************************/
static void LCD_InitReg(void) {
	LCD_WriteReg(0XF9);
	LCD_WriteData(0x00);
	LCD_WriteData(0x08);

	LCD_WriteReg(0xC0);
	LCD_WriteData(0x19); //VREG1OUT POSITIVE
	LCD_WriteData(0x1a); //VREG2OUT NEGATIVE

	LCD_WriteReg(0xC1);
	LCD_WriteData(0x45); //VGH,VGL    VGH>=14V.
	LCD_WriteData(0x00);

	LCD_WriteReg(0xC2);	//Normal mode, increase can change the display quality, while increasing power consumption
	LCD_WriteData(0x33);

	LCD_WriteReg(0XC5);
	LCD_WriteData(0x00);
	LCD_WriteData(0x28);	//VCM_REG[7:0]. <=0X80.

	LCD_WriteReg(0xB1);	//Sets the frame frequency of full color normal mode
	LCD_WriteData(0xA0);	//0XB0 =70HZ, <=0XB0.0xA0=62HZ
	LCD_WriteData(0x11);

	LCD_WriteReg(0xB4);
	LCD_WriteData(0x02); //2 DOT FRAME MODE,F<=70HZ.

	LCD_WriteReg(0xB6); //
	LCD_WriteData(0x00);
	LCD_WriteData(0x42); //0 GS SS SM ISC[3:0];
	LCD_WriteData(0x3B);

	LCD_WriteReg(0xB7);
	LCD_WriteData(0x07);

	LCD_WriteReg(0xE0);
	LCD_WriteData(0x1F);
	LCD_WriteData(0x25);
	LCD_WriteData(0x22);
	LCD_WriteData(0x0B);
	LCD_WriteData(0x06);
	LCD_WriteData(0x0A);
	LCD_WriteData(0x4E);
	LCD_WriteData(0xC6);
	LCD_WriteData(0x39);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);

	LCD_WriteReg(0XE1);
	LCD_WriteData(0x1F);
	LCD_WriteData(0x3F);
	LCD_WriteData(0x3F);
	LCD_WriteData(0x0F);
	LCD_WriteData(0x1F);
	LCD_WriteData(0x0F);
	LCD_WriteData(0x46);
	LCD_WriteData(0x49);
	LCD_WriteData(0x31);
	LCD_WriteData(0x05);
	LCD_WriteData(0x09);
	LCD_WriteData(0x03);
	LCD_WriteData(0x1C);
	LCD_WriteData(0x1A);
	LCD_WriteData(0x00);

	LCD_WriteReg(0XF1);
	LCD_WriteData(0x36);
	LCD_WriteData(0x04);
	LCD_WriteData(0x00);
	LCD_WriteData(0x3C);
	LCD_WriteData(0x0F);
	LCD_WriteData(0x0F);
	LCD_WriteData(0xA4);
	LCD_WriteData(0x02);

	LCD_WriteReg(0XF2);
	LCD_WriteData(0x18);
	LCD_WriteData(0xA3);
	LCD_WriteData(0x12);
	LCD_WriteData(0x02);
	LCD_WriteData(0x32);
	LCD_WriteData(0x12);
	LCD_WriteData(0xFF);
	LCD_WriteData(0x32);
	LCD_WriteData(0x00);

	LCD_WriteReg(0XF4);
	LCD_WriteData(0x40);
	LCD_WriteData(0x00);
	LCD_WriteData(0x08);
	LCD_WriteData(0x91);
	LCD_WriteData(0x04);

	LCD_WriteReg(0XF8);
	LCD_WriteData(0x21);
	LCD_WriteData(0x04);

	LCD_WriteReg(0X3A);	//Set Interface Pixel Format
	LCD_WriteData(0x55);

}

/********************************************************************************
 function:	Set the display scan and color transfer modes
 parameter:
 Scan_dir   :   Scan direction
 Colorchose :   RGB or GBR color format
 ********************************************************************************/
void LCD_SetGramScanWay(LCD_SCAN_DIR Scan_dir) {
	uint16_t MemoryAccessReg_Data = 0; //addr:0x36
	uint16_t DisFunReg_Data = 0; //addr:0xB6

	// Gets the scan direction of GRAM
	switch (Scan_dir) {
	case L2R_U2D:                       // Rotation=        0
		MemoryAccessReg_Data = 0x08;    // MADCTL:          BGR
		DisFunReg_Data = 0x22;  // DISPFUNCCTL:     ISC='b0010, SM=0, SS=1, GS=0
		break;
	case L2R_D2U:
		MemoryAccessReg_Data = 0x08;    // MADCTL:          BGR
		DisFunReg_Data = 0x62;
		break;
	case R2L_U2D: //0X4
		MemoryAccessReg_Data = 0x08;    // MADCTL:          BGR
		DisFunReg_Data = 0x02;
		break;
	case R2L_D2U: //0XC                 // Rotation=        2
		MemoryAccessReg_Data = 0x08;    // MADCTL:          BGR
		DisFunReg_Data = 0x42;  // DISPFUNCCTL:     ISC='b0010, SM=0, SS=0, GS=1
		break;
	case U2D_L2R: //0X2
		MemoryAccessReg_Data = 0x28;    // MADCTL:          BGR, MV
		DisFunReg_Data = 0x22;
		break;
	case U2D_R2L: //0X6                 // Rotation=        1
		MemoryAccessReg_Data = 0x28;    // MADCTL:          BGR, MV
		DisFunReg_Data = 0x02;  // DISPFUNCCTL:     ISC='b0010, SM=0, SS=0, GS=0
		break;
	case D2U_L2R: //0XA                 // Rotation=        3
		MemoryAccessReg_Data = 0x28;    // MADCTL:          BGR, MV
		DisFunReg_Data = 0x62;  // DISPFUNCCTRL:    ISC='b0010, SM=0, SS=1, GS=1
		break;
	case D2U_R2L: //0XE
		MemoryAccessReg_Data = 0x28;    // MADCTL: BGR | MV
		DisFunReg_Data = 0x42;
		break;
	}

	//Get the screen scan direction
	sLCD_DIS.LCD_Scan_Dir = Scan_dir;

	//Get GRAM and LCD width and height
	if (Scan_dir == L2R_U2D || Scan_dir == L2R_D2U || Scan_dir == R2L_U2D
			|| Scan_dir == R2L_D2U) {
		sLCD_DIS.LCD_Dis_Column = LCD_HEIGHT;
		sLCD_DIS.LCD_Dis_Page = LCD_WIDTH;
	} else {
		sLCD_DIS.LCD_Dis_Column = LCD_WIDTH;
		sLCD_DIS.LCD_Dis_Page = LCD_HEIGHT;
	}

	// Set the read / write scan direction of the frame memory
	LCD_WriteReg(0xB6);
	LCD_WriteData(0X00);
	LCD_WriteData(DisFunReg_Data);

	LCD_WriteReg(0x36);
	LCD_WriteData(MemoryAccessReg_Data);
}

/********************************************************************************
 function:
 initialization
 ********************************************************************************/
void LCD_Init(LCD_SCAN_DIR LCD_ScanDir, uint16_t LCD_BLval) {
	//Hardware reset
	LCD_Reset();

	if (LCD_BLval > 255)
		LCD_BLval = 255;
	LCD_SetBackLight(LCD_BLval);

	//Set the initialization register
	LCD_InitReg();

	//Set the display scan and color transfer modes
	LCD_SetGramScanWay(LCD_ScanDir);
	Driver_Delay_ms(200);

	//sleep out
	LCD_WriteReg(0x11);
	Driver_Delay_ms(120);

	//Turn on the LCD display
	LCD_WriteReg(0x29);
}

/********************************************************************************
 function:	Sets the start position and size of the display area
 parameter:
 Xstart 	:   X direction Start coordinates
 Ystart  :   Y direction Start coordinates
 Xend    :   X direction end coordinates
 Yend    :   Y direction end coordinates
 ********************************************************************************/
/*
void LCD_SetWindow(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend) {
	//set the X coordinates
	LCD_WriteReg(0x2A);
	LCD_WriteData(Xstart >> 8);	//Set the horizontal starting point to the high octet
	LCD_WriteData(Xstart & 0xff);//Set the horizontal starting point to the low octet
	LCD_WriteData((Xend - 1) >> 8);	//Set the horizontal end to the high octet
	LCD_WriteData((Xend - 1) & 0xff);//Set the horizontal end to the low octet

	//set the Y coordinates
	LCD_WriteReg(0x2B);
	LCD_WriteData(Ystart >> 8);
	LCD_WriteData(Ystart & 0xff);
	LCD_WriteData((Yend - 1) >> 8);
	LCD_WriteData((Yend - 1) & 0xff);
	LCD_WriteReg(0x2C);
}
*/

// Xend -1 and Yend-1 will break LVGL!!
void LCD_SetWindow(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend) {
	//set the X coordinates
	LCD_WriteReg(0x2A);
	LCD_WriteData(Xstart >> 8);	//Set the horizontal starting point to the high octet
	LCD_WriteData(Xstart & 0xff);//Set the horizontal starting point to the low octet
	LCD_WriteData(Xend >> 8);	//Set the horizontal end to the high octet
	LCD_WriteData(Xend & 0xff);//Set the horizontal end to the low octet

	//set the Y coordinates
	LCD_WriteReg(0x2B);
	LCD_WriteData(Ystart >> 8);
	LCD_WriteData(Ystart & 0xff);
	LCD_WriteData(Yend >> 8);
	LCD_WriteData(Yend & 0xff);
	LCD_WriteReg(0x2C);
}


/********************************************************************************
 function:	Set the display point (Xpoint, Ypoint)
 parameter:
 xStart :   X direction Start coordinates
 xEnd   :   X direction end coordinates
 ********************************************************************************/
void LCD_SetCursor(POINT Xpoint, POINT Ypoint) {
	LCD_SetWindow(Xpoint, Ypoint, Xpoint, Ypoint);
}

/********************************************************************************
 function:	Set show color
 parameter:
 Color  :   Set show color,16-bit depth
 ********************************************************************************/
//static void LCD_SetColor(LENGTH Dis_Width, LENGTH Dis_Height, COLOR Color ){
void LCD_SetColor(COLOR Color, POINT Xpoint, POINT Ypoint) {
	LCD_Write_AllData(Color, (uint32_t) Xpoint * (uint32_t) Ypoint);
}

/********************************************************************************
 function:	Point (Xpoint, Ypoint) Fill the color
 parameter:
 Xpoint :   The x coordinate of the point
 Ypoint :   The y coordinate of the point
 Color  :   Set the color
 ********************************************************************************/
void LCD_SetPointlColor( POINT Xpoint, POINT Ypoint, COLOR Color) {
	if ((Xpoint <= sLCD_DIS.LCD_Dis_Column)
			&& (Ypoint <= sLCD_DIS.LCD_Dis_Page)) {
		LCD_SetCursor(Xpoint, Ypoint);
		LCD_SetColor(Color, 1, 1);
	}
}

/********************************************************************************
 function:	Fill the area with the color
 parameter:
 Xstart :   Start point x coordinate
 Ystart :   Start point y coordinate
 Xend   :   End point coordinates
 Yend   :   End point coordinates
 Color  :   Set the color
 ********************************************************************************/
void LCD_SetArealColor(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
COLOR Color) {
	if ((Xend > Xstart) && (Yend > Ystart)) {
		LCD_SetWindow(Xstart, Ystart, Xend, Yend);
		LCD_SetColor(Color, Xend - Xstart, Yend - Ystart);
	}
}

void LCD_Clear(COLOR Color) {
	LCD_SetArealColor(0, 0, sLCD_DIS.LCD_Dis_Column, sLCD_DIS.LCD_Dis_Page,
			Color);
}
/******************************************************************************
 function: Coordinate conversion
 ******************************************************************************/
void GUI_Swop(POINT Point1, POINT Point2) {
	POINT Temp;
	Temp = Point1;
	Point1 = Point2;
	Point2 = Temp;
}

/******************************************************************************
 function: Coordinate conversion
 ******************************************************************************/
void GUI_Clear(COLOR Color) {
	LCD_Clear(Color);
}

/******************************************************************************
 function: Draw Point(Xpoint, Ypoint) Fill the color
 parameter:
 Xpoint    :   The x coordinate of the point
 Ypoint    :   The y coordinate of the point
 Color     :   Set color
 Dot_Pixel :   point size
 ******************************************************************************/
void GUI_DrawPoint(POINT Xpoint, POINT Ypoint, COLOR Color, DOT_PIXEL Dot_Pixel,
		DOT_STYLE DOT_STYLE) {
	if (Xpoint > sLCD_DIS.LCD_Dis_Column || Ypoint > sLCD_DIS.LCD_Dis_Page) {
//    DEBUG("GUI_DrawPoint Input exceeds the normal display range\r\n");
		return;
	}

	int16_t XDir_Num, YDir_Num;
	if (DOT_STYLE == DOT_STYLE_DFT) {
		for (XDir_Num = 0; XDir_Num < 2 * Dot_Pixel - 1; XDir_Num++) {
			for (YDir_Num = 0; YDir_Num < 2 * Dot_Pixel - 1; YDir_Num++) {
				//CAL! LCD_SetPointlColor(Xpoint + XDir_Num - Dot_Pixel, Ypoint + YDir_Num - Dot_Pixel, Color);
				LCD_SetPointlColor(Xpoint + XDir_Num - Dot_Pixel + 1,
						Ypoint + YDir_Num - Dot_Pixel + 1, Color);
			}
		}
	} else {
		for (XDir_Num = 0; XDir_Num < Dot_Pixel; XDir_Num++) {
			for (YDir_Num = 0; YDir_Num < Dot_Pixel; YDir_Num++) {
				//CAL! The following appears to have a bug that causes the Xpoint to be
				//CAL! shifted by -1 and the Ypoint to be shifted by -1. The net result
				//CAL! is that attempts to write to X=0 or Y=0 will cause an underflow,
				//CAL! leading to a Column/Page Address out of bounds.
				//CAL! LCD_SetPointlColor(Xpoint + XDir_Num - 1, Ypoint + YDir_Num - 1, Color);
				LCD_SetPointlColor(Xpoint + XDir_Num, Ypoint + YDir_Num, Color);
			}
		}
	}
}

/******************************************************************************
 function: Draw a line of arbitrary slope
 parameter:
 Xstart ：Starting x point coordinates
 Ystart ：Starting x point coordinates
 Xend   ：End point x coordinate
 Yend   ：End point y coordinate
 Color  ：The color of the line segment
 ******************************************************************************/
void GUI_DrawLine(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
COLOR Color, LINE_STYLE Line_Style, DOT_PIXEL Dot_Pixel) {
	if (Xstart > sLCD_DIS.LCD_Dis_Column || Ystart > sLCD_DIS.LCD_Dis_Page
			|| Xend > sLCD_DIS.LCD_Dis_Column || Yend > sLCD_DIS.LCD_Dis_Page) {
//    DEBUG("GUI_DrawLine Input exceeds the normal display range\r\n");
		return;
	}

	if (Xstart > Xend)
		GUI_Swop(Xstart, Xend);
	if (Ystart > Yend)
		GUI_Swop(Ystart, Yend);

	POINT Xpoint = Xstart;
	POINT Ypoint = Ystart;
	int dx = (int) Xend - (int) Xstart >= 0 ? Xend - Xstart : Xstart - Xend;
	int dy = (int) Yend - (int) Ystart <= 0 ? Yend - Ystart : Ystart - Yend;

	// Increment direction, 1 is positive, -1 is counter;
	int XAddway = Xstart < Xend ? 1 : -1;
	int YAddway = Ystart < Yend ? 1 : -1;

	//Cumulative error
	int Esp = dx + dy;
	char Line_Style_Temp = 0;

	for (;;) {
		Line_Style_Temp++;
		//Painted dotted line, 2 point is really virtual
		if (Line_Style == LINE_DOTTED && Line_Style_Temp % 3 == 0) {
			//DEBUG("LINE_DOTTED\r\n");
			GUI_DrawPoint(Xpoint, Ypoint, LCD_BACKGROUND, Dot_Pixel,
			DOT_STYLE_DFT);
			Line_Style_Temp = 0;
		} else {
			GUI_DrawPoint(Xpoint, Ypoint, Color, Dot_Pixel, DOT_STYLE_DFT);
		}
		if (2 * Esp >= dy) {
			if (Xpoint == Xend)
				break;
			Esp += dy;
			Xpoint += XAddway;
		}
		if (2 * Esp <= dx) {
			if (Ypoint == Yend)
				break;
			Esp += dx;
			Ypoint += YAddway;
		}
	}
}

/******************************************************************************
 function: Draw a rectangle
 parameter:
 Xstart ：Rectangular  Starting x point coordinates
 Ystart ：Rectangular  Starting x point coordinates
 Xend   ：Rectangular  End point x coordinate
 Yend   ：Rectangular  End point y coordinate
 Color  ：The color of the Rectangular segment
 Filled : Whether it is filled--- 1 solid 0：empty
 ******************************************************************************/
void GUI_DrawRectangle(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
COLOR Color, DRAW_FILL Filled, DOT_PIXEL Dot_Pixel) {
	if (Xstart > sLCD_DIS.LCD_Dis_Column || Ystart > sLCD_DIS.LCD_Dis_Page
			|| Xend > sLCD_DIS.LCD_Dis_Column || Yend > sLCD_DIS.LCD_Dis_Page) {
//    DEBUG("Input exceeds the normal display range\r\n");
		return;
	}

	if (Xstart > Xend)
		GUI_Swop(Xstart, Xend);
	if (Ystart > Yend)
		GUI_Swop(Ystart, Yend);

	if (Filled) {
#if LOW_Speed_Show
    POINT Ypoint;
        for(Ypoint = Ystart; Ypoint < Yend; Ypoint++) {
            GUI_DrawLine(Xstart, Ypoint, Xend, Ypoint, Color , LINE_SOLID, Dot_Pixel);
        }
  #elif HIGH_Speed_Show
		LCD_SetArealColor(Xstart, Ystart, Xend, Yend, Color);
#endif
	} else {
		GUI_DrawLine(Xstart, Ystart, Xend, Ystart, Color, LINE_SOLID,
				Dot_Pixel);
		GUI_DrawLine(Xstart, Ystart, Xstart, Yend, Color, LINE_SOLID,
				Dot_Pixel);
		GUI_DrawLine(Xend, Yend, Xend, Ystart, Color, LINE_SOLID, Dot_Pixel);
		GUI_DrawLine(Xend, Yend, Xstart, Yend, Color, LINE_SOLID, Dot_Pixel);
	}
}

/******************************************************************************
 function: Use the 8-point method to draw a circle of the
 specified size at the specified position.
 parameter:
 X_Center  : Center X coordinate
 Y_Center  : Center Y coordinate
 Radius    : circle Radius
 Color     : The color of the ：circle segment
 Filled    : Whether it is filled: 1 filling 0：Do not
 ******************************************************************************/
void GUI_DrawCircle(POINT X_Center, POINT Y_Center, LENGTH Radius,
COLOR Color, DRAW_FILL Draw_Fill, DOT_PIXEL Dot_Pixel) {
	if (X_Center > sLCD_DIS.LCD_Dis_Column
			|| Y_Center >= sLCD_DIS.LCD_Dis_Page) {
//    DEBUG("GUI_DrawCircle Input exceeds the normal display range\r\n");
		return;
	}

	//Draw a circle from(0, R) as a starting point
	int16_t XCurrent, YCurrent;
	XCurrent = 0;
	YCurrent = Radius;

	//Cumulative error,judge the next point of the logo
	int16_t Esp = 3 - (Radius << 1);

	int16_t sCountY;
	if (Draw_Fill == DRAW_FULL) {
		while (XCurrent <= YCurrent) { //Realistic circles
			for (sCountY = XCurrent; sCountY <= YCurrent; sCountY++) {
				GUI_DrawPoint(X_Center + XCurrent, Y_Center + sCountY, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT); //1
				GUI_DrawPoint(X_Center - XCurrent, Y_Center + sCountY, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT); //2
				GUI_DrawPoint(X_Center - sCountY, Y_Center + XCurrent, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT); //3
				GUI_DrawPoint(X_Center - sCountY, Y_Center - XCurrent, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT); //4
				GUI_DrawPoint(X_Center - XCurrent, Y_Center - sCountY, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT); //5
				GUI_DrawPoint(X_Center + XCurrent, Y_Center - sCountY, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT); //6
				GUI_DrawPoint(X_Center + sCountY, Y_Center - XCurrent, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT); //7
				GUI_DrawPoint(X_Center + sCountY, Y_Center + XCurrent, Color,
				DOT_PIXEL_DFT, DOT_STYLE_DFT);
			}
			if (Esp < 0)
				Esp += 4 * XCurrent + 6;
			else {
				Esp += 10 + 4 * (XCurrent - YCurrent);
				YCurrent--;
			}
			XCurrent++;
		}
	} else { //Draw a hollow circle
		while (XCurrent <= YCurrent) {
			GUI_DrawPoint(X_Center + XCurrent, Y_Center + YCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //1
			GUI_DrawPoint(X_Center - XCurrent, Y_Center + YCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //2
			GUI_DrawPoint(X_Center - YCurrent, Y_Center + XCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //3
			GUI_DrawPoint(X_Center - YCurrent, Y_Center - XCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //4
			GUI_DrawPoint(X_Center - XCurrent, Y_Center - YCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //5
			GUI_DrawPoint(X_Center + XCurrent, Y_Center - YCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //6
			GUI_DrawPoint(X_Center + YCurrent, Y_Center - XCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //7
			GUI_DrawPoint(X_Center + YCurrent, Y_Center + XCurrent, Color,
					Dot_Pixel, DOT_STYLE_DFT); //0

			if (Esp < 0)
				Esp += 4 * XCurrent + 6;
			else {
				Esp += 10 + 4 * (XCurrent - YCurrent);
				YCurrent--;
			}
			XCurrent++;
		}
	}
}

/******************************************************************************
 function: Show English characters
 parameter:
 Xpoint           : X coordinate
 Ypoint           : Y coordinate
 Acsii_Char       : To display the English characters
 Font             : A structure pointer that displays a character size
 Color_Background : Select the background color of the English character
 Color_Foreground : Select the foreground color of the English character
 ******************************************************************************/
/*
void GUI_DisChar(POINT Xpoint, POINT Ypoint, const char Acsii_Char, sFONT *Font,
COLOR Color_Background, COLOR Color_Foreground) {
	POINT Page, Column;

	if (Xpoint > sLCD_DIS.LCD_Dis_Column || Ypoint > sLCD_DIS.LCD_Dis_Page) {
//    DEBUG("GUI_DisChar Input exceeds the normal display range\r\n");
		return;
	}

	uint32_t Char_Offset = (Acsii_Char - ' ') * Font->Height
			* (Font->Width / 8 + (Font->Width % 8 ? 1 : 0));
	const unsigned char *ptr = &Font->table[Char_Offset];

	for (Page = 0; Page < Font->Height; Page++) {
		for (Column = 0; Column < Font->Width; Column++) {

			//To determine whether the font background color and screen background color is consistent
			if (FONT_BACKGROUND == Color_Background) { //this process is to speed up the scan
				if (pgm_read_byte(ptr) & (0x80 >> (Column % 8)))
					GUI_DrawPoint(Xpoint + Column, Ypoint + Page,
							Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
			} else {
				if (pgm_read_byte(ptr) & (0x80 >> (Column % 8))) {
					GUI_DrawPoint(Xpoint + Column, Ypoint + Page,
							Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
				} else {
					GUI_DrawPoint(Xpoint + Column, Ypoint + Page,
							Color_Background, DOT_PIXEL_DFT, DOT_STYLE_DFT);
				}
			}
			//One pixel is 8 bits
			if (Column % 8 == 7)
				ptr++;
		}// Write a line
		if (Font->Width % 8 != 0)
			ptr++;
	}// Write all
}
*/
/******************************************************************************
 function: Display the string
 parameter:
 Xstart           : X coordinate
 Ystart           : Y coordinate
 pString          : The first address of the English string to be displayed
 Font             : A structure pointer that displays a character size
 Color_Background : Select the background color of the English character
 Color_Foreground : Select the foreground color of the English character
 ******************************************************************************/
/*
void GUI_DisString_EN(POINT Xstart, POINT Ystart, const char *pString,
		sFONT *Font, COLOR Color_Background, COLOR Color_Foreground) {
	POINT Xpoint = Xstart;
	POINT Ypoint = Ystart;

	if (Xstart > sLCD_DIS.LCD_Dis_Column || Ystart > sLCD_DIS.LCD_Dis_Page) {
//    DEBUG("GUI_DisString_EN Input exceeds the normal display range\r\n");
		return;
	}

	while (*pString != '\0') {
		//if X direction filled , reposition to(Xstart,Ypoint),Ypoint is Y direction plus the height of the character
		if ((Xpoint + Font->Width) > sLCD_DIS.LCD_Dis_Column) {
			Xpoint = Xstart;
			Ypoint += Font->Height;
		}

		// If the Y direction is full, reposition to(Xstart, Ystart)
		if ((Ypoint + Font->Height) > sLCD_DIS.LCD_Dis_Page) {
			Xpoint = Xstart;
			Ypoint = Ystart;
		}
		GUI_DisChar(Xpoint, Ypoint, *pString, Font, Color_Background,
				Color_Foreground);

		//The next character of the address
		pString++;

		//The next word of the abscissa increases the font of the broadband
		Xpoint += Font->Width;
	}
}
*/
/******************************************************************************
 function: Display the string
 parameter:
 Xstart           : X coordinate
 Ystart           : Y coordinate
 Nummber          : The number displayed
 Font             : A structure pointer that displays a character size
 Color_Background : Select the background color of the English character
 Color_Foreground : Select the foreground color of the English character
 ******************************************************************************/
/*
#define  ARRAY_LEN 255
void GUI_DisNum(POINT Xpoint, POINT Ypoint, int32_t Nummber, sFONT *Font,
COLOR Color_Background, COLOR Color_Foreground) {

	int16_t Num_Bit = 0, Str_Bit = 0;
	uint8_t Str_Array[ARRAY_LEN] = { 0 }, Num_Array[ARRAY_LEN] = { 0 };
	uint8_t *pStr = Str_Array;

	if (Xpoint > sLCD_DIS.LCD_Dis_Column || Ypoint > sLCD_DIS.LCD_Dis_Page) {
//    DEBUG("GUI_DisNum Input exceeds the normal display range\r\n");
		return;
	}

	//Converts a number to a string
	while (Nummber) {
		Num_Array[Num_Bit] = Nummber % 10 + '0';
		Num_Bit++;
		Nummber /= 10;
	}

	//The string is inverted
	while (Num_Bit > 0) {
		Str_Array[Str_Bit] = Num_Array[Num_Bit - 1];
		Str_Bit++;
		Num_Bit--;
	}

	//show
	GUI_DisString_EN(Xpoint, Ypoint, (const char*) pStr, Font, Color_Background,
			Color_Foreground);
}
*/
/******************************************************************************
 function: Display the bit map,1 byte = 8bit = 8 points
 parameter:
 Xpoint : X coordinate
 Ypoint : Y coordinate
 pMap   : Pointing to the picture
 Width  : Bitmap Width
 Height : Bitmap Height
 note:
 This function is suitable for bitmap, because a 16-bit data accounted for 16 points
 ******************************************************************************/
void GUI_Disbitmap(POINT Xpoint, POINT Ypoint, const unsigned char *pMap,
POINT Width, POINT Height) {
	POINT i, j, byteWidth = (Width + 7) / 8;
	for (j = 0; j < Height; j++) {
		for (i = 0; i < Width; i++) {
			if (*(pMap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
				GUI_DrawPoint(Xpoint + i, Ypoint + j, WHITE, DOT_PIXEL_DFT,
				DOT_STYLE_DFT);
			}
		}
	}
}

/******************************************************************************
 function: Display the Gray map,1 byte = 8bit = 2 points
 parameter:
 Xpoint : X coordinate
 Ypoint : Y coordinate
 pMap   : Pointing to the picture
 Width  : Bitmap Width
 Height : Bitmap Height
 note:
 This function is suitable for bitmap, because a 4-bit data accounted for 1 points
 Please use the Image2lcd generated array
 ******************************************************************************/
void GUI_DisGrayMap(POINT Xpoint, POINT Ypoint, const unsigned char *pBmp) {
	//Get the Map header Gray, width, height
	char Gray;
	Gray = *(pBmp + 1);
	POINT Height, Width;
	Width = (*(pBmp + 3) << 8) | (*(pBmp + 2));
	Height = (*(pBmp + 5) << 8) | (*(pBmp + 4));

	POINT i, j;
	if (Gray == 0x04) { //Sixteen gray levels
		pBmp = pBmp + 6;
		for (j = 0; j < Height; j++)
			for (i = 0; i < Width / 2; i++) {
				GUI_DrawPoint(Xpoint + i * 2, Ypoint + j, ~(*pBmp >> 4),
				DOT_PIXEL_DFT, DOT_STYLE_DFT);
				GUI_DrawPoint(Xpoint + i * 2 + 1, Ypoint + j, ~*pBmp,
				DOT_PIXEL_DFT, DOT_STYLE_DFT);
				pBmp++;
			}
	} else {
//    DEBUG("Does not support type\r\n");
		return;
	}
}
/*
sFONT* GUI_GetFontSize(POINT Dx, POINT Dy) {
	sFONT *Font = NULL;
	if (Dx > Font24.Width && Dy > Font24.Height) {
		Font = &Font24;
	} else if ((Dx > Font20.Width && Dx < Font24.Width)
			&& (Dy > Font20.Height && Dy < Font24.Height)) {
		Font = &Font20;
	} else if ((Dx > Font16.Width && Dx < Font20.Width)
			&& (Dy > Font16.Height && Dy < Font20.Height)) {
		Font = &Font16;
	} else if ((Dx > Font12.Width && Dx < Font16.Width)
			&& (Dy > Font12.Height && Dy < Font16.Height)) {
		Font = &Font12;
	} else if ((Dx > Font8.Width && Dx < Font12.Width)
			&& (Dy > Font8.Height && Dy < Font12.Height)) {
		Font = &Font8;
	} else {
//    DEBUG("Please change the display area size, or add a larger font to modify\r\n");
	}
	return Font;
}
*/
/******************************************************************************
 function: According to the display area adaptive display time
 parameter:
 xStart :   X direction Start coordinates
 Ystart :   Y direction Start coordinates
 Xend   :   X direction end coordinates
 Yend   :   Y direction end coordinates
 pTime  :   Pointer to the definition of the structure
 Color  :   Set show color
 note:
 ******************************************************************************/
/*
void GUI_Showtime(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
		DEV_TIME *pTime, COLOR Color) {
	uint8_t value[10] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	sFONT *Font = NULL;

	//According to the display area adaptive font size
	POINT Dx = (Xend - Xstart) / 7; //Determine the spacing between characters
	POINT Dy = Yend - Ystart;      //determine the font size
	Font = GUI_GetFontSize(Dx, Dy);

	if ((pTime->Sec % 10) < 10 && (pTime->Sec % 10) > 0) {
		LCD_SetArealColor(Xstart + Dx * 6, Ystart, Xend, Yend, WHITE); // xx:xx:x0
	} else {
		if ((pTime->Sec / 10) < 6 && (pTime->Sec / 10) > 0) {
			LCD_SetArealColor(Xstart + Dx * 5, Ystart, Xend, Yend, WHITE); // xx:xx:00
		} else {      //sec = 60
			pTime->Min = pTime->Min + 1;
			pTime->Sec = 0;
			if ((pTime->Min % 10) < 10 && (pTime->Min % 10) > 0) {
				LCD_SetArealColor(Xstart + Dx * 3 + Dx / 2, Ystart, Xend, Yend,
				WHITE);      // xx:x0:00
			} else {
				if ((pTime->Min / 10) < 6 && (pTime->Min / 10) > 0) {
					LCD_SetArealColor(Xstart + Dx * 2 + Dx / 2, Ystart, Xend,
							Yend, WHITE);      // xx:00:00
				} else {      //min = 60
					pTime->Hour = pTime->Hour + 1;
					pTime->Min = 0;
					if ((pTime->Hour % 10) < 4 && (pTime->Hour % 10) > 0
							&& pTime->Hour < 24) {      // x0:00:00
						LCD_SetArealColor(Xstart + Dx, Ystart, Xend, Yend,
						WHITE);
					} else {
						pTime->Hour = 0;
						pTime->Min = 0;
						pTime->Sec = 0;
						LCD_SetArealColor(Xstart, Ystart, Xend, Yend, WHITE); // 00:00:00
					}
				}
			}
		}
	}

	//Write data into the cache
	GUI_DisChar(Xstart, Ystart, value[pTime->Hour / 10], Font, FONT_BACKGROUND,
			Color);
	GUI_DisChar(Xstart + Dx, Ystart, value[pTime->Hour % 10], Font,
	FONT_BACKGROUND, Color);
	GUI_DisChar(Xstart + Dx + Dx / 4 + Dx / 2, Ystart, ':', Font,
	FONT_BACKGROUND, Color);
	GUI_DisChar(Xstart + Dx * 2 + Dx / 2, Ystart, value[pTime->Min / 10], Font,
	FONT_BACKGROUND, Color);
	GUI_DisChar(Xstart + Dx * 3 + Dx / 2, Ystart, value[pTime->Min % 10], Font,
	FONT_BACKGROUND, Color);
	GUI_DisChar(Xstart + Dx * 4 + Dx / 2 - Dx / 4, Ystart, ':', Font,
	FONT_BACKGROUND, Color);
	GUI_DisChar(Xstart + Dx * 5, Ystart, value[pTime->Sec / 10], Font,
	FONT_BACKGROUND, Color);
	GUI_DisChar(Xstart + Dx * 6, Ystart, value[pTime->Sec % 10], Font,
	FONT_BACKGROUND, Color);
}
*/
/******************************************************************************
 function: GUI_Show
 note:
 Clear,
 Draw Line,
 Draw Rectangle,
 Draw Rings,
 Draw Olympic Rings,
 Display String,
 Show Pic
 ******************************************************************************/
/*
void GUI_Show(void) {
	GUI_Clear(WHITE);
	if (sLCD_DIS.LCD_Dis_Column > sLCD_DIS.LCD_Dis_Page) { //Horizontal screen display

//    DEBUG("Draw Line\r\n");
		GUI_DrawLine(0, 10, LCD_WIDTH, 10, RED, LINE_SOLID, DOT_PIXEL_2X2);
		GUI_DrawLine(0, 20, LCD_WIDTH, 20, RED, LINE_DOTTED, DOT_PIXEL_DFT);
		GUI_DrawLine(0, 300, LCD_WIDTH, 300, RED, LINE_DOTTED, DOT_PIXEL_DFT);
		GUI_DrawLine(0, 310, LCD_WIDTH, 310, RED, LINE_SOLID, DOT_PIXEL_2X2);

//    DEBUG("Draw Rectangle\r\n");
		GUI_DrawRectangle(10, 30, sLCD_DIS.LCD_Dis_Column - 10,
				sLCD_DIS.LCD_Dis_Page - 30, BLUE, DRAW_EMPTY, DOT_PIXEL_DFT);
		GUI_DrawRectangle(20, 40, sLCD_DIS.LCD_Dis_Column - 20, 60, BLUE,
				DRAW_FULL, DOT_PIXEL_DFT);

//    DEBUG("Draw Olympic Rings\r\n");
		uint16_t Cx1 = 190, Cy1 = 240, Cr = 20;
		uint16_t Cx2 = Cx1 + (2.5 * Cr), Cy2 = Cy1;
		uint16_t Cx3 = Cx1 + (5 * Cr), Cy3 = Cy1;
		uint16_t Cx4 = (Cx1 + Cx2) / 2, Cy4 = Cy1 + Cr;
		uint16_t Cx5 = (Cx2 + Cx3) / 2, Cy5 = Cy1 + Cr;

		GUI_DrawCircle(Cx1, Cy1, Cr, BLUE, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx2, Cy2, Cr, BLACK, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx3, Cy3, Cr, RED, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx4, Cy4, Cr, YELLOW, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx5, Cy5, Cr, GREEN, DRAW_EMPTY, DOT_PIXEL_2X2);

//    DEBUG("Draw Realistic circles\r\n");
		GUI_DrawCircle(50, 250, 30, CYAN, DRAW_FULL, DOT_PIXEL_DFT);
		GUI_DrawCircle(sLCD_DIS.LCD_Dis_Column - 50, 250, 30, CYAN, DRAW_FULL,
		DOT_PIXEL_DFT);

//    DEBUG("Display String\r\n");
		GUI_DisString_EN(80, 80, "TFT STM32 Electronic", &Font24,
		LCD_BACKGROUND, BLUE);
		GUI_DisString_EN(80, 120, "3.5inch TFTLCD", &Font20, RED, BLUE);

//    DEBUG("Display Nummber\r\n");
		GUI_DisNum(80, 150, 1234567890, &Font16, LCD_BACKGROUND, BLUE);

	} else { //Vertical screen display

//    DEBUG("Draw Line\r\n");
		GUI_DrawLine(0, 10, sLCD_DIS.LCD_Dis_Column, 10, RED, LINE_SOLID,
				DOT_PIXEL_2X2);
		GUI_DrawLine(0, 20, sLCD_DIS.LCD_Dis_Column, 20, RED, LINE_DOTTED,
		DOT_PIXEL_DFT);
		GUI_DrawLine(0, sLCD_DIS.LCD_Dis_Page - 20, sLCD_DIS.LCD_Dis_Column,
				sLCD_DIS.LCD_Dis_Page - 20, RED, LINE_DOTTED, DOT_PIXEL_DFT);
		GUI_DrawLine(0, sLCD_DIS.LCD_Dis_Page - 10, sLCD_DIS.LCD_Dis_Column,
				sLCD_DIS.LCD_Dis_Page - 10, RED, LINE_SOLID, DOT_PIXEL_2X2);

//    DEBUG("Draw Rectangle\r\n");
		GUI_DrawRectangle(10, 30, sLCD_DIS.LCD_Dis_Column - 10,
				sLCD_DIS.LCD_Dis_Page - 30, BLUE, DRAW_EMPTY, DOT_PIXEL_DFT);
		GUI_DrawRectangle(20, 40, sLCD_DIS.LCD_Dis_Column - 20, 60, BLUE,
				DRAW_FULL, DOT_PIXEL_DFT);

//    DEBUG("Draw Olympic Rings\r\n");
		uint16_t Cx1 = 120, Cy1 = 300, Cr = 20;
		uint16_t Cx2 = Cx1 + (2.5 * Cr), Cy2 = Cy1;
		uint16_t Cx3 = Cx1 + (5 * Cr), Cy3 = Cy1;
		uint16_t Cx4 = (Cx1 + Cx2) / 2, Cy4 = Cy1 + Cr;
		uint16_t Cx5 = (Cx2 + Cx3) / 2, Cy5 = Cy1 + Cr;

		GUI_DrawCircle(Cx1, Cy1, Cr, BLUE, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx2, Cy2, Cr, BLACK, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx3, Cy3, Cr, RED, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx4, Cy4, Cr, YELLOW, DRAW_EMPTY, DOT_PIXEL_2X2);
		GUI_DrawCircle(Cx5, Cy5, Cr, GREEN, DRAW_EMPTY, DOT_PIXEL_2X2);

//    DEBUG("Draw Realistic circles\r\n");
		GUI_DrawCircle(50, 400, 30, CYAN, DRAW_FULL, DOT_PIXEL_DFT);
		GUI_DrawCircle(sLCD_DIS.LCD_Dis_Column - 50, 400, 30, CYAN, DRAW_FULL,
		DOT_PIXEL_DFT);

//    DEBUG("Display String\r\n");
		GUI_DisString_EN(40, 120, "WaveShare Electronic", &Font24,
		LCD_BACKGROUND, BLUE);
		GUI_DisString_EN(40, 180, "3.5inch TFTLCD", &Font20, RED, BLUE);

//    DEBUG("Display Nummber\r\n");
		GUI_DisNum(40, 210, 1234567890, &Font16, LCD_BACKGROUND, BLUE);

	}
}
*/
