#include "main.h"

//#include "fonts.h"

#define LCD_CS_0        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
#define LCD_CS_1        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

#define LCD_RST_0       HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
#define LCD_RST_1       HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);

#define LCD_DC_0        HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
#define LCD_DC_1        HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))

#define	COLOR				uint16_t		//The variable type of the color (unsigned short) 
#define	POINT				uint16_t		//The type of coordinate (unsigned short) 
#define	LENGTH				uint16_t		//The type of coordinate (unsigned short) 


#define LCD_X_MAXPIXEL  480UL  //LCD width maximum memory
#define LCD_Y_MAXPIXEL  320UL //LCD height maximum memory
#define LCD_X	 0
#define LCD_Y	 0

#define LCD_WIDTH  (LCD_X_MAXPIXEL - 2 * LCD_X)  //LCD width
#define LCD_HEIGHT  LCD_Y_MAXPIXEL //LCD height


typedef enum {
    L2R_U2D  = 0,	//The display interface is displayed , left to right, up to down
    L2R_D2U  ,
    R2L_U2D  ,
    R2L_D2U  ,

    U2D_L2R  ,
    U2D_R2L  ,
    D2U_L2R  ,
    D2U_R2L  ,
} LCD_SCAN_DIR;
#define SCAN_DIR_DFT  D2U_L2R  //Default scan direction = L2R_U2D


typedef struct {
    LENGTH LCD_Dis_Column;	//COLUMN
    LENGTH LCD_Dis_Page;	//PAGE
    LCD_SCAN_DIR LCD_Scan_Dir;
    POINT LCD_X_Adjust;		//LCD x actual display position calibration
    POINT LCD_Y_Adjust;		//LCD y actual display position calibration
} LCD_DIS;


#define LOW_Speed_Show 0
#define HIGH_Speed_Show 1

/********************************************************************************
function:
			dot pixel
********************************************************************************/
typedef enum {
    DOT_PIXEL_1X1  = 1,		// dot pixel 1 x 1
    DOT_PIXEL_2X2  , 		// dot pixel 2 X 2
    DOT_PIXEL_3X3  ,		// dot pixel 3 X 3
    DOT_PIXEL_4X4  ,		// dot pixel 4 X 4
    DOT_PIXEL_5X5  , 		// dot pixel 5 X 5
    DOT_PIXEL_6X6  , 		// dot pixel 6 X 6
    DOT_PIXEL_7X7  , 		// dot pixel 7 X 7
    DOT_PIXEL_8X8  , 		// dot pixel 8 X 8
} DOT_PIXEL;
#define DOT_PIXEL_DFT  DOT_PIXEL_1X1  //Default dot pilex

/********************************************************************************
function:
			dot Fill style
********************************************************************************/
typedef enum {
    DOT_FILL_AROUND  = 1,		// dot pixel 1 x 1
    DOT_FILL_RIGHTUP  , 		// dot pixel 2 X 2
} DOT_STYLE;
#define DOT_STYLE_DFT  DOT_FILL_AROUND  //Default dot pilex
/********************************************************************************
function:
			solid line and dotted line
********************************************************************************/
typedef enum {
    LINE_SOLID = 0,
    LINE_DOTTED,
} LINE_STYLE;

/********************************************************************************
function:
			DRAW Internal fill
********************************************************************************/
typedef enum {
    DRAW_EMPTY = 0,
    DRAW_FULL,
} DRAW_FILL;

/********************************************************************************
function:
	time
********************************************************************************/
typedef struct {
    uint16_t Year;  //0000
    uint8_t  Month; //1 - 12
    uint8_t  Day;   //1 - 30
    uint8_t  Hour;  //0 - 23
    uint8_t  Min;   //0 - 59
    uint8_t  Sec;   //0 - 59
} DEV_TIME;
extern DEV_TIME sDev_time;

#define LCD_BACKGROUND		WHITE   //Default background color
#define FONT_BACKGROUND		WHITE   //Default font background color
#define FONT_FOREGROUND	    GRED    //Default font foreground color

#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED 		   0XFFE0
#define GBLUE		   0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN 		   0XBC40
#define BRRED 		   0XFC07
#define GRAY  		   0X8430
#define LCD_BACKGROUND		WHITE   //Default background color
#define FONT_BACKGROUND		WHITE   //Default font background color
#define FONT_FOREGROUND	    GRED    //Default font foreground color

void Driver_Delay_ms(unsigned long xms);
void Driver_Delay_us(int xus);
void LCD_Init(LCD_SCAN_DIR LCD_ScanDir, uint16_t LCD_BLval);
void LCD_Reset(void);
void LCD_SetGramScanWay(LCD_SCAN_DIR Scan_dir);

void LCD_WriteReg(uint8_t Reg);
void LCD_WriteData(uint8_t Data);
void LCD_Write_AllData(uint16_t Data, uint32_t DataLen);

void LCD_SetWindow(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend);
void LCD_SetCursor(POINT Xpoint, POINT Ypoint);
void LCD_SetColor(COLOR Color ,POINT Xpoint, POINT Ypoint);
void LCD_SetPointlColor(POINT Xpoint, POINT Ypoint, COLOR Color);
void LCD_SetArealColor(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,COLOR  Color);
void LCD_Clear(COLOR  Color);

void GUI_Clear(COLOR Color);

//Drawing
void GUI_DrawPoint(POINT Xpoint, POINT Ypoint, COLOR Color, DOT_PIXEL Dot_Pixel, DOT_STYLE Dot_FillWay);
void GUI_DrawLine(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color, LINE_STYLE Line_Style, DOT_PIXEL Dot_Pixel);
void GUI_DrawRectangle(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color, DRAW_FILL Filled , DOT_PIXEL Dot_Pixel );
void GUI_DrawCircle(POINT X_Center, POINT Y_Center, LENGTH Radius, COLOR Color, DRAW_FILL Draw_Fill , DOT_PIXEL Dot_Pixel );

//pic
void GUI_Disbitmap(POINT Xpoint, POINT Ypoint, const unsigned char *pMap, POINT Width, POINT Height);
void GUI_DisGrayMap(POINT Xpoint, POINT Ypoint, const unsigned char *pBmp);

//Display string
//void GUI_DisChar(POINT Xstart, POINT Ystart, const char Acsii_Char, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground);
//void GUI_DisString_EN(POINT Xstart, POINT Ystart, const char * pString, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground );
//void GUI_DisNum(POINT Xpoint, POINT Ypoint, int32_t Nummber, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground );
void GUI_Showtime(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, DEV_TIME *pTime, COLOR Color);
//show
void GUI_Show(void);






