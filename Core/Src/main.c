/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include "ILI9486.h"
#include "xpt2046.h"
#include "lvgl.h"
#include "ui.h"
#include "actions.h"
#include "PeakingFilter.h"
//#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t format;
    uint16_t channels;
    uint32_t frequency;
    uint32_t bytes_per_sec;
    uint16_t bytes_per_block;
    uint16_t bits_per_sample;
} fmt_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_BUFFER_SIZE LCD_X_MAXPIXEL * 64 * 2

#define I2S_DMA_BUFFER_SAMPLES 256
#define I2S_DMA_BUFFER_SIZE 2 * 2 * I2S_DMA_BUFFER_SAMPLES // 2 full buffers L+R samples
#define NUM_BANDS               3
#define SAMPLE_RATE             44100.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
static uint8_t buf1[LCD_BUFFER_SIZE];
static uint8_t buf2[LCD_BUFFER_SIZE];

static lv_display_t *lcd_disp;
PeakingFilter filters[NUM_BANDS];

volatile float amplifier = 1.0f;
int16_t i2s_dma_buffer[I2S_DMA_BUFFER_SIZE];
volatile uint8_t audio_playing = 0;
volatile uint8_t open_next_file = 1;

uint8_t nfiles = 0;

char filelist[20][50] = {0}; //20 files of 50 chars

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FIL music_file; // File
FILINFO music_file_info, fno;
DIR dir;
FRESULT res;  // result

#define USE_DMA_LCD 1
#if USE_DMA_LCD

static volatile bool lcd_bus_busy = 0;
void my_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
	LV_UNUSED(disp);

	int height = area->y2 - area->y1 + 1;
	int width = area->x2 - area->x1 + 1;
	uint32_t flush_size = height * width * 2;

	//Set the drawing region
	LCD_SetWindow(area->x1, area->y1, area->x2, area->y2);
	LCD_DC_1;
	LCD_CS_0;
	lcd_bus_busy = 1;
//	SCB_CleanInvalidateDCache();
	HAL_SPI_Transmit_DMA(&hspi1, px_map, flush_size);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	//Return CS to high
	LCD_CS_1;
	lcd_bus_busy = 0;
	// Inform the graphics library that you are ready with the flushing
	lv_display_flush_ready(lcd_disp);
//	SCB_CleanInvalidateDCache();
}
#else
//	lv_display_set_flush_cb(lcd_disp, my_flush_cb);
void my_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
	int height = area->y2 - area->y1 + 1;
	int width = area->x2 - area->x1 + 1;
	uint32_t flush_size = height * width * 2;

	//Set the drawing region
	LCD_SetWindow(area->x1, area->y1, area->x2, area->y2);
	LCD_DC_1;
	LCD_CS_0;
	HAL_SPI_Transmit(&hspi1, px_map, flush_size, HAL_MAX_DELAY);
	//Return CS to high
	LCD_CS_1;
	// Inform the graphics library that you are ready with the flushing
	lv_display_flush_ready(disp);
}
#endif

float convert_slider_dB(int8_t slider_val)
{
	return (((float)slider_val - 50.0)/4.166667f);
}

void action_update_bass(lv_event_t * e)
{
	float gain = convert_slider_dB((int8_t)lv_slider_get_value(objects.slider_bass));
//	printf("bass gain: %f\r\n", gain);
	char format[11];
	snprintf(format, 11, "%.2f dB", gain);
	lv_label_set_text(objects.label_gain_bass, format);
	PeakingFilter_SetParameters(&filters[0], 70, 2.3, gain);
}

void action_update_mid(lv_event_t * e)
{
	float gain = convert_slider_dB((int8_t)lv_slider_get_value(objects.slider_mid));
//	printf("mid gain: %f\r\n", gain);
	char format[11];
	snprintf(format, 11, "%.2f dB", gain);
	lv_label_set_text(objects.label_gain_mid, format);
	PeakingFilter_SetParameters(&filters[1], 4300, 3.2, gain);
}


void action_update_treble(lv_event_t * e)
{
	float gain = convert_slider_dB((int8_t)lv_slider_get_value(objects.slider_treble));
//	printf("treble gain: %f\r\n", gain);
	char format[11];
	snprintf(format, 11, "%.2f dB", gain);
	lv_label_set_text(objects.label_gain_treble, format);
	PeakingFilter_SetParameters(&filters[2], 13000, 3, gain);
}

void touch_discard_firstreading()
{
	uint8_t read_discard[2] = { 0 };

	SPI_Prescaler_Touch();
	XPT2046_TouchReadChannel(READ_X, read_discard);
	SPI_Prescaler_LCD();
}

static void lvgl_xpt2046_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
	static int16_t x, y;

	if (XPT2046_TouchPressed()) {
#if USE_DMA_LCD
//		 A little blocking but it works
		while(lcd_bus_busy);
	//	if(lcd_bus_busy) {
	//		data->continue_reading = XPT2046_TouchPressed();
	//		return;
	//	}
#endif
		touch_discard_firstreading();
		XPT2046_TouchGetCoordinates(&x, &y);
		data->point.x = x;
		data->point.y = y;
		data->state = LV_INDEV_STATE_PRESSED;
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
	}
}

void lvgl_xpt2046_touch_init()
{
	lv_indev_t *indev = lv_indev_create();
	// Touch pad is a pointer-like device.
	lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
	lv_indev_set_read_cb(indev, lvgl_xpt2046_read_cb);
}

FRESULT parse_wav_header(FIL *f, fmt_typedef *format) {
    uint8_t buf[512];
    UINT n;

    if (f_read(f, buf, sizeof(buf), &n) != FR_OK || n < 44) {
        printf("Read error or file too small\r\n");
        return FR_INT_ERR;
    }

    if (memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0) {
        printf("Not a RIFF/WAVE file\r\n");
        return FR_INT_ERR;
    }

    size_t offset = 12; // Skip RIFF header

    while (offset + 8 < sizeof(buf)) {
        char chunk_id[5] = {0};
        memcpy(chunk_id, buf + offset, 4);
        uint32_t chunk_size = *(uint32_t*)(buf + offset + 4);

        if (memcmp(chunk_id, "fmt ", 4) == 0) {
            memcpy(format, buf + offset + 8, sizeof(fmt_typedef));
        } else if (memcmp(chunk_id, "data", 4) == 0) {
            // Found data chunk, seek to actual PCM data
            if (f_lseek(f, offset + 8) != FR_OK) {
                printf("Seek failed\n");
                return FR_INT_ERR;
            }
            //printf("Found data chunk at offset %u\r\n", offset + 8);
            return FR_OK;
        }

        offset += 8 + chunk_size;
        if (offset & 1) offset++; // Pad to even boundary if needed
    }

    printf("Data chunk not found\r\n");
    return FR_INT_ERR;
}

/*
void process_buffer(int16_t *target_buffer) {
    int16_t buf[2 * I2S_DMA_BUFFER_SAMPLES] = { 0 };
    unsigned int bytes_read = 0;

    if (f_read(&music_file, &buf, sizeof(buf), &bytes_read) == FR_OK) {
        for (int i = 0; i < 2 * I2S_DMA_BUFFER_SAMPLES; ++i) {
            buf[i] = buf[i] * amplifier;
        }

        memcpy(target_buffer, buf, sizeof(buf));

        if (bytes_read < sizeof(buf)) {
            printf("File done!\r\n");
            HAL_I2S_DMAStop(&hi2s2);
            audio_playing = 0;
            open_next_file = 1;
        }
    }
}
*/

void action_reset_eq(lv_event_t * e)
{
	PeakingFilter_Init(&filters[0], SAMPLE_RATE);
	PeakingFilter_Init(&filters[1], SAMPLE_RATE);
	PeakingFilter_Init(&filters[2], SAMPLE_RATE);
	amplifier = 1.0f;

	lv_label_set_text(objects.label_gain_bass, "0 dB");
	lv_label_set_text(objects.label_gain_mid, "0 dB");
	lv_label_set_text(objects.label_gain_treble, "0 dB");
	lv_label_set_text(objects.label_volume, "100");

	lv_slider_set_value(objects.slider_bass, 50, LV_ANIM_OFF);
	lv_slider_set_value(objects.slider_mid, 50, LV_ANIM_OFF);
	lv_slider_set_value(objects.slider_treble, 50, LV_ANIM_OFF);
	lv_slider_set_value(objects.slider_volume, 100, LV_ANIM_OFF);
}

void process_buffer(int16_t *target_buffer) {
    int16_t buf[2 * I2S_DMA_BUFFER_SAMPLES] = { 0 };
    unsigned int bytes_read = 0;

    if (f_read(&music_file, &buf, sizeof(buf), &bytes_read) == FR_OK) {
        for (int i = 0; i < I2S_DMA_BUFFER_SAMPLES; ++i) {
            // Convert to float32
			float l = (float)buf[2 * i]     / 32768.0f;
			float r = (float)buf[2 * i + 1] / 32768.0f;

			// EQ processing
			for (int b = 0; b < NUM_BANDS; ++b) {
				l = PeakingFilter_Update(&filters[b], l);
				r = PeakingFilter_Update(&filters[b], r);
	        }

			// Amplify and convert back
            l = l * amplifier * 32768.0f;
            r = r * amplifier * 32768.0f;

            // Clamp values
            if (l > 32767.0f) l = 32767.0f;
            if (l < -32768.0f) l = -32768.0f;
            if (r > 32767.0f) r = 32767.0f;
            if (r < -32768.0f) r = -32768.0f;

            //Writeback
            target_buffer[2 * i]     = (int16_t)l;
            target_buffer[2 * i + 1] = (int16_t)r;
        }

        if (bytes_read < sizeof(buf)) {
            printf("File done!\r\n");
            HAL_I2S_DMAStop(&hi2s2);
            audio_playing = 0;
            open_next_file = 1;
        }
    }
}


void play_next()
{
	HAL_I2S_DMAStop(&hi2s2);
	audio_playing = 0;
	open_next_file = 1;
}

void action_update_volume(lv_event_t * e)
{
	int8_t value = (int8_t)lv_slider_get_value(objects.slider_volume);
	amplifier = ((float)value) / 100;
	lv_label_set_text_fmt(objects.label_volume, "%d", value);
}

void action_goto_main_page(lv_event_t * e)
{
	loadScreen(SCREEN_ID_MAIN);
}
void action_goto_settings_page(lv_event_t * e)
{
	loadScreen(SCREEN_ID_SETTINGS_PAGE);
}

void action_next_song(lv_event_t * e)
{
	play_next();
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (audio_playing) {
        process_buffer(&i2s_dma_buffer[2 * I2S_DMA_BUFFER_SAMPLES]);
    }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (audio_playing) {
        process_buffer(&i2s_dma_buffer[0]);
    }
}

bool paused = false;

void play_pause_stream()
{
	if(!paused){
		HAL_I2S_DMAPause(&hi2s2);
		paused = true;
	} else {
		HAL_I2S_DMAResume(&hi2s2);
		paused = false;
	}
}

void action_play_pause_audio(lv_event_t * e)
{
//	printf("play/pause button pressed\r\n");
	play_pause_stream();
}

void start_playing()
{
	// Close if there was an open file; if not, the error is harmless
    f_close(&music_file);

    res = f_findnext(&dir, &music_file_info);

    if (res != FR_OK || music_file_info.fname[0] == '\0') { // If we're out of files start again
        res = f_findfirst(&dir, &music_file_info, "", "*.wav");
    }

    printf("Next file: %s\r\n", music_file_info.fname);

    res = f_open(&music_file, music_file_info.fname, FA_READ);
    if (res != FR_OK) {
        printf("Unable to open %s\r\n", music_file_info.fname);
        return;
    }

    fmt_typedef wav_format;

    if (parse_wav_header(&music_file, &wav_format) != FR_OK) {
        printf("Unable to parse header\r\n");
        open_next_file = 1;
    } else {
//        printf("Wav format: %d\r\n", wav_format.format);
//        printf("Wav channels: %d\r\n", wav_format.channels);
//        printf("Wav frequency: %lu\r\n", wav_format.frequency);
//        printf("Wav bytes per sec: %lu\r\n", wav_format.bytes_per_sec);
//        printf("Wav bytes per block: %d\r\n", wav_format.bytes_per_block);
//        printf("Wav bits per sample: %d\r\n", wav_format.bits_per_sample);

        // Pre-fill both halves of the buffer before starting DMA
        process_buffer(&i2s_dma_buffer[0]); // Fill first half
        process_buffer(&i2s_dma_buffer[2 * I2S_DMA_BUFFER_SAMPLES]); // Fill second half

        open_next_file = 0;
        audio_playing = 1;
        lv_label_set_text(objects.label_song, music_file_info.fname);
        HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) &i2s_dma_buffer, I2S_DMA_BUFFER_SIZE);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  MX_I2S2_Init();
  MX_SDMMC1_SD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	PeakingFilter_Init(&filters[0], SAMPLE_RATE);
	PeakingFilter_Init(&filters[1], SAMPLE_RATE);
	PeakingFilter_Init(&filters[2], SAMPLE_RATE);

	res = f_mount(&SDFatFS, SDPath, 1);
  	if (res != FR_OK){
  		puts("Error in mounting SDCard!\r\n");
  	}
  	else {
  		puts("SDCard mounted successfully!\r\n");
  	}

	LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;
	LCD_Init(Lcd_ScanDir, 1000);

	lv_init();
	lv_tick_set_cb(HAL_GetTick);
	lcd_disp = lv_display_create(LCD_X_MAXPIXEL, LCD_Y_MAXPIXEL);
	lv_display_set_buffers(lcd_disp, buf1, buf2, sizeof(buf1),
			LV_DISPLAY_RENDER_MODE_PARTIAL);
	lv_display_set_flush_cb(lcd_disp, my_flush_cb);
	lvgl_xpt2046_touch_init();

	ui_init();

	lv_table_set_column_count(objects.songs_table, 1);
	lv_table_set_column_width(objects.songs_table, 0, 330);
	lv_table_set_cell_value(objects.songs_table, 0, 0, "Name");

  	f_opendir(&dir, "/");   // Open Root
  	do {
  	    f_readdir(&dir, &fno);
  	    if(fno.fname[0] != 0){
  	    	strcpy(filelist[nfiles], fno.fname);
  	    	nfiles++;
//  	        printf("File found: %s\r\n", fno.fname);
  	        lv_table_set_cell_value(objects.songs_table, nfiles, 0, fno.fname);
  	    }
  	} while(fno.fname[0] != 0);
  	f_closedir(&dir);

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (open_next_file) {
            start_playing();
        }
		uint32_t time_till_next = lv_timer_handler();
		HAL_Delay(time_till_next);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 3;
  /* USER CODE BEGIN SDMMC1_Init 2 */

	if (HAL_SD_Init(&hsd1) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
	{
	  Error_Handler();
	}

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 800-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_BUSY_Pin */
  GPIO_InitStruct.Pin = TP_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TP_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin TP_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CD_Pin */
  GPIO_InitStruct.Pin = SDIO_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
