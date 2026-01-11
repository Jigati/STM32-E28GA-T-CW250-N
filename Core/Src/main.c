/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - E28GA-T-CW250-N LCD Driver
  * @author         : STM32CubeIDE Project
  ******************************************************************************
  * Hardware Connections for STM32F410RB Nucleo:
  *
  * LCD Pin  →  Nucleo Pin  →  Arduino Pin
  * ----------------------------------------
  * DB0-DB7  →  PC0-PC7      (Data bus)
  * CS       →  PA4          (A2)
  * DC       →  PA5          (D13)
  * WR       →  PA6          (D12)
  * RD       →  PA7          (D11)
  * RESET    →  PB0          (A3)
  *
  * IM0-IM3  →  All to GND (8-bit parallel mode)
  * VDD/VDDI →  3.3V
  * GND      →  GND
  * LEDA     →  3.3V + 47Ω resistor
  * LEDK     →  GND
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// NOTE: Nucleo F410RB's onboard LED (LD2) is on PA5, which we use for LCD DC
// We'll add a separate LED on PB10 for status indication, or use PC13 if available
#define STATUS_LED_PIN GPIO_PIN_10
#define STATUS_LED_GPIO_PORT GPIOB

// LCD dimensions
#define LCD_WIDTH  240
#define LCD_HEIGHT 320

// Color definitions (RGB565)
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_ORANGE  0xFD20
#define COLOR_PURPLE  0x8010
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Pin control macros
#define LCD_CS_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define LCD_CS_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define LCD_DC_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define LCD_DC_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define LCD_WR_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_WR_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define LCD_RD_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define LCD_RD_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define LCD_RESET_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define LCD_RESET_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void LCD_WriteData(uint8_t data);
void LCD_WriteCommand(uint8_t cmd);
void LCD_Reset(void);
void LCD_Init(void);
void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_Clear(uint16_t color);
void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
void LCD_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size);
void Demo_ColorBars(void);
void Demo_MovingBox(void);
void Demo_Text(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Simple 5x7 font for basic text rendering
const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x41, 0x22, 0x14, 0x08, 0x00}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x01, 0x01}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x32}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  /* USER CODE BEGIN 2 */

  // Turn on status LED to indicate startup
  HAL_GPIO_WritePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_SET);

  // Initialize LCD
  HAL_Delay(100);
  LCD_Reset();
  LCD_Init();

  // Clear screen
  LCD_Clear(COLOR_BLACK);
  HAL_Delay(500);

  // Display startup message
  LCD_DrawString(60, 10, "STM32F410RB", COLOR_WHITE, COLOR_BLACK, 2);
  LCD_DrawString(40, 40, "LCD Demo Project", COLOR_CYAN, COLOR_BLACK, 2);
  LCD_DrawString(30, 70, "E28GA-T-CW250-N", COLOR_YELLOW, COLOR_BLACK, 1);
  LCD_DrawString(30, 90, "LED on PB10", COLOR_GREEN, COLOR_BLACK, 1);
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Demo 1: Color bars
    Demo_ColorBars();
    HAL_Delay(2000);
    LCD_Clear(COLOR_BLUE);
    LCD_DrawString(30, 20, "I LOVE YOU ELLA", COLOR_WHITE, COLOR_BLUE, 2);
    HAL_Delay(4000);

    // Demo 2: Text display
    Demo_Text();
    HAL_Delay(2000);

    // Demo 3: Moving box animation
    //Demo_MovingBox();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 (Control pins) */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 (LD3 - Nucleo onboard LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 (Reset pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 (Status LED) */
  GPIO_InitStruct.Pin = STATUS_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0-PC7 (Data bus) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                       |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void LCD_WriteData(uint8_t data) {
    LCD_DC_HIGH();
    LCD_CS_LOW();

    GPIOC->ODR = (GPIOC->ODR & 0xFF00) | data;

    LCD_WR_LOW();
    __NOP(); __NOP();
    LCD_WR_HIGH();

    LCD_CS_HIGH();
}

void LCD_WriteCommand(uint8_t cmd) {
    LCD_DC_LOW();
    LCD_CS_LOW();

    GPIOC->ODR = (GPIOC->ODR & 0xFF00) | cmd;

    LCD_WR_LOW();
    __NOP(); __NOP();
    LCD_WR_HIGH();

    LCD_CS_HIGH();
}

void LCD_Reset(void) {
    LCD_RESET_HIGH();
    HAL_Delay(10);
    LCD_RESET_LOW();
    HAL_Delay(10);
    LCD_RESET_HIGH();
    HAL_Delay(120);
}

void LCD_Init(void) {
    // ILI9341 initialization sequence
    LCD_WriteCommand(0x01); // Software Reset
    HAL_Delay(120);

    LCD_WriteCommand(0x11); // Sleep Out
    HAL_Delay(120);

    LCD_WriteCommand(0x3A); // Pixel Format
    LCD_WriteData(0x55);    // 16-bit color

    LCD_WriteCommand(0x36); // Memory Access Control
    LCD_WriteData(0x08);    // BGR only (no mirroring)

    LCD_WriteCommand(0x2A); // Column Address Set
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0xEF); // 239

    LCD_WriteCommand(0x2B); // Page Address Set
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x01);
    LCD_WriteData(0x3F); // 319

    LCD_WriteCommand(0x29); // Display ON
    HAL_Delay(100);
}

void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    LCD_WriteCommand(0x2A); // Column Address Set
    LCD_WriteData(x1 >> 8);
    LCD_WriteData(x1 & 0xFF);
    LCD_WriteData(x2 >> 8);
    LCD_WriteData(x2 & 0xFF);

    LCD_WriteCommand(0x2B); // Page Address Set
    LCD_WriteData(y1 >> 8);
    LCD_WriteData(y1 & 0xFF);
    LCD_WriteData(y2 >> 8);
    LCD_WriteData(y2 & 0xFF);

    LCD_WriteCommand(0x2C); // Memory Write
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if(x >= LCD_WIDTH || y >= LCD_HEIGHT) return;

    LCD_SetWindow(x, y, x, y);
    LCD_WriteData(color >> 8);
    LCD_WriteData(color & 0xFF);
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if(x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if(x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if(y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    LCD_SetWindow(x, y, x + w - 1, y + h - 1);

    uint32_t pixels = (uint32_t)w * h;
    uint8_t colorHigh = color >> 8;
    uint8_t colorLow = color & 0xFF;

    for(uint32_t i = 0; i < pixels; i++) {
        LCD_WriteData(colorHigh);
        LCD_WriteData(colorLow);
    }
}

void LCD_Clear(uint16_t color) {
    LCD_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    if(c < 32 || c > 90) c = 32; // Space for unknown chars

    for(uint8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[c - 32][i];
        for(uint8_t j = 0; j < 8; j++) {
            if(line & 0x01) {
                if(size == 1) {
                    LCD_DrawPixel(x + i, y + j, color);
                } else {
                    LCD_FillRect(x + i*size, y + j*size, size, size, color);
                }
            } else if(bg != color) {
                if(size == 1) {
                    LCD_DrawPixel(x + i, y + j, bg);
                } else {
                    LCD_FillRect(x + i*size, y + j*size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
}

void LCD_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size) {
    while(*str) {
        LCD_DrawChar(x, y, *str, color, bg, size);
        x += 6 * size;
        str++;
    }
}

void Demo_ColorBars(void) {
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN); // Toggle LED

    LCD_Clear(COLOR_BLACK);

    uint16_t barHeight = LCD_HEIGHT / 7;

    LCD_FillRect(0, 0, LCD_WIDTH, barHeight, COLOR_RED);
    LCD_FillRect(0, barHeight, LCD_WIDTH, barHeight, COLOR_GREEN);
    LCD_FillRect(0, barHeight*2, LCD_WIDTH, barHeight, COLOR_BLUE);
    LCD_FillRect(0, barHeight*3, LCD_WIDTH, barHeight, COLOR_YELLOW);
    LCD_FillRect(0, barHeight*4, LCD_WIDTH, barHeight, COLOR_CYAN);
    LCD_FillRect(0, barHeight*5, LCD_WIDTH, barHeight, COLOR_MAGENTA);
    LCD_FillRect(0, barHeight*6, LCD_WIDTH, LCD_HEIGHT - barHeight*6, COLOR_WHITE);

    LCD_DrawString(50, 10, "COLOR BARS TEST", COLOR_BLACK, COLOR_RED, 2);
}

void Demo_Text(void) {
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN); // Toggle LED

    LCD_Clear(COLOR_BLUE);

    LCD_DrawString(30, 20, "STM32 + LCD Demo", COLOR_WHITE, COLOR_BLUE, 2);
    LCD_DrawString(10, 60, "Text Size 1", COLOR_YELLOW, COLOR_BLUE, 1);
    LCD_DrawString(10, 80, "Text Size 2", COLOR_CYAN, COLOR_BLUE, 2);
    LCD_DrawString(10, 110, "Text Size 3", COLOR_GREEN, COLOR_BLUE, 3);

    LCD_DrawString(10, 160, "8-bit Parallel", COLOR_WHITE, COLOR_BLUE, 1);
    LCD_DrawString(10, 175, "240x320 Display", COLOR_WHITE, COLOR_BLUE, 1);
    LCD_DrawString(10, 190, "RGB565 Color", COLOR_WHITE, COLOR_BLUE, 1);

    LCD_DrawString(20, 250, "Press Reset to", COLOR_ORANGE, COLOR_BLUE, 2);
    LCD_DrawString(20, 275, "restart demo", COLOR_ORANGE, COLOR_BLUE, 2);
}

void Demo_MovingBox(void) {
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN); // Toggle LED

    LCD_Clear(COLOR_BLACK);
    LCD_DrawString(40, 10, "MOVING BOX DEMO", COLOR_WHITE, COLOR_BLACK, 2);

    uint16_t boxSize = 40;
    uint16_t y = 100;

    // Move right
    for(uint16_t x = 0; x < LCD_WIDTH - boxSize; x += 3) {
        LCD_FillRect(x, y, boxSize, boxSize, COLOR_RED);
        HAL_Delay(10);
        LCD_FillRect(x, y, boxSize, boxSize, COLOR_BLACK);

        if(x % 30 == 0) HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN);
    }

    // Move left
    for(uint16_t x = LCD_WIDTH - boxSize; x > 0; x -= 3) {
        LCD_FillRect(x, y, boxSize, boxSize, COLOR_GREEN);
        HAL_Delay(10);
        LCD_FillRect(x, y, boxSize, boxSize, COLOR_BLACK);

        if(x % 30 == 0) HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN);
    }

    // Move down
    for(uint16_t ypos = y; ypos < LCD_HEIGHT - boxSize; ypos += 3) {
        LCD_FillRect(20, ypos, boxSize, boxSize, COLOR_BLUE);
        HAL_Delay(10);
        LCD_FillRect(20, ypos, boxSize, boxSize, COLOR_BLACK);

        if(ypos % 30 == 0) HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN);
    }

    // Move up
    for(uint16_t ypos = LCD_HEIGHT - boxSize; ypos > y; ypos -= 3) {
        LCD_FillRect(20, ypos, boxSize, boxSize, COLOR_YELLOW);
        HAL_Delay(10);
        LCD_FillRect(20, ypos, boxSize, boxSize, COLOR_BLACK);

        if(ypos % 30 == 0) HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_PIN);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
