/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : EE4065 HW3 - Otsu + Morphology + UART2 transmit (STM32F446RE)
  *
  * Output protocol over UART2 (little endian):
  *   uint16_t w
  *   uint16_t h
  *   uint8_t  T
  *   uint8_t  thr[w*h]
  *   uint8_t  dil[w*h]
  *   uint8_t  ero[w*h]
  *   uint8_t  opn[w*h]
  *   uint8_t  cls[w*h]
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "image.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;   // CubeMX normally declares this in usart.c
                             // If your project already has huart2 in usart.c,
                             // then REMOVE this line and use: extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
static uint8_t thr[IMG_WIDTH * IMG_HEIGHT];
static uint8_t dil[IMG_WIDTH * IMG_HEIGHT];
static uint8_t ero[IMG_WIDTH * IMG_HEIGHT];
static uint8_t opn[IMG_WIDTH * IMG_HEIGHT];
static uint8_t cls[IMG_WIDTH * IMG_HEIGHT];
static uint8_t tmp[IMG_WIDTH * IMG_HEIGHT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */

/* If your project already defines huart2 in usart.c, comment the local huart2 above
   and uncomment this line:
   extern UART_HandleTypeDef huart2;
*/

static void uart_send(const uint8_t *data, uint32_t len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
}

static uint8_t otsu_threshold_u8(const uint8_t *img, int w, int h)
{
  uint32_t hist[256];
  memset(hist, 0, sizeof(hist));

  int N = w * h;
  for (int i = 0; i < N; i++) hist[img[i]]++;

  uint64_t sum_all = 0;
  for (int i = 0; i < 256; i++) sum_all += (uint64_t)i * hist[i];

  uint64_t sum_b = 0;
  uint32_t w_b = 0;
  uint32_t w_f = 0;

  double max_sigma = -1.0;
  uint8_t best_t = 0;

  for (int t = 0; t < 256; t++) {
    w_b += hist[t];
    if (w_b == 0) continue;

    w_f = (uint32_t)N - w_b;
    if (w_f == 0) break;

    sum_b += (uint64_t)t * hist[t];

    double m_b = (double)sum_b / (double)w_b;
    double m_f = (double)(sum_all - sum_b) / (double)w_f;

    double diff = (m_b - m_f);
    double sigma_between = (double)w_b * (double)w_f * diff * diff;

    if (sigma_between > max_sigma) {
      max_sigma = sigma_between;
      best_t = (uint8_t)t;
    }
  }
  return best_t;
}

static void threshold_apply_u8(const uint8_t *in, uint8_t *out, int w, int h, uint8_t T)
{
  int N = w * h;
  for (int i = 0; i < N; i++) out[i] = (in[i] > T) ? 255 : 0;
}

static inline uint8_t is_fg(uint8_t v) { return (v > 0); }

static void dilate3x3_u8(const uint8_t *in, uint8_t *out, int w, int h)
{
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint8_t any = 0;
      for (int dy = -1; dy <= 1; dy++) {
        int yy = y + dy;
        if (yy < 0 || yy >= h) continue;
        for (int dx = -1; dx <= 1; dx++) {
          int xx = x + dx;
          if (xx < 0 || xx >= w) continue;
          if (is_fg(in[yy*w + xx])) { any = 1; break; }
        }
        if (any) break;
      }
      out[y*w + x] = any ? 255 : 0;
    }
  }
}

static void erode3x3_u8(const uint8_t *in, uint8_t *out, int w, int h)
{
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint8_t all = 1;
      for (int dy = -1; dy <= 1; dy++) {
        int yy = y + dy;
        if (yy < 0 || yy >= h) { all = 0; break; }
        for (int dx = -1; dx <= 1; dx++) {
          int xx = x + dx;
          if (xx < 0 || xx >= w) { all = 0; break; }
          if (!is_fg(in[yy*w + xx])) { all = 0; break; }
        }
        if (!all) break;
      }
      out[y*w + x] = all ? 255 : 0;
    }
  }
}

static void opening3x3_u8(const uint8_t *in, uint8_t *tmpbuf, uint8_t *out, int w, int h)
{
  erode3x3_u8(in, tmpbuf, w, h);
  dilate3x3_u8(tmpbuf, out, w, h);
}

static void closing3x3_u8(const uint8_t *in, uint8_t *tmpbuf, uint8_t *out, int w, int h)
{
  dilate3x3_u8(in, tmpbuf, w, h);
  erode3x3_u8(tmpbuf, out, w, h);
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  int w = IMG_WIDTH;
  int h = IMG_HEIGHT;

  uint8_t T = otsu_threshold_u8(image, w, h);
  threshold_apply_u8(image, thr, w, h, T);

  dilate3x3_u8(thr, dil, w, h);
  erode3x3_u8(thr, ero, w, h);
  opening3x3_u8(thr, tmp, opn, w, h);
  closing3x3_u8(thr, tmp, cls, w, h);

  // ---- Send results to PC via UART2 ----
  uint16_t ww = (uint16_t)w;
  uint16_t hh = (uint16_t)h;

  uart_send((uint8_t*)&ww, 2);
  uart_send((uint8_t*)&hh, 2);
  uart_send(&T, 1);

  uart_send(thr, (uint32_t)(w*h));
  uart_send(dil, (uint32_t)(w*h));
  uart_send(ero, (uint32_t)(w*h));
  uart_send(opn, (uint32_t)(w*h));
  uart_send(cls, (uint32_t)(w*h));

  /* USER CODE END 2 */

  while (1)
  {
    // Nothing else required
  }
}

/* Clock config (CubeMX can generate this; keep yours if already present) */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  // Minimal GPIO init (CubeMX will add more if needed)
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
