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
#include "image.h"   // IMG_WIDTH, IMG_HEIGHT, image[]
#include <math.h>
#include <stdint.h>



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/*
 * Global diziler (boyutu sabit olduğu için burada tanımlıyoruz)
 * Q1 & Q2 histogramları:
 */
uint32_t hist_orig[256];   // orijinal görüntü histogramı
uint32_t hist_eq[256];     // eşitlenmiş görüntü histogramı

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* ---- Fonksiyon Prototipleri ---- */

// Q1: Histogram
void compute_histogram(const uint8_t *img, int width, int height, uint32_t *hist);

// Q2: Histogram equalization
void hist_equalization(const uint8_t *in, uint8_t *out, int width, int height);

// Q3: 2D konvolüsyon (3x3 kernel) + low-pass / high-pass
void conv2d_3x3(const uint8_t *in, int16_t *out,
                int width, int height,
                const int8_t kernel[3][3]);

void lowpass_filter(const uint8_t *in, uint8_t *out, int width, int height);
void highpass_filter(const uint8_t *in, uint8_t *out, int width, int height);

// Q4: Median filtre (3x3)
void median_filter_3x3(const uint8_t *in, uint8_t *out, int width, int height);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========================
 * Q1: Histogram Hesaplama
 * ======================== */
void compute_histogram(const uint8_t *img, int width, int height, uint32_t *hist)
{
    // hist dizisini sıfırla
    for (int i = 0; i < 256; i++)
    {
        hist[i] = 0;
    }

    int N = width * height;
    for (int i = 0; i < N; i++)
    {
        uint8_t val = img[i];
        hist[val]++;
    }
}

/* ==================================
 * Q2: Histogram Eşitleme Fonksiyonu
 * ================================== */
void hist_equalization(const uint8_t *in, uint8_t *out, int width, int height)
{
    uint32_t hist[256] = {0};
    uint32_t cdf[256]  = {0};
    uint8_t  map[256];

    int N = width * height;

    // 1) Histogram
    compute_histogram(in, width, height, hist);

    // 2) CDF (kümülatif histogram)
    uint32_t cumulative = 0;
    for (int k = 0; k < 256; k++)
    {
        cumulative += hist[k];
        cdf[k] = cumulative;
    }

    // 3) Lookup tablosu: (cdf / N) * 255
    for (int k = 0; k < 256; k++)
    {
        map[k] = (uint8_t)((cdf[k] * 255U) / (uint32_t)N);
    }

    // 4) Tüm pikseller için dönüştür
    for (int i = 0; i < N; i++)
    {
        uint8_t oldv = in[i];
        out[i] = map[oldv];
    }
}

/* ======================================
 * Q3: 2D Konvolüsyon (3x3 çekirdek ile)
 * ====================================== */
void conv2d_3x3(const uint8_t *in, int16_t *out,
                int width, int height,
                const int8_t kernel[3][3])
{
    int N = width * height;

    // Çıkışı sıfırla
    for (int i = 0; i < N; i++)
    {
        out[i] = 0;
    }

    // Kenarları basitçe es geçiyoruz (1..h-2, 1..w-2)
    for (int y = 1; y < height - 1; y++)
    {
        for (int x = 1; x < width - 1; x++)
        {
            int32_t sum = 0;

            for (int ky = -1; ky <= 1; ky++)
            {
                for (int kx = -1; kx <= 1; kx++)
                {
                    int iy = y + ky;
                    int ix = x + kx;
                    int idx = iy * width + ix;

                    uint8_t px = in[idx];
                    int8_t  kv = kernel[ky + 1][kx + 1];

                    sum += (int32_t)px * (int32_t)kv;
                }
            }

            int out_idx = y * width + x;
            out[out_idx] = (int16_t)sum;
        }
    }
}

/* ======================================
 * Q3(b): Low-pass (ortalama) filtreleme
 * ====================================== */
void lowpass_filter(const uint8_t *in, uint8_t *out, int width, int height)
{
    // 3x3 ortalama filtresi: her eleman 1
    const int8_t kernel[3][3] = {
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1}
    };

    int N = width * height;
    int16_t tmp[N];

    // Konvolüsyon
    conv2d_3x3(in, tmp, width, height, kernel);

    // Normalize (9'a böl) ve 0..255 aralığına kıs
    for (int i = 0; i < N; i++)
    {
        int32_t v = tmp[i] / 9;   // integer division
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        out[i] = (uint8_t)v;
    }
}

/* ======================================
 * Q3(c): High-pass (Laplacian) filtreleme
 * ====================================== */
void highpass_filter(const uint8_t *in, uint8_t *out, int width, int height)
{
    // Basit Laplacian çekirdek
    const int8_t kernel[3][3] = {
        { 0, -1,  0},
        {-1,  4, -1},
        { 0, -1,  0}
    };

    int N = width * height;
    int16_t tmp[N];

    // Konvolüsyon
    conv2d_3x3(in, tmp, width, height, kernel);

    // Negatif değerleri 0'a, büyükleri 255'e kıs
    for (int i = 0; i < N; i++)
    {
        int32_t v = tmp[i];
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        out[i] = (uint8_t)v;
    }
}

/* ==========================
 * Q4: Median filtre (3x3)
 * ========================== */

// Küçük bir bubble sort fonksiyonu
static void sort_array_uint8(uint8_t *arr, int n)
{
    for (int i = 0; i < n - 1; i++)
    {
        for (int j = 0; j < n - 1 - i; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                uint8_t tmp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
}

void median_filter_3x3(const uint8_t *in, uint8_t *out, int width, int height)
{
    int N = width * height;

    // Önce girişten kopyala (kenarlar için direkt bırakalım)
    for (int i = 0; i < N; i++)
    {
        out[i] = in[i];
    }

    uint8_t window[9];

    // Sadece 1..h-2, 1..w-2 içindeki piksellere median uygula
    for (int y = 1; y < height - 1; y++)
    {
        for (int x = 1; x < width - 1; x++)
        {
            // 3x3 pencereyi window[] içine doldur
            int k = 0;
            for (int ky = -1; ky <= 1; ky++)
            {
                for (int kx = -1; kx <= 1; kx++)
                {
                    int iy = y + ky;
                    int ix = x + kx;
                    int idx = iy * width + ix;
                    window[k++] = in[idx];
                }
            }

            // 9 elemanı sırala ve ortadakini al
            sort_array_uint8(window, 9);
            uint8_t med = window[4];

            int out_idx = y * width + x;
            out[out_idx] = med;
        }
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

  // =============================
  //  EE4065 – HW2 İşlemleri
  // =============================

  int width  = IMG_WIDTH;
  int height = IMG_HEIGHT;
  // Çıktı dizileri (stack üzerinde, 64x64 = 4096 byte her biri)
  uint8_t img_eq[IMG_WIDTH * IMG_HEIGHT];
  uint8_t img_lp[IMG_WIDTH * IMG_HEIGHT];
  uint8_t img_hp[IMG_WIDTH * IMG_HEIGHT];
  uint8_t img_med[IMG_WIDTH * IMG_HEIGHT];


  // Q1: Orijinal görüntünün histogramı
  compute_histogram(image, width, height, hist_orig);

  // Q2: Histogram eşitleme
  hist_equalization(image, img_eq, width, height);
  compute_histogram(img_eq, width, height, hist_eq);

  // Q3: Low-pass & High-pass filtreler
  lowpass_filter(image, img_lp, width, height);
  highpass_filter(image, img_hp, width, height);

  // Q4: Median filtre
  median_filter_3x3(image, img_med, width, height);

  // Sonuçlar:
  //  - hist_orig, hist_eq    → orijinal & eşitlenmiş histogramlar
  //  - img_eq, img_lp, img_hp, img_med → Memory / Expressions'dan izlenebilir

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Sürekli işlem yapmamız gerekmiyor;
    // sonuçlar dizilerde tutuluyor.
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
