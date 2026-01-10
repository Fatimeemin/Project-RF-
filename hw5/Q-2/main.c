/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 MNIST Hu Moments - AI Entegrasyonu (Tam Kod)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h> // pow, sqrt gibi matematik islemleri icin sart

/* X-CUBE-AI Kutuphaneleri */
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
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
CRC_HandleTypeDef hcrc;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* --- AI Icin Global Degiskenler --- */

ai_handle network = AI_HANDLE_NULL;

/* Bellek Hizalama (Alignment) cok onemlidir */
ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE] __attribute__((aligned(4)));

/* Giris: 7 adet Hu Momenti (Normalize edilmis) */
float ai_input[AI_NETWORK_IN_1_SIZE] __attribute__((aligned(4)));

/* Cikis: 10 adet rakam olasiligi */
float ai_output[AI_NETWORK_OUT_1_SIZE] __attribute__((aligned(4)));


/* ================================================================= */
/* BURALARI PYTHON CIKTISINA GORE KENDIN DOLDURMALISIN       */
/* ================================================================= */

/* 1. MEAN (Ortalama) Degerleri (Python ciktisindan kopyala) */
const float MEAN_VALS[7] = {
     0.33422612f, 0.04476821f, 0.00818686f, 0.00141505f, 0.00000671f, 0.00015559f, -0.00000646f // <--- PYTHON CIKTISINI BURAYA YAZ
};

/* 2. STD (Standart Sapma) Degerleri (Python ciktisindan kopyala) */
const float STD_VALS[7] = {
    0.08404617f, 0.06376581f, 0.01406146f, 0.00258284f, 0.00008245f, 0.00072079f, 0.00006312f// <--- PYTHON CIKTISINI BURAYA YAZ
};

/* 3. TEST RESMI (Python ciktisindan 28x28 arrayi buraya yapistir) */
const uint8_t test_image[28*28] = {
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 84, 185, 159, 151, 60, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 222, 254, 254, 254, 254, 241, 198, 198, 198, 198, 198, 198, 198, 198, 170, 52, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 67, 114, 72, 114, 163, 227, 254, 225, 254, 254, 254, 250, 229, 254, 254, 140, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 66, 14, 67, 67, 67, 59, 21, 236, 254, 106, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 83, 253, 209, 18, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 233, 255, 83, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 129, 254, 238, 44, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 59, 249, 254, 62, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 133, 254, 187, 5, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 205, 248, 58, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 126, 254, 182, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 75, 251, 240, 57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 19, 221, 254, 166, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 203, 254, 219, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 38, 254, 254, 77, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 224, 254, 115, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 133, 254, 254, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 61, 242, 254, 254, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 121, 254, 254, 219, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 121, 254, 207, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		};

/* ================================================================= */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf fonksiyonunu UART'a yonlendirme */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
    return len;
}

/* --- HU MOMENTS HESAPLAMA FONKSIYONU --- */
/* Bu fonksiyon 28x28 resmi alir ve output dizisine 7 tane Hu Momenti yazar */
void calculate_hu_moments(const uint8_t* img, int width, int height, float* hu_output) {

    // 1. Raw Moments (m00, m10, m01) hesapla
    double m00 = 0, m10 = 0, m01 = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint8_t pixel = img[y * width + x];
            if (pixel > 0) { // BinaryImage mantigi
                double val = (double)pixel;
                m00 += val;
                m10 += x * val;
                m01 += y * val;
            }
        }
    }

    // Eger goruntu tamamen siyahsa bolme hatasi olmasin
    if (m00 == 0) {
        for(int k=0; k<7; k++) hu_output[k] = 0.0f;
        return;
    }

    // Agirlik merkezi
    double cx = m10 / m00;
    double cy = m01 / m00;

    // 2. Central Moments hesapla
    double mu20=0, mu02=0, mu11=0, mu30=0, mu12=0, mu21=0, mu03=0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
             uint8_t pixel = img[y * width + x];
             if (pixel > 0) {
                 double val = (double)pixel;
                 double dx = x - cx;
                 double dy = y - cy;

                 mu20 += val * pow(dx, 2);
                 mu02 += val * pow(dy, 2);
                 mu11 += val * dx * dy;
                 mu30 += val * pow(dx, 3);
                 mu03 += val * pow(dy, 3);
                 mu12 += val * dx * pow(dy, 2);
                 mu21 += val * pow(dx, 2) * dy;
             }
        }
    }

    // 3. Normalized Central Moments (nu)
    double inv_m00 = 1.0 / m00;
    double nu20 = mu20 * pow(inv_m00, 2);
    double nu02 = mu02 * pow(inv_m00, 2);
    double nu11 = mu11 * pow(inv_m00, 2);
    double nu30 = mu30 * pow(inv_m00, 2.5);
    double nu03 = mu03 * pow(inv_m00, 2.5);
    double nu12 = mu12 * pow(inv_m00, 2.5);
    double nu21 = mu21 * pow(inv_m00, 2.5);

    // 4. Hu Moments (7 invariants) formülleri
    double t0 = nu20 + nu02;
    double t1 = nu20 - nu02;
    double t2 = nu30 - 3 * nu12;
    double t3 = 3 * nu21 - nu03;
    double t4 = nu30 + nu12;
    double t5 = nu21 + nu03;

    hu_output[0] = (float)(t0);
    hu_output[1] = (float)(pow(t1, 2) + 4 * pow(nu11, 2));
    hu_output[2] = (float)(pow(t2, 2) + pow(t3, 2));
    hu_output[3] = (float)(pow(t4, 2) + pow(t5, 2));
    hu_output[4] = (float)(t2 * t4 * (pow(t4, 2) - 3 * pow(t5, 2)) + t3 * t5 * (3 * pow(t4, 2) - pow(t5, 2)));
    hu_output[5] = (float)(t1 * (pow(t4, 2) - pow(t5, 2)) + 4 * nu11 * t4 * t5);
    hu_output[6] = (float)(t3 * t4 * (pow(t4, 2) - 3 * pow(t5, 2)) - t2 * t5 * (3 * pow(t4, 2) - pow(t5, 2)));
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* --- 1. FPU (Floating Point Unit) ZORLA ACMA KODU --- */
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* --- 2. CRC ZORLA ACMA --- */
  /* NOT: CubeMX'te CRC'yi aktif etmezseniz asagidaki satirda hata alirsiniz. */
  __HAL_RCC_CRC_CLK_ENABLE();
  HAL_Delay(10);
  __HAL_RCC_CRC_FORCE_RESET();
  __HAL_RCC_CRC_RELEASE_RESET();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\n\r\n********************************\r\n");
  printf("STM32 MNIST HU MOMENTS AI TESTI\r\n");
  printf("********************************\r\n");

  /* AI Başlat */
  ai_error err;
  /* Ağı oluştur */
  err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
      printf("HATA: Model olusturulamadi (Kod: %d)\r\n", err.type);
  }

  ai_network_params params = AI_NETWORK_PARAMS_INIT(
      AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
      AI_NETWORK_DATA_ACTIVATIONS(activations)
  );

  if (!ai_network_init(network, &params)) {
      printf("HATA: Model init edilemedi!\r\n");
  } else {
      printf("AI Modeli Basariyla Yuklendi.\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    printf("\r\n--- YENI TEST DONGUSU ---\r\n");

    /* 1. ADIM: Hu Momentlerini Hesapla */
    printf("Adim 1: Hu Momentleri hesaplaniyor...\r\n");
    float raw_hu[7];
    calculate_hu_moments(test_image, 28, 28, raw_hu);

    printf("   Hesaplanan Ham Hu Degerleri:\r\n");
    for(int i=0; i<7; i++) printf("   [%d]: %f\r\n", i, raw_hu[i]);

    /* 2. ADIM: Normalizasyon (Python'daki mean ve std ile) */
    printf("Adim 2: Normalizasyon yapiliyor...\r\n");
    for(int i=0; i<7; i++) {
        /* Formül: (değer - mean) / (std + 1e-8) */
        ai_input[i] = (raw_hu[i] - MEAN_VALS[i]) / (STD_VALS[i] + 0.00000001f);
    }

    /* 3. ADIM: Yapay Zekayi Calistir */
    printf("Adim 3: AI_Run calistiriliyor...\r\n");

    ai_buffer input_bufs[1];
    ai_buffer output_bufs[1];

    /* Input Yapilandirmasi */
    input_bufs[0].format = AI_BUFFER_FORMAT_FLOAT;
    input_bufs[0].data = AI_HANDLE_PTR(ai_input);
    input_bufs[0].meta_info = NULL;
    /* .n_batches SATIRI SILINDI - HATA VERMESIN DIYE */

    /* Output Yapilandirmasi */
    output_bufs[0].format = AI_BUFFER_FORMAT_FLOAT;
    output_bufs[0].data = AI_HANDLE_PTR(ai_output);
    output_bufs[0].meta_info = NULL;
    /* .n_batches SATIRI SILINDI */

    /* Modeli Calistir */
    ai_i32 n_batch = ai_network_run(network, input_bufs, output_bufs);

    if (n_batch != 1) {
        printf("HATA: ai_network_run basarisiz!\r\n");
        ai_error err = ai_network_get_error(network);
        printf("Hata Kodu: Type=%d, Code=%d\r\n", err.type, err.code);
    } else {
        /* 4. ADIM: Sonucu Yazdir (Argmax) */
        float max_score = -1000.0f; // Cok kucuk bir sayi baslangic icin
        int prediction = -1;

        printf("\r\n   --- TAHMIN SONUCLARI ---\r\n");
        for (int i = 0; i < 10; i++) {
            /* Olasiligi yuzdeye cevirip yazalim */
            printf("   Rakam %d: %% %.2f\r\n", i, ai_output[i] * 100.0f);

            if (ai_output[i] > max_score) {
                max_score = ai_output[i];
                prediction = i;
            }
        }

        printf("\r\n   >>> AI TAHMINI: RAKAM [ %d ] (Guven: %% %.2f) <<<\r\n", prediction, max_score * 100.0f);
    }

    HAL_Delay(3000); // 3 saniye bekle

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
#include <sys/stat.h>

int _close(int file) { return -1; }
int _fstat(int file, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _read(int file, char *ptr, int len) { return 0; }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
