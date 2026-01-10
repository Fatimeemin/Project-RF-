/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - KERAS TEST VERSIYONU (SAFE)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* X-CUBE-AI Kutuphaneleri */
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"

/* ---> BU DOSYAYI Core/Inc KLASORUNE KOYMAYI UNUTMA <--- */
#include "test_audio.h"
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
/* --- AI Icin Global Degiskenler (STACK TASMASINI ONLEMEK ICIN BURADA) --- */

ai_handle network = AI_HANDLE_NULL;

/* 1. Aktivasyon Buffer'i (Modelin calisirken kullandigi gecici RAM alani) */
/* 4-Byte hizalama (Alignment) islemci performansi ve hatalari onlemek icin sarttir */
ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE] __attribute__((aligned(4)));

/* 2. Giris Buffer'i (Input) */
/* Dogrudan network.h icindeki boyutu kullaniyoruz */
float ai_input_buffer[AI_NETWORK_IN_1_SIZE] __attribute__((aligned(4)));

/* 3. Cikis Buffer'i (Output) */
/* Modelin kac cikisi varsa (Ornegin 10 sinif) ona gore yer ayiriyoruz */
float ai_output_buffer[AI_NETWORK_OUT_1_SIZE] __attribute__((aligned(4)));

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

/* 1. printf fonksiyonunu UART'a yonlendirme */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
    return len;
}

void AI_Init(void) {
    ai_error err;

    /* Ağı oluştur */
    err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE) {
        printf("HATA: Model olusturulamadi (Kod: %d)\r\n", err.type);
        return;
    }

    /* --- Modelin Bilgilerini Yazdiralim --- */
    ai_network_report report;
    if (ai_network_get_info(network, &report)) {
        printf("\r\n--- MODEL BILGILERI ---\r\n");
        printf("Model Beklenen Giris Boyutu: %ld adet float\r\n", (long)report.n_inputs);
        printf("Input Buffer Kapasitesi: %ld (Array Size)\r\n", (long)AI_NETWORK_IN_1_SIZE);
        printf("Output Buffer Kapasitesi: %ld (Array Size)\r\n", (long)AI_NETWORK_OUT_1_SIZE);
        printf("Activations (RAM) Ihtiyaci: %ld Byte\r\n", (long)report.activations.size);
        printf("-----------------------\r\n\r\n");
    }

    ai_network_params params = AI_NETWORK_PARAMS_INIT(
        AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
        AI_NETWORK_DATA_ACTIVATIONS(activations)
    );

    if (!ai_network_init(network, &params)) {
        printf("HATA: Model baslatilamadi!\r\n");
    } else {
        printf("AI Modeli Basariyla Yuklendi ve Init Edildi.\r\n");
    }
}

/* 3. Yapay Zeka Calistirma Fonksiyonu */
void AI_Run(float *input_data, float *output_data) {
    printf("   [AI_Run] Fonksiyonuna girildi.\r\n");

    ai_i32 batch;
    ai_buffer input_bufs[1];
    ai_buffer output_bufs[1];

    /* Input Yapilandirmasi */
    input_bufs[0].format = AI_BUFFER_FORMAT_FLOAT;
    input_bufs[0].data = AI_HANDLE_PTR(input_data);
    input_bufs[0].meta_info = NULL;

    /* Output Yapilandirmasi */
    output_bufs[0].format = AI_BUFFER_FORMAT_FLOAT;
    output_bufs[0].data = AI_HANDLE_PTR(output_data);
    output_bufs[0].meta_info = NULL;

    printf("   [AI_Run] ai_network_run cagriliyor...\r\n");

    /* Modeli Çalıştır */
    batch = ai_network_run(network, input_bufs, output_bufs);

    if (batch != 1) {+
        printf("HATA: ai_network_run basarisiz! (Kod: %ld)\r\n", (long)batch);
        ai_error err = ai_network_get_error(network);
        printf("Hata Detayi - Type: %d, Code: %d\r\n", err.type, err.code);
    } else {
        printf("   [AI_Run] Basariyla tamamlandi.\r\n");

        /* En yuksek olasiligi bul */
        int best_idx = 0;
        float max_val = output_data[0];

        /* AI_NETWORK_OUT_1_SIZE kadar doner */
        for (int i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
            if(output_data[i] > max_val){
                max_val = output_data[i];
                best_idx = i;
            }
        }

        printf("\r\n   >>> TAHMIN SONUCU: %d  (Guven: %% %.2f) <<<\r\n", best_idx, max_val * 100);
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
  /* --- CRC FIX (ALTIN VURUS) --- */
  /* X-CUBE-AI kutuphanesi CRC'yi aktif gormezse HardFault verebilir.
     Burada manuel olarak acip resetliyoruz. */
  __HAL_RCC_CRC_CLK_ENABLE();
  HAL_Delay(10);
  __HAL_RCC_CRC_FORCE_RESET();
  __HAL_RCC_CRC_RELEASE_RESET();
  /* ----------------------------- */
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
  printf("STM32 AI TEST (KERAS + SAFE MEMORY)\r\n");
  printf("********************************\r\n");

  /* Yapay Zekayı Hazırla */
  AI_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    printf("\r\n--- YENI TEST DONGUSU ---\r\n");
    printf("ADIM 1: Veri donusumu (Int16 -> Float) basliyor...\r\n");

    /* --- GUVENLI VERI DONUSUMU --- */
    /* Test verisini (test_audio.h icinden gelir) input buffer'a kopyaliyoruz */

    // Temizlik: Buffer'i sifirla
    memset(ai_input_buffer, 0, sizeof(ai_input_buffer));

    for (int i = 0; i < g_test_audio_len; i++) {

        /* Debug loglarini azalttik */
        if(i == 0) printf("   -> Donusum basladi.\r\n");

        /* KRITIK NOKTA: Buffer Tasmamasini Engelle */
        if (i < AI_NETWORK_IN_1_SIZE) {
            ai_input_buffer[i] = (float)g_test_audio_data[i] / 32768.0f;
        }
        /* Else: Tasarsa yazma, guvenli modda devam et */
    }
    printf("ADIM 1 BITTI: Veri donusumu tamamlandi.\r\n");

    /* 2. ADIM: YAPAY ZEKA TAHMINI */
    printf(" \r\n");

    /* Global bufferlari gonderiyoruz */
    AI_Run(ai_input_buffer, ai_output_buffer);

    printf(" \r\n");

    HAL_Delay(2000);

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

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
