#include "esp_camera.h"
#include <WiFi.h> // Wi-Fi kütüphanesi kalsın ama bağlanmasa da çalışacak

// ================== AĞ AYARLARI ==================
const char* ssid = "Redmi Note 9 Pro";
const char* password = "rumeysa1234";

// ================== PİN TANIMLARI (AI THINKER) ==================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ================== 0-9 RAKAM ŞABLONLARI (TEMPLATES) ==================
const uint8_t DIGIT_TEMPLATES[10][64] = {
  {0,1,1,1,1,1,0,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,0,0}, // 0
  {0,0,0,1,1,0,0,0, 0,0,1,1,1,0,0,0, 0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0, 0,1,1,1,1,1,1,0}, // 1
  {0,1,1,1,1,1,0,0, 1,1,0,0,0,1,1,0, 0,0,0,0,0,1,1,0, 0,0,0,0,1,1,0,0, 0,0,1,1,1,0,0,0, 0,1,1,0,0,0,0,0, 1,1,0,0,0,0,0,0, 1,1,1,1,1,1,1,0}, // 2
  {0,1,1,1,1,1,0,0, 0,0,0,0,0,1,1,0, 0,0,0,0,0,1,1,0, 0,0,1,1,1,1,0,0, 0,0,0,0,0,1,1,0, 0,0,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,0,0}, // 3
  {0,0,0,0,1,1,0,0, 0,0,0,1,1,1,0,0, 0,0,1,1,0,1,1,0, 0,1,1,0,0,1,1,0, 1,1,1,1,1,1,1,0, 0,0,0,0,0,1,1,0, 0,0,0,0,0,1,1,0, 0,0,0,0,0,1,1,0}, // 4
  {1,1,1,1,1,1,1,0, 1,1,0,0,0,0,0,0, 1,1,1,1,1,0,0,0, 0,0,0,0,0,1,1,0, 0,0,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,0,0}, // 5
  {0,0,1,1,1,1,0,0, 0,1,1,0,0,0,0,0, 1,1,0,0,0,0,0,0, 1,1,1,1,1,1,0,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,0,0}, // 6
  {1,1,1,1,1,1,1,0, 0,0,0,0,0,1,1,0, 0,0,0,0,1,1,0,0, 0,0,0,1,1,0,0,0, 0,0,1,1,0,0,0,0, 0,0,1,1,0,0,0,0, 0,0,1,1,0,0,0,0, 0,0,1,1,0,0,0,0}, // 7
  {0,1,1,1,1,1,0,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,0,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,0,0}, // 8
  {0,1,1,1,1,1,0,0, 1,1,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,1,0, 0,0,0,0,0,1,1,0, 0,0,0,0,0,1,1,0, 1,1,0,0,0,1,1,0, 0,1,1,1,1,1,0,0}  // 9
};

// ================== GÖRÜNTÜ İŞLEME ==================
void preprocess_image(camera_fb_t * fb, uint8_t *output_matrix) {
  int w_block = fb->width / 8;
  int h_block = fb->height / 8;
  
  // EŞİK DEĞERİ (Threshold): Ortam ışığına göre değiştir!
  // Eğer ekranda her şey '#' çıkıyorsa bu değeri DÜŞÜR (örn: 60)
  // Eğer ekranda her şey '.' çıkıyorsa bu değeri ARTIR (örn: 120)
  int threshold = 80; 

  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      long sum = 0;
      for (int by = 0; by < h_block; by++) {
        for (int bx = 0; bx < w_block; bx++) {
          int idx = ((y * h_block + by) * fb->width) + (x * w_block + bx);
          sum += fb->buf[idx];
        }
      }
      int avg = sum / (w_block * h_block);
      
      // Koyu renk (Kalem) -> 1 (#)
      // Açık renk (Kağıt) -> 0 (.)
      if (avg < threshold) {
        output_matrix[y * 8 + x] = 1; 
      } else {
        output_matrix[y * 8 + x] = 0; 
      }
    }
  }
}

int predict_digit(uint8_t *img) {
  int best_digit = -1;
  int min_difference = 9999; 

  // --- SERİ PORTA RESİM ÇİZME ---
  // Hocaya bunu göstereceksin. Görüntüyü metin olarak çiziyor.
  Serial.println("\n--- KAMERA GORUNTUSU (8x8) ---");
  for(int i=0; i<64; i++) {
    if(i%8==0) Serial.println();
    Serial.print(img[i] ? "# " : ". "); // # Yazı, . Kağıt
  }
  Serial.println("\n------------------------------");

  for (int d = 0; d < 10; d++) {
    int difference = 0;
    for (int i = 0; i < 64; i++) {
      if (img[i] != DIGIT_TEMPLATES[d][i]) {
        difference++;
      }
    }
    if (difference < min_difference) {
      min_difference = difference;
      best_digit = d;
    }
  }
  return best_digit;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA; // 160x120
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera Hatasi: 0x%x", err);
    return;
  }

  // Wi-Fi bağlamaya çalışır ama bağlanamazsa da devam eder!
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 5) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  Serial.println("\nSistem Hazir! Tahmin Basliyor...");
}

void loop() {
  // 1. FOTOĞRAF ÇEK
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Kamera okuma hatasi");
    delay(1000);
    return;
  }

  // 2. İŞLE VE TAHMİN ET (Otomatik çalışır)
  uint8_t input_matrix[64];
  preprocess_image(fb, input_matrix);
  
  int digit = predict_digit(input_matrix);
  
  // 3. SONUCU YAZ
  Serial.printf(">>> TAHMIN EDILEN RAKAM: [ %d ] <<<\n", digit);
  
  // 4. HAFIZAYI TEMİZLE
  esp_camera_fb_return(fb);
  
  delay(1000); // Saniyede 1 tahmin (Hoca okuyabilsin diye)
}