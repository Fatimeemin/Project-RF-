#include "esp_camera.h"
#include <WiFi.h>

// ================== AĞ BİLGİLERİN ==================
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

WiFiServer server(80);

// İşlenmiş görüntüyü saklamak için büyük bir bellek alanı ayıralım
// Maksimum 2.5 kat büyütme ihtimaline karşı buffer (160*2.5 * 120*2.5 ~ 120KB)
// ESP32'nin PSRAM'i varsa orayı kullanacağız, yoksa Heap.
uint8_t *output_buffer = NULL;
size_t output_buffer_size = 0;

// ================== SORU 3 CEVABI: RESIZE FONKSİYONU ==================
// Bu fonksiyon hem Upsampling hem Downsampling yapar.
// scale_factor > 1.0 ise Büyütür (Upsample)
// scale_factor < 1.0 ise Küçültür (Downsample)
// Algoritma: Nearest Neighbor Interpolation
void resize_image(uint8_t *src, int src_w, int src_h, float scale_factor, uint8_t **out_buf, int *out_w, int *out_h) {
  
  // 1. Yeni boyutları hesapla (Tamsayı olmayan scale desteği)
  int new_w = (int)(src_w * scale_factor);
  int new_h = (int)(src_h * scale_factor);
  int new_len = new_w * new_h;

  // 2. Bellek Yönetimi (Eğer buffer yetersizse yeniden ayır)
  if (output_buffer == NULL || output_buffer_size < new_len) {
    if (output_buffer) free(output_buffer);
    output_buffer = (uint8_t *)malloc(new_len);
    output_buffer_size = new_len;
    Serial.printf("[RAM] Yeni Buffer Ayrildi: %d bytes\n", new_len);
  }
  
  // 3. EN YAKIN KOMŞU ALGORİTMASI (NEAREST NEIGHBOR)
  // Her yeni piksel için, eski resimdeki karşılığını buluyoruz.
  for (int y = 0; y < new_h; y++) {
    for (int x = 0; x < new_w; x++) {
      
      // Matematik: Yeni koordinatı scale değerine bölerek eski koordinatı bul
      // Örn: Scale 1.5 ise, yeni resimdeki 15. piksel aslında eski resimdeki 10. pikseldir.
      int orig_x = (int)(x / scale_factor);
      int orig_y = (int)(y / scale_factor);

      // Sınır koruması (Taşmayı önle)
      if (orig_x >= src_w) orig_x = src_w - 1;
      if (orig_y >= src_h) orig_y = src_h - 1;

      // Pikseli kopyala
      int old_index = (orig_y * src_w) + orig_x;
      int new_index = (y * new_w) + x;
      
      output_buffer[new_index] = src[old_index];
    }
  }

  // Çıktı parametrelerini güncelle
  *out_buf = output_buffer;
  *out_w = new_w;
  *out_h = new_h;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  
  // Grayscale kullanıyoruz (İşlemesi en kolayı)
  config.pixel_format = PIXFORMAT_GRAYSCALE; 
  config.frame_size = FRAMESIZE_QQVGA; // 160x120
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera Hatasi: 0x%x", err);
    return;
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Baglandi!");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  
  if (client) {
    String request = client.readStringUntil('\r');
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println();

    int mode = 0; // 0: Normal, 1: Upsample, 2: Downsample

    while (client.connected()) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) break;

      uint8_t *final_img = fb->buf;
      int final_w = fb->width;
      int final_h = fb->height;
      int final_len = fb->len;

      // MODLARI DÖNÜŞTÜR (Hocaya göstermek için)
      // Her 20 karede bir mod değiştir
      static int frame_counter = 0;
      frame_counter++;
      if(frame_counter > 20) {
        mode = (mode + 1) % 3; // 0 -> 1 -> 2 -> 0 ...
        frame_counter = 0;
      }

      if (mode == 1) { 
        // --- UPSAMPLING (1.5x) ---
        float scale = 1.5;
        resize_image(fb->buf, fb->width, fb->height, scale, &final_img, &final_w, &final_h);
        final_len = final_w * final_h;
        Serial.printf("Mode: UPSAMPLE (1.5x) | Boyut: %dx%d\n", final_w, final_h);
      } 
      else if (mode == 2) {
        // --- DOWNSAMPLING (0.6x) ---
        float scale = 0.6;
        resize_image(fb->buf, fb->width, fb->height, scale, &final_img, &final_w, &final_h);
        final_len = final_w * final_h;
        Serial.printf("Mode: DOWNSAMPLE (0.6x) | Boyut: %dx%d\n", final_w, final_h);
      }
      else {
        Serial.printf("Mode: NORMAL (1.0x) | Boyut: %dx%d\n", final_w, final_h);
      }

      // Görüntüyü Gönder
      client.println("--frame");
      client.println("Content-Type: application/octet-stream"); // Ham veri
      client.println("Content-Length: " + String(final_len));
      // Genişlik ve Yüksekliği de header'a ekleyelim ki Python anlasın (Opsiyonel ama iyi olur)
      client.println("X-Width: " + String(final_w));
      client.println("X-Height: " + String(final_h));
      client.println();
      client.write(final_img, final_len);
      client.println();
      
      esp_camera_fb_return(fb);
      delay(100); 
    }
  }
}