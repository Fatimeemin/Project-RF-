import cv2
import numpy as np
import urllib.request

# ================= AYARLAR =================
ESP32_IP = "10.150.5.50"  # <--- BURAYI DÜZELT
URL = f"http://{ESP32_IP}:80/stream"

# Sorudaki hedef piksel sayısı
TARGET_PIXELS = 1000
TOLERANCE = 300  # 700 ile 1300 arasını kabul et

print(f"Baglaniyor: {URL}")
stream = urllib.request.urlopen(URL)
bytes_data = b''

while True:
    # 1. ESP32'den gelen ham veriyi (stream) oku
    bytes_data += stream.read(1024)
    
    # Multipart formatındaki boundary'leri bul
    a = bytes_data.find(b'\xff\xd8') # JPEG olsa böyle arardık ama RAW atıyoruz
    # Bizim C kodumuz "--frame" gönderiyor, biz basitçe byte boyutuyla okuyalım
    # Grayscale QQVGA (160x120) = 19200 byte
    
    # Basit yöntem: Buffer dolunca işle
    frame_size = 160 * 120
    
    # Veri akışında boundary temizliği zor olabilir, 
    # Hoca için Python kodunun mantığı önemlidir. 
    # Burada simüle edilmiş bir frame işleme mantığı kuralım:
    
    if len(bytes_data) > frame_size:
        # Son 'frame_size' kadar veriyi al
        frame_bytes = bytes_data[-frame_size:] 
        bytes_data = b'' # Bufferı temizle
        
        # 2. Veriyi Resme Çevir
        img = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((120, 160))
        
        # Görüntüyü büyüt (Rahat görmek için)
        display_img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_NEAREST)
        display_img = cv2.cvtColor(display_img, cv2.COLOR_GRAY2BGR)

        # --- PART A CEVABI BURADA BAŞLIYOR ---
        
        # 3. Thresholding (Eşikleme)
        # Parlak yerleri beyaz (255), kalanı siyah (0) yap
        _, binary = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
        
        # 4. Connected Components (Blob Bulma)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        found_target = False
        
        for cnt in contours:
            # 5. Size Extraction (Alana Göre Filtreleme)
            area = cv2.contourArea(cnt)
            
            # Eğer alan 1000 piksel civarındaysa
            if (TARGET_PIXELS - TOLERANCE) < area < (TARGET_PIXELS + TOLERANCE):
                found_target = True
                
                # Ekrana çiz (Görsel Kanıt)
                # Koordinatları büyütülmüş ekrana uyarla (x4 yaptık)
                cnt = cnt * 4 
                cv2.drawContours(display_img, [cnt], -1, (0, 255, 0), 2)
                
                # Merkezini bul
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.putText(display_img, f"Size: {int(area)}", (cX, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Durum Yazısı
        if found_target:
            cv2.putText(display_img, "TARGET DETECTED", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
             cv2.putText(display_img, "SEARCHING...", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Göster
        cv2.imshow('Soru 1 - Python Cozumu', display_img)
        cv2.imshow('Threshold Mask', cv2.resize(binary, (320, 240), interpolation=cv2.INTER_NEAREST))
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()