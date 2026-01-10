import tensorflow as tf
import numpy as np
import os
import librosa

# 1. AYARLAR
MODEL_PATH = "mlp_fsdd_model.h5"
TFLITE_PATH = "model.tflite"
C_MODEL_HEADER = "model_data.h"
C_AUDIO_HEADER = "test_audio.h"

# Test için kullanılacak ses dosyası (recordings klasöründen rastgele bir dosya seçin)
# Buradaki dosya isminin recordings klasöründe olduğundan emin olun!
TEST_AUDIO_FILE = os.path.join("recordings", "0_jackson_0.wav") 
SAMPLE_RATE = 8000

def convert_to_c_array(bytes_data, array_name, data_type="unsigned char"):
    """Binary veriyi C header dosyası formatına çevirir."""
    c_str = f"const {data_type} {array_name}[] = {{\n"
    for i, val in enumerate(bytes_data):
        c_str += f"0x{val:02x}, "
        if (i + 1) % 12 == 0:
            c_str += "\n"
    c_str += "\n};\n"
    c_str += f"const unsigned int {array_name}_len = {len(bytes_data)};\n"
    return c_str

# --- AŞAMA 1: .h5 -> .tflite ÇEVİRİMİ ---
print("1. Model TFLite formatına çevriliyor...")
try:
    model = tf.keras.models.load_model(MODEL_PATH)
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    # STM32 için optimizasyon (isteğe bağlı ama önerilir)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    tflite_model = converter.convert()

    with open(TFLITE_PATH, "wb") as f:
        f.write(tflite_model)
    print(f" -> {TFLITE_PATH} oluşturuldu.")
except Exception as e:
    print(f"HATA: Model çevrilemedi. {e}")
    exit()

# --- AŞAMA 2: .tflite -> C Array ÇEVİRİMİ ---
print("2. TFLite model C dizisine çevriliyor...")
try:
    with open(TFLITE_PATH, "rb") as f:
        tflite_content = f.read()
    
    c_code = "#ifndef MODEL_DATA_H\n#define MODEL_DATA_H\n\n"
    c_code += convert_to_c_array(tflite_content, "g_model_data")
    c_code += "\n#endif"
    
    with open(C_MODEL_HEADER, "w") as f:
        f.write(c_code)
    print(f" -> {C_MODEL_HEADER} oluşturuldu. (STM32'ye eklenecek)")
except Exception as e:
    print(f"HATA: C dizisi oluşturulamadı. {e}")

# --- AŞAMA 3: .wav -> C Array (STATİK TEST İÇİN) ---
print("3. Test ses dosyası C dizisine çevriliyor...")
try:
    if not os.path.exists(TEST_AUDIO_FILE):
        print(f"UYARI: {TEST_AUDIO_FILE} bulunamadı! Lütfen recordings klasöründe var olan bir dosya seçin.")
    else:
        # Sesi yükle
        y, sr = librosa.load(TEST_AUDIO_FILE, sr=SAMPLE_RATE)
        
        # Sesi 16-bit integer formatına ölçekle (Mikrofon verisi gibi olması için)
        # Librosa float (-1.0 ile 1.0) döndürür, biz bunu int16 (-32768 ile 32767) yaparız.
        y_int16 = (y * 32767).astype(np.int16)
        
        # C kodu oluştur
        c_audio_code = "#ifndef TEST_AUDIO_H\n#define TEST_AUDIO_H\n\n"
        c_audio_code += "#include <stdint.h>\n\n"
        
        c_audio_code += f"// Kaynak Dosya: {TEST_AUDIO_FILE}\n"
        c_audio_code += "const int16_t g_test_audio_data[] = {\n"
        for i, val in enumerate(y_int16):
            c_audio_code += f"{val}, "
            if (i + 1) % 15 == 0:
                c_audio_code += "\n"
        c_audio_code += "\n};\n"
        c_audio_code += f"const unsigned int g_test_audio_len = {len(y_int16)};\n"
        c_audio_code += "\n#endif"

        with open(C_AUDIO_HEADER, "w") as f:
            f.write(c_audio_code)
        print(f" -> {C_AUDIO_HEADER} oluşturuldu. (STM32'ye eklenecek)")

except Exception as e:
    print(f"HATA: Ses dosyası çevrilemedi. {e}")

print("\n--- TÜM İŞLEMLER TAMAMLANDI ---")