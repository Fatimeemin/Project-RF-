import numpy as np
import librosa
import tensorflow as tf

# AYARLAR
MODEL_PATH = "mlp_fsdd_model.h5"
TEST_AUDIO_FILE = "recordings/0_jackson_0.wav" # Burayı kendi dosyanızla değiştirin
HEADER_FILE = "test_input_data.h"

# 1. Sesi Yükle ve İşle (Python tarafında MFCC yapıyoruz)
print("Ses dosyası işleniyor...")
y, sr = librosa.load(TEST_AUDIO_FILE, sr=8000)

# MFCC Ayarları (Eğitimdekiyle AYNI olmalı)
mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13, n_fft=2048, hop_length=512)

# Boyut Eşitleme (Padding/Trimming - Max 50 frame)
MAX_LEN = 50
if mfcc.shape[1] < MAX_LEN:
    pad_width = MAX_LEN - mfcc.shape[1]
    mfcc = np.pad(mfcc, pad_width=((0, 0), (0, pad_width)), mode='constant')
else:
    mfcc = mfcc[:, :MAX_LEN]

# Veriyi Düzleştir (Flatten) -> (13, 50) -> (650,)
input_data = mfcc.flatten().astype(np.float32)

# 2. C Header Dosyası Oluştur
print(f"{HEADER_FILE} oluşturuluyor...")
with open(HEADER_FILE, "w") as f:
    f.write("#ifndef TEST_INPUT_DATA_H\n")
    f.write("#define TEST_INPUT_DATA_H\n\n")
    f.write(f"// Kaynak: {TEST_AUDIO_FILE}\n")
    f.write(f"// Boyut: {len(input_data)}\n")
    f.write("const float ai_input_data[] = {\n")
    
    for i, val in enumerate(input_data):
        f.write(f"{val:.6f}f, ")
        if (i+1) % 10 == 0:
            f.write("\n")
            
    f.write("\n};\n")
    f.write("#endif\n")

print("Tamamlandı! Şimdi STM32CubeIDE'ye geçebilirsiniz.")