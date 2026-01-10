import os
import numpy as np
import tensorflow as tf
import librosa
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelBinarizer
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Activation, Dropout

# 1. AYARLAR
RECORDINGS_DIR = "recordings"  # Ses dosyalarının olduğu klasör
SAMPLE_RATE = 8000             # Kitapta belirtilen örnekleme hızı
NUM_MFCC = 13                  # MFCC katsayı sayısı (Kitapla uyumlu olması için)
N_FFT = 2048                   # Pencere boyutu
HOP_LENGTH = 512               # Kaydırma miktarı
MAX_LEN = 50                   # Sabit giriş boyutu için zaman adımı (padding için)

# 2. VERİ YÜKLEME VE ÖZELLİK ÇIKARMA (Feature Extraction)
def create_dataset(folder_path):
    features = []
    labels = []
    
    print("Veriler yükleniyor ve özellikler çıkarılıyor...")
    
    files = [f for f in os.listdir(folder_path) if f.endswith('.wav')]
    
    for file_name in files:
        file_path = os.path.join(folder_path, file_name)
        
        # Dosya isminden etiketi al (Örn: 0_jackson_0.wav -> label = 0)
        label = int(file_name.split('_')[0])
        
        # Sesi yükle (8kHz olarak)
        y, sr = librosa.load(file_path, sr=SAMPLE_RATE)
        
        # MFCC Özelliklerini Çıkar
        mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=NUM_MFCC, n_fft=N_FFT, hop_length=HOP_LENGTH)
        
        # Boyut eşitleme (Padding/Trimming) - Tüm girişler aynı boyutta olmalı
        if mfcc.shape[1] < MAX_LEN:
            pad_width = MAX_LEN - mfcc.shape[1]
            mfcc = np.pad(mfcc, pad_width=((0, 0), (0, pad_width)), mode='constant')
        else:
            mfcc = mfcc[:, :MAX_LEN]
            
        # Veriyi düzleştir (Flatten) -> (13, 50) matrisini (650,) vektörüne çevir
        features.append(mfcc.flatten())
        labels.append(label)
        
    return np.array(features), np.array(labels)

# Veriyi işle
X, y = create_dataset(RECORDINGS_DIR)

# Etiketleri One-Hot Encoding yap (Örn: 5 -> [0,0,0,0,0,1,0,0,0,0])
label_binarizer = LabelBinarizer()
y = label_binarizer.fit_transform(y)

# Eğitim ve Test verisi olarak ayır (%80 Eğitim, %20 Test)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

print(f"Eğitim Verisi Boyutu: {X_train.shape}")
print(f"Test Verisi Boyutu: {X_test.shape}")

# 3. MODEL OLUŞTURMA (Kitap Listing 11.5'teki Mimari)
# Model: 3 Katmanlı MLP (Fully Connected)
model = Sequential()

# Giriş Katmanı ve 1. Gizli Katman (100 Nöron, ReLU)
model.add(Dense(100, input_shape=(X_train.shape[1],))) 
model.add(Activation('relu'))

# 2. Gizli Katman (100 Nöron, ReLU)
model.add(Dense(100))
model.add(Activation('relu'))

# Çıkış Katmanı (10 Nöron - 0'dan 9'a rakamlar için, Softmax)
model.add(Dense(10))
model.add(Activation('softmax'))

# Modeli Derle
# Kitapta: Optimizer=Adam(1e-3), Loss=CategoricalCrossentropy
model.compile(loss='categorical_crossentropy', 
              optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
              metrics=['accuracy'])

model.summary()

# 4. MODEL EĞİTİMİ
print("Model eğitimi başlıyor...")
history = model.fit(X_train, y_train, epochs=50, batch_size=32, validation_data=(X_test, y_test))

# 5. MODELİ KAYDETME
# STM32'ye aktarmak için bu dosya kullanılacak
model_filename = "mlp_fsdd_model.h5"
model.save(model_filename)
print(f"Model başarıyla '{model_filename}' olarak kaydedildi.")

# Başarı oranını yazdır
loss, accuracy = model.evaluate(X_test, y_test)
print(f"Test Doğruluğu: %{accuracy*100:.2f}")