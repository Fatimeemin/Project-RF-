import numpy as np
import cv2
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint

# --- 1. VERİ YÜKLEME ---
(train_images, train_labels), (test_images, test_labels) = tf.keras.datasets.mnist.load_data()

# --- 2. ÖZNİTELİK ÇIKARIMI (HU MOMENTS) ---
print("Hu Momentleri hesaplanıyor...")
train_huMoments = np.empty((len(train_images), 7))
test_huMoments = np.empty((len(test_images), 7))

# Fonksiyon: Görüntüden Hu Moment hesapla (STM32'de bunu C ile taklit edeceğiz)
def get_hu_moments(img):
    moments = cv2.moments(img, binaryImage=True) 
    hu = cv2.HuMoments(moments).reshape(7)
    # Log transformu genelde Hu momentlerinde kullanılır ama senin kodunda yoktu, 
    # senin koduna sadık kalıyoruz (Log yok).
    return hu

for i, img in enumerate(train_images):
    train_huMoments[i] = get_hu_moments(img)

for i, img in enumerate(test_images):
    test_huMoments[i] = get_hu_moments(img)

# --- 3. NORMALİZASYON (BU DEĞERLERİ NOT AL!) ---
mean = np.mean(train_huMoments, axis=0)
std = np.std(train_huMoments, axis=0)
train_huMoments = (train_huMoments - mean) / (std + 1e-8)
test_huMoments = (test_huMoments - mean) / (std + 1e-8)

print("\n\n" + "="*40)
print("STM32 İÇİN GEREKLİ DEĞERLER (Kopyala bunları):")
print("MEAN (Ortalama):")
print(", ".join([f"{x:.8f}f" for x in mean]))
print("\nSTD (Standart Sapma):")
print(", ".join([f"{x:.8f}f" for x in std]))
print("="*40 + "\n")

# --- 4. MODEL MİMARİSİ ---
model = keras.models.Sequential([
    keras.layers.Dense(100, input_shape=[7], activation="relu"),
    keras.layers.Dense(100, activation="relu"),
    keras.layers.Dense(10, activation="softmax")
])

model.compile(loss="sparse_categorical_crossentropy", optimizer="adam", metrics=['accuracy'])

# --- 5. EĞİTİM ---
model.fit(train_huMoments, train_labels, epochs=10, batch_size=32, validation_split=0.2, verbose=1)

# --- 6. KAYDETME (.h5 formatı STM32 için daha iyidir) ---
model.save("mnist_hu_model.h5")
print("Model 'mnist_hu_model.h5' olarak kaydedildi.")

# --- 7. TEST İÇİN BİR RESİM DIŞARI AKTAR ---
# STM32'de test etmek için ilk test resmini C array formatında yazdıralım
sample_img = test_images[0]
print(f"\nTest Edilen Resmin Gerçek Etiketi: {test_labels[0]}")
print("STM32 için örnek resim verisi (C Array):")
print("const uint8_t test_image[28*28] = {")
for i in range(28):
    row_vals = ", ".join([str(x) for x in sample_img[i]])
    print(f"  {row_vals},")
print("};") # ... (Senin eğitim kodların buradaydı) ...

# Modeli önce belleğe tam yüklediğinden emin olalım (veya model değişkenini kullanalım)
# model = keras.models.load_model("mnist_hu_model.h5") # Eğer kod akışı devam ediyorsa gerek yok

print("Model TFLite formatına çevriliyor...")

# Converter oluştur
converter = tf.lite.TFLiteConverter.from_keras_model(model)

# Opsiyonel: Dosya boyutunu küçültmek ve uyumluluğu artırmak için optimizasyon
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# Çeviri işlemini yap
tflite_model = converter.convert()

# Dosyayı kaydet
with open("mnist_hu_model.tflite", "wb") as f:
    f.write(tflite_model)

print("Başarılı! 'mnist_hu_model.tflite' dosyası oluşturuldu.")