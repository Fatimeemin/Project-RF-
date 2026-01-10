 import os
import numpy as np
import cv2
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
from matplotlib import pyplot as plt

# --- 1. VERİ YÜKLEME (DATA LOADING) ---
# Kitaptaki 'mnist' kütüphanesi yerine Keras kullanıyoruz (404 hatasını önlemek için)
print("Veriler yükleniyor...")
(train_images, train_labels), (test_images, test_labels) = tf.keras.datasets.mnist.load_data()

# --- 2. ÖZNİTELİK ÇIKARIMI (FEATURE EXTRACTION - HU MOMENTS) ---
print("Hu Momentleri çıkarılıyor (bu işlem biraz sürebilir)...")

# Boş dizileri oluştur
train_huMoments = np.empty((len(train_images), 7))
test_huMoments = np.empty((len(test_images), 7))

# Eğitim verisi için döngü
for i, img in enumerate(train_images):
    # Kitapta 'True' kullanılmış (binary image varsayımı), biz de uyuyoruz.
    moments = cv2.moments(img, binaryImage=True) 
    train_huMoments[i] = cv2.HuMoments(moments).reshape(7)

# Test verisi için döngü
for i, img in enumerate(test_images):
    moments = cv2.moments(img, binaryImage=True)
    test_huMoments[i] = cv2.HuMoments(moments).reshape(7)

# --- 3. NORMALİZASYON (ÖNEMLİ EKLEME) ---
# Sinir ağlarının (MLP) daha iyi çalışması için veriyi ölçeklendiriyoruz.
mean = np.mean(train_huMoments, axis=0)
std = np.std(train_huMoments, axis=0)

# 0'a bölme hatasını önlemek için küçük bir sayı ekledik
train_huMoments = (train_huMoments - mean) / (std + 1e-8)
test_huMoments = (test_huMoments - mean) / (std + 1e-8)

# --- 4. MODEL MİMARİSİ (MLP) ---
# Kitaptaki mimari: 100 Nöron -> 100 Nöron -> 10 Nöron (Çıktı)
model = keras.models.Sequential([
    keras.layers.Dense(100, input_shape=[7], activation="relu"),
    keras.layers.Dense(100, activation="relu"),
    # Çoklu sınıflandırma olduğu için çıkış katmanı 'softmax' kullanılır
    keras.layers.Dense(10, activation="softmax")
])

# Modeli derleme
# SparseCategoricalCrossentropy: Etiketler integer (0,1,2...) olduğu için kullanılır.
model.compile(loss=keras.losses.SparseCategoricalCrossentropy(), 
              optimizer=keras.optimizers.Adam(learning_rate=1e-4),
              metrics=['accuracy'])

# --- 5. CALLBACKS (GERİ ÇAĞIRMALAR) ---
# En iyi modeli kaydetmek ve eğitim iyileşmezse erken durdurmak için
mc_callback = ModelCheckpoint("mlp_mnist_model.keras", save_best_only=True, monitor='val_loss')
es_callback = EarlyStopping(monitor="val_loss", patience=10, restore_best_weights=True)

# --- 6. EĞİTİM (TRAINING) ---
print("Eğitim başlıyor...")
# Validation split ekledik ki EarlyStopping neye bakacağını bilsin
history = model.fit(train_huMoments, 
                    train_labels, 
                    epochs=200,  # Kitapta 1000 denmiş ama 200 genelde yeterlidir
                    batch_size=32,
                    validation_split=0.2, 
                    verbose=1, 
                    callbacks=[mc_callback, es_callback])

# --- 7. DEĞERLENDİRME & CONFUSION MATRIX ---
print("Tahminler yapılıyor...")
nn_preds = model.predict(test_huMoments)
predicted_classes = np.argmax(nn_preds, axis=1)

categories = np.unique(test_labels) # 0, 1, ..., 9

conf_matrix = confusion_matrix(test_labels, predicted_classes)
cm_display = ConfusionMatrixDisplay(confusion_matrix=conf_matrix, display_labels=categories)

fig, ax = plt.subplots(figsize=(10, 10))
cm_display.plot(ax=ax, cmap=plt.cm.Blues)
cm_display.ax_.set_title("Neural Network Confusion Matrix (MLP)")
plt.show()

print("İşlem tamamlandı. Grafik penceresini kapatarak sonlandırabilirsiniz.")