import numpy as np
import cv2
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import tensorflow as tf
from matplotlib import pyplot as plt
import os

# --- 1. VERİ YÜKLEME (DATA LOADING) ---
# 'mnist' kütüphanesi hata verdiği için (404), veriyi TensorFlow'un kendi içinden çekiyoruz.
# Bu yöntem en güvenilir ve hızlı yöntemdir.
print("Veriler Keras üzerinden yükleniyor...")
(raw_train_images, raw_train_labels), (raw_test_images, raw_test_labels) = tf.keras.datasets.mnist.load_data()

# --- 2. ÖZNİTELİK ÇIKARIMI (FEATURE EXTRACTION - HU MOMENTS) ---
print("Hu Momentleri çıkarılıyor (bu işlem biraz sürebilir)...")

# Dizileri hazırlıyoruz
train_huMoments = np.empty((len(raw_train_images), 7))
test_huMoments = np.empty((len(raw_test_images), 7))

# Eğitim verisi için döngü
for i, img in enumerate(raw_train_images):
    # cv2.moments uint8 formatında 2D array bekler, Keras verisi zaten öyledir.
    moments = cv2.moments(img, binaryImage=False)
    train_huMoments[i] = cv2.HuMoments(moments).reshape(7)

# Test verisi için döngü
for i, img in enumerate(raw_test_images):
    moments = cv2.moments(img, binaryImage=False)
    test_huMoments[i] = cv2.HuMoments(moments).reshape(7)

# --- 3. VERİ NORMALİZASYONU (Z-SCORE) ---
features_mean = np.mean(train_huMoments, axis=0)
features_std = np.std(train_huMoments, axis=0)

# 0'a bölme hatasını engellemek için küçük bir epsilon eklenebilir ama genelde gerekmez
train_huMoments = (train_huMoments - features_mean) / (features_std + 1e-8)
test_huMoments = (test_huMoments - features_mean) / (features_std + 1e-8)

# --- 4. ETİKETLERİ HAZIRLAMA (BINARY CLASSIFICATION) ---
# 0 rakamı -> Class 0
# Diğer tüm rakamlar (1-9) -> Class 1
train_labels_binary = np.copy(raw_train_labels)
test_labels_binary = np.copy(raw_test_labels)

train_labels_binary[train_labels_binary != 0] = 1
test_labels_binary[test_labels_binary != 0] = 1

# --- 5. MODEL TANIMLAMA (SINGLE NEURON) ---
model = tf.keras.models.Sequential([
    tf.keras.layers.Dense(1, input_shape=[7], activation='sigmoid')
])

model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3),
              loss=tf.keras.losses.BinaryCrossentropy(),
              metrics=[tf.keras.metrics.BinaryAccuracy()])

# --- 6. EĞİTİM (TRAINING) ---
print("Eğitim başlıyor...")
# class_weight dengesiz veri setleri için önemlidir (0 sayısı az, diğerleri çok)
history = model.fit(train_huMoments,
                    train_labels_binary,
                    batch_size=128,
                    epochs=50,
                    class_weight={0: 8, 1: 1}, 
                    verbose=1)

# --- 7. DEĞERLENDİRME & CONFUSION MATRIX ---
print("Tahminler yapılıyor...")
perceptron_preds = model.predict(test_huMoments)
# Sigmoid çıktısı 0.5'ten büyükse 1, küçükse 0
preds_binary = (perceptron_preds > 0.5).astype("int32")

conf_matrix = confusion_matrix(test_labels_binary, preds_binary)
cm_display = ConfusionMatrixDisplay(confusion_matrix=conf_matrix, display_labels=["0", "Not-0"])

cm_display.plot(cmap=plt.cm.Blues)
cm_display.ax_.set_title("Single Neuron Classifier Confusion Matrix")
plt.show()

# --- 8. MODELİ KAYDETME ---
model.save("mnist_single_neuron.h5")
print("Model 'mnist_single_neuron.h5' olarak kaydedildi.")