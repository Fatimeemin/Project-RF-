import numpy as np
import tensorflow as tf
from tensorflow import keras
import cv2

# --- 1. VERİ YÜKLEME ---
print("Veriler yükleniyor (MNIST)...")
(train_images, train_labels), (test_images, test_labels) = tf.keras.datasets.mnist.load_data()

# --- 2. VERİYİ KÜÇÜLTME (ESP32 RAM'i İçin Çok Önemli) ---
# 28x28 (784 piksel) ESP32'yi yorar. 
# Görüntüleri 14x14 (196 piksel) boyutuna indiriyoruz.
print("Görüntüler ESP32 uyumlu boyuta (14x14) küçültülüyor...")

def resize_images(images):
    resized = []
    for img in images:
        # 14x14'e küçült
        img_small = cv2.resize(img, (14, 14), interpolation=cv2.INTER_AREA)
        resized.append(img_small)
    return np.array(resized)

train_images_small = resize_images(train_images)
test_images_small = resize_images(test_images)

# Normalize et (0-1 arasına çek)
X_train = train_images_small.reshape(-1, 196).astype('float32') / 255.0
X_test = test_images_small.reshape(-1, 196).astype('float32') / 255.0

# --- 3. MODEL MİMARİSİ (ESP32 Uyumlu Basit MLP) ---
# 196 Giriş -> 20 Gizli Katman -> 10 Çıkış
# Bu yapı ESP32'nin RAM'ine rahatça sığar.
model = keras.models.Sequential([
    keras.layers.Dense(20, input_shape=(196,), activation="relu"),
    keras.layers.Dense(10, activation="softmax")
])

model.compile(optimizer='adam', 
              loss='sparse_categorical_crossentropy', 
              metrics=['accuracy'])

# --- 4. EĞİTİM ---
print("Model eğitiliyor...")
model.fit(X_train, train_labels, epochs=10, batch_size=32, verbose=1)

# --- 5. AĞIRLIKLARI C KODUNA ÇEVİRME (SİHİRLİ KISIM) ---
print("\n" + "="*50)
print("AŞAĞIDAKİ KODU KOPYALA VE ESP32 KODUNUN İÇİNE YAPIŞTIR")
print("="*50 + "\n")

# Ağırlıkları al
weights = model.get_weights()
W1 = weights[0] # Giriş -> Gizli Ağırlıkları
B1 = weights[1] # Giriş -> Gizli Biasları
W2 = weights[2] # Gizli -> Çıkış Ağırlıkları
B2 = weights[3] # Gizli -> Çıkış Biasları

def print_c_array(name, array):
    print(f"const float {name}[{array.shape[0]}][{array.shape[1]}] = {{")
    for row in array:
        print("  {", end="")
        for val in row:
            print(f"{val:.5f}, ", end="")
        print("},")
    print("};\n")

def print_c_vector(name, array):
    print(f"const float {name}[{array.shape[0]}] = {{")
    for val in array:
        print(f"{val:.5f}, ", end="")
    print("};\n")

print("// --- PYTHON TARAFINDAN URETILEN AGIRLIKLAR ---")
print_c_array("W1", W1)
print_c_vector("B1", B1)
print_c_array("W2", W2)
print_c_vector("B2", B2)
print("// -------------------------------------------")