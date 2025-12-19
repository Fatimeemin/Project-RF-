import serial, struct
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

PORT = "/dev/cu.usbmodem141103"
BAUD = 115200

def read_exact(ser, n):
    data = ser.read(n)
    if len(data) != n:
        raise RuntimeError(f"Expected {n} bytes, got {len(data)}")
    return data

with serial.Serial(PORT, BAUD, timeout=10) as ser:
    # Sometimes there is old buffered data; clearing helps.
    ser.reset_input_buffer()

    # Read w, h, T
    w = struct.unpack("<H", read_exact(ser, 2))[0]
    h = struct.unpack("<H", read_exact(ser, 2))[0]
    T = read_exact(ser, 1)[0]

    print("Received:", "w =", w, "h =", h, "Otsu T =", T)

    def read_img():
        raw = read_exact(ser, w * h)
        return np.frombuffer(raw, dtype=np.uint8).reshape((h, w))

    thr = read_img()
    dil = read_img()
    ero = read_img()
    opn = read_img()
    cls = read_img()

# Display
titles = ["Thresholded", "Dilation", "Erosion", "Opening", "Closing"]
imgs = [thr, dil, ero, opn, cls]

for t, im in zip(titles, imgs):
    plt.figure()
    plt.title(t)
    plt.imshow(im, cmap="gray")
    plt.axis("off")

plt.show()

# Save images (optional)
Image.fromarray(thr).save("thr.png")
Image.fromarray(dil).save("dil.png")
Image.fromarray(ero).save("ero.png")
Image.fromarray(opn).save("opn.png")
Image.fromarray(cls).save("cls.png")

print("Saved: thr.png, dil.png, ero.png, opn.png, cls.png")
