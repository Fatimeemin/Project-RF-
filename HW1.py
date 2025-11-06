from PIL import Image
import numpy as np

# === 1. Load and convert to grayscale ===
img = Image.open("london.png").convert("L").resize((64, 64))  # you can use (128,128)
data = np.array(img, dtype=np.uint8)

# === 2. Save as C header ===
filename = "image.h"
with open(filename, "w") as f:
    f.write("#ifndef IMAGE_H\n#define IMAGE_H\n\n")
    f.write(f"#define IMG_WIDTH {data.shape[1]}\n")
    f.write(f"#define IMG_HEIGHT {data.shape[0]}\n\n")
    f.write("const uint8_t image[IMG_WIDTH * IMG_HEIGHT] = {\n")

    # Write pixels row by row
    for i, row in enumerate(data):
        row_str = ", ".join(str(v) for v in row)
        if i < data.shape[0] - 1:
            f.write("    " + row_str + ",\n")
        else:
            f.write("    " + row_str + "\n")
    f.write("};\n\n#endif\n")

print("✅ image.h created successfully!")
