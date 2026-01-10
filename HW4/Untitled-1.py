model = tf.keras.models.load_model("mlp_mnist_model.keras")

def representative_dataset():
    for i in range(100):
        yield [train_huMoments[i:i+1].astype(np.float32)]

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_dataset
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

tflite_model = converter.convert()

open("mnist_hu_mlp_int8.tflite","wb").write(tflite_model)
