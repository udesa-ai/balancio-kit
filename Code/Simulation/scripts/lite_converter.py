import tensorflow as tf

folder_name = "test2"

converter = tf.lite.TFLiteConverter.from_keras_model_file("/home/agus/Documents/UdeSA/Balancio_V0/Code/Simulation/scripts/Models/" + folder_name + "/model.h5")
converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()

# Save the model to disk
open("/home/agus/Documents/UdeSA/Balancio_V0/Code/Simulation/scripts/Models/" + folder_name + "/Lite/model_quantized.tflite", "wb").write(tflite_model)

## RUN IN TERMINAL:
# xxd -i model_quantized.tflite > model_quantized.cc