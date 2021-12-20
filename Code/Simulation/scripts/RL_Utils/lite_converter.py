import tensorflow as tf
import os

FOLDER_NAME = "A2C_best"

path2folder = os.getcwd() + "/Models/my_models/" + FOLDER_NAME
converter = tf.lite.TFLiteConverter.from_keras_model_file(path2folder + "/model.h5")
# converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()

if not os.path.exists(path2folder + "/Lite"):
    os.makedirs(path2folder + "/Lite")

# Save the model to disk
open(path2folder + "/Lite/model_quantized.tflite", "wb").write(tflite_model)

# Run in terminal
# xxd -i model_quantized.tflite > model_quantized.cc
os.system("xxd -i " + path2folder + "/Lite/" + "model_quantized.tflite" + " > " + path2folder + "/Lite/" + "model_quantized.cc")
