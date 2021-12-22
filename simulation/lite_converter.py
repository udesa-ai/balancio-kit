import tensorflow as tf
import os

FOLDER_NAME = "A2C"

path2folder = os.path.join('/', *os.path.dirname(os.path.realpath(__file__)).split('/')[:-1], "rl_data/models", FOLDER_NAME)

converter = tf.lite.TFLiteConverter.from_keras_model_file(path2folder + "/model.h5")
# converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()

if not os.path.exists(path2folder + "/lite"):
    os.makedirs(path2folder + "/lite")

# Save the model to disk
open(path2folder + "/lite/model_quantized.tflite", "wb").write(tflite_model)

# Run in terminal
# xxd -i model_quantized.tflite > model_quantized.cc
os.system("xxd -i " + path2folder + "/lite/" + "model_quantized.tflite" + " > " + path2folder + "/lite/" + "model_quantized.cc")
