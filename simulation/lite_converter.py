# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================
"""
Script for converting a keras model into a C byte array.
"""

# Filter tensorflow warnings
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=Warning)
import tensorflow as tf
tf.get_logger().setLevel('INFO')
tf.autograph.set_verbosity(0)
import logging
tf.get_logger().setLevel(logging.ERROR)

FOLDER_NAME = "test"


def main():
    path2folder = os.path.join("../rl_data/models", FOLDER_NAME)

    converter = tf.lite.TFLiteConverter.from_keras_model_file(path2folder + "/model.h5")
    # converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
    tflite_model = converter.convert()

    if not os.path.exists(path2folder + "/lite"):
        os.makedirs(path2folder + "/lite")

    # Save the model to disk
    open(path2folder + "/lite/model_quantized.tflite", "wb").write(tflite_model)

    # Run in terminal
    # xxd -i model_quantized.tflite > model_quantized.cc
    os.system(
        "xxd -i " + path2folder + "/lite/" + "model_quantized.tflite" + " > " + path2folder + "/lite/" + "model_quantized.cc")


if __name__ == '__main__':
    main()
