# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================
"""
Script for converting a keras model into a C byte array.
Based on: https://github.com/eloquentarduino/tinymlgen/blob/master/tinymlgen/tinymlgen.pys.
Almost identical, but using TF1.
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
import hexdump
import re

# Folder containing model to convert.
FOLDER_NAME = "test"


def main():
    path2folder = os.path.join("../rl_data/models", FOLDER_NAME)
    if not os.path.exists(path2folder + "/lite"):
        os.makedirs(path2folder + "/lite")

    # Convert keras model into TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model_file(path2folder + "/model.h5")
    # converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
    tflite_model = converter.convert()
    # Save TFLite model to disk
    open(path2folder + "/lite/model_quantized.tflite", "wb").write(tflite_model)

    # Convert TFLite model into C byte array.
    bytes = hexdump.dump(tflite_model).split(' ')
    c_array = ', '.join(['0x%02x' % int(byte, 16) for byte in bytes])
    c = 'const unsigned char rl_model[] DATA_ALIGN_ATTRIBUTE = {%s};' % (c_array)
    c = c.replace('{', '{\n\t').replace('}', '\n}')
    c = re.sub(r'(0x..?, ){12}', lambda x: '%s\n\t' % x.group(0), c)
    c += '\nconst int rl_model_len = %d;' % (len(bytes))
    preamble = '''
    // if having troubles with min/max, uncomment the following
    // #undef min    
    // #undef max
    #ifdef __has_attribute
    #define HAVE_ATTRIBUTE(x) __has_attribute(x)
    #else
    #define HAVE_ATTRIBUTE(x) 0
    #endif
    #if HAVE_ATTRIBUTE(aligned) || (defined(__GNUC__) && !defined(__clang__))
    #define DATA_ALIGN_ATTRIBUTE __attribute__((aligned(4)))
    #else
    #define DATA_ALIGN_ATTRIBUTE
    #endif
    '''
    open(path2folder + "/lite/model_quantized.cc", 'w').write(preamble + c)


if __name__ == '__main__':
    main()
