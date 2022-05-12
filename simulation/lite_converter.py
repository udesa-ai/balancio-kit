# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================
"""
Script for converting a keras model into a C byte array, and storing it in the corresponding folder.
Based on: https://github.com/eloquentarduino/tinymlgen/blob/master/tinymlgen/tinymlgen.pys.
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
import argparse


def main():
    # Instantiate the parser
    parser = argparse.ArgumentParser(description='Script to convert a keras/TF model into a C byte array.')
    parser.add_argument("-m", "--model_folder_name", action='store', default='A2C_p_1', type=str,
                        help="Name of the folder containing the model to convert. [Default: 'A2C_p_1'].")
    args = parser.parse_args()

    # Folder containing model to convert.
    folder_name = args.model_folder_name

    path2model_load = os.path.join("../rl_data/models", folder_name)
    path2model_save = "../mcu/src/main/models"
    path2model_save_all = "../mcu/src/main"
    if not os.path.exists(path2model_save):
        os.makedirs(path2model_save)

    # Convert keras model into TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model_file(path2model_load + "/model.h5")
    # converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
    tflite_model = converter.convert()
    # Save TFLite model to disk
    # open(path2folder + "/lite/model_quantized.tflite", "wb").write(tflite_model)

    # Convert TFLite model into C byte array, and save it to file.
    bytes = hexdump.dump(tflite_model).split(' ')
    c_array = ', '.join(['0x%02x' % int(byte, 16) for byte in bytes])
    c = 'const unsigned char rl_model[] DATA_ALIGN_ATTRIBUTE = {%s};' % (c_array)
    c = c.replace('{', '{\n\t').replace('}', '\n}')
    c = re.sub(r'(0x..?, ){12}', lambda x: '%s\n\t' % x.group(0), c)
    c += '\nconst int rl_model_len = %d;' % (len(bytes))
    open(path2model_save + "/{}.h".format(folder_name), 'w').write(c)

    # Update (or create) the file that contains all models.
    preamble_all = '''#include "config.h"
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
#endif \n
'''

    ending = "\n#endif \n"
    c = preamble_all
    list_models = os.listdir(path2model_save)
    for model in list_models:
        if model[0] != ".":
            with open(path2model_save + "/" + model, 'r') as stream:
                preamble = '''\n#ifdef {} \n'''.format(model[:-2])
                c = c + preamble + stream.read() + ending
                stream.close()

    open(path2model_save_all + "/rl_model.h", 'w').write(c)


if __name__ == '__main__':
    main()
