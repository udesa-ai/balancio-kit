import math
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
# from tensorflow.keras import layers

def get_model():
    SAMPLES = 1000
    np.random.seed(1337)
    x_values = np.random.uniform(low=0, high=2*math.pi, size=SAMPLES)
    # shuffle and add noise
    np.random.shuffle(x_values)
    y_values = np.sin(x_values)
    y_values += 0.1 * np.random.randn(*y_values.shape)

    # split into train, validation, test
    TRAIN_SPLIT =  int(0.6 * SAMPLES)
    TEST_SPLIT = int(0.2 * SAMPLES + TRAIN_SPLIT)
    x_train, x_test, x_validate = np.split(x_values, [TRAIN_SPLIT, TEST_SPLIT])
    y_train, y_test, y_validate = np.split(y_values, [TRAIN_SPLIT, TEST_SPLIT])

    # create a NN with 2 layers of 16 neurons
    model = tf.keras.Sequential()
    model.add(tf.keras.layers.Dense(16, activation='relu', input_shape=(1,)))
    model.add(tf.keras.layers.Dense(16, activation='relu'))
    model.add(tf.keras.layers.Dense(1))
    model.compile(optimizer='rmsprop', loss='mse', metrics=['mae'])
    model.fit(x_train, y_train, epochs=10000, batch_size=16384,
                        validation_data=(x_validate, y_validate))
    return model

model = get_model()
x = 0.01*np.arange(0, 200*math.pi)
y_real = np.sin(x)
y_pred = model.predict(x)
plt.plot(x, y_real)
plt.plot(x, y_pred)
plt.show()

# Convert model for microcontroller compatibility
model.save("/home/agus/Documents/UdeSA/Balancio_V0/Code/Mcu/Src/tf_test/model.h5")
converter = tf.lite.TFLiteConverter.from_keras_model_file("/home/agus/Documents/UdeSA/Balancio_V0/Code/Mcu/Src/tf_test/model.h5")
converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()

# Save the model to disk
open("/home/agus/Documents/UdeSA/Balancio_V0/Code/Mcu/Src/tf_test/sine_model_quantized.tflite", "wb").write(tflite_model)

## RUN IN TERMINAL:
# xxd -i sine_model_quantized.tflite > sine_model_quantized.cc
