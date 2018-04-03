#!/usr/bin/env python

"""

docstring

"""

import numpy as np
from keras import Input
from keras.engine import Model
from keras.layers import Conv2D
from keras.layers import MaxPooling2D
from keras.layers import Flatten
from keras.layers import Dense
from keras import optimizers
from sklearn.metrics import accuracy_score


def build_model(height, width, n_classes):
    """

    :param height:
    :param width:
    :param n_classes:
    :return:
    """
    # ...
    input = Input(shape=[height, width, 1])
    # layer 1
    x = Conv2D(32, kernel_size=3, padding="same", activation="relu")(input)
    # layer 2
    x = Conv2D(32, kernel_size=3, padding="same", activation="relu")(x)
    x = MaxPooling2D(pool_size=2, strides=2, padding="same")(x)
    # layer 3
    x = Conv2D(16, kernel_size=3, padding="same", activation="relu")(x)
    x = MaxPooling2D(pool_size=2, strides=2, padding="same")(x)
    # fully connected
    x = Flatten()(x)
    x = Dense(32, activation="relu")(x)
    x = Dense(16, activation="relu")(x)
    x = Dense(n_classes, activation="softmax")(x)
    model = Model(inputs=[input], outputs=[x])

    return model


def train(model, x_train, y_train, epochs=25, learning_rate=5e-2, batch_size=128):
    """

    :param model:
    :param x_train:
    :param y_train:
    :param epochs:
    :param learning_rate:
    :param batch_size:
    :return:
    """

    model.compile(optimizer=optimizers.sgd(lr=learning_rate), loss="categorical_crossentropy", metrics=["accuracy"])

    # train
    model.fit(x=x_train, y=y_train, batch_size=batch_size, epochs=epochs)


if __name__ == '__main__':

    # Parameters
    height, width = 28, 28
    n_classes = 2
    batch_size = 128

    # Get data
    x_train = 0
    y_train = 0
    x_test = 0
    y_test = 0

    # Get the model
    model = build_model(height=height, width=width, n_classes=n_classes)

    # Train
    train(model, x_train, y_train, batch_size=batch_size)

    # Predict
    y_pred = model.predict(x_test, batch_size=batch_size)

    # Accuracy
    acc = accuracy_score(y_test, y_pred.astype(np.int))
    print(acc)







