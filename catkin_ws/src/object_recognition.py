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


def build_model(height_, width_, n_classes_):
    """

    :param height_:
    :param width_:
    :param n_classes_:
    :return:
    """
    # ...
    input_layer = Input(shape=[height_, width_, 1])
    # layer 1
    x = Conv2D(32, kernel_size=3, padding="same", activation="relu")(input_layer)
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
    x = Dense(n_classes_, activation="softmax")(x)
    model_ = Model(inputs=[input_layer], outputs=[x])

    return model_


def train(model_, x_train_, y_train_, epochs_=25, learning_rate_=5e-2, batch_size_=128):
    """

    :param model_:
    :param x_train_:
    :param y_train_:
    :param epochs_:
    :param learning_rate_:
    :param batch_size_:
    :return:
    """

    model_.compile(optimizer=optimizers.sgd(lr=learning_rate_), loss="categorical_crossentropy", metrics=["accuracy"])

    # train
    model_.fit(x=x_train_, y=y_train_, batch_size=batch_size_, epochs=epochs_)


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
    model = build_model(height_=height, width_=width, n_classes_=n_classes)

    # Train
    train(model, x_train, y_train, batch_size_=batch_size)

    # Predict
    y_pred = model.predict(x_test, batch_size=batch_size)

    # Accuracy
    acc = accuracy_score(y_test, y_pred.astype(np.int))
    print(acc)







