import os
os.environ['KERAS_BACKEND'] = 'theano'
from keras import Sequential
from keras import layers
from keras.layers import Dense,Dropout
from keras.optimizers import Adam
from keras import metrics
from keras.callbacks import ModelCheckpoint
import keras.backend as K
import numpy as np
from keras.models import load_model


def get_submean_model():
    model = Sequential()
    model.add(Dense(units=256,kernel_initializer='uniform',activation='relu', input_dim=640))

    def sub_mean(x):
        x -= K.mean(x,axis=1,keepdims=True)
        return x
    model.add(layers.core.Lambda(sub_mean,output_shape=lambda input_shape:input_shape))

    model.add(Dropout(0.2))

    model.add(Dense(units=256, kernel_initializer='uniform', activation='relu'))

    model.add(Dropout(0.2))

    model.add(Dense(units=2,kernel_initializer='uniform',activation='softmax'))

    model.compile(optimizer=Adam(lr=1e-4), loss='categorical_crossentropy', metrics=[metrics.categorical_accuracy])
    return model
model = get_submean_model()
#res=model.predict(np.random.random((3,7)))
#model.save('add_layers.h5')
model.summary()

def compute_input_shape(train_file):
    import h5py
    h5f = h5py.File(train_file, 'r')
    input_shape = h5f['input_shape'][()]
    return input_shape


def compute_step_per_epoch(train_file):
    import h5py
    h5f = h5py.File(train_file, 'r')
    step_per_epoch = h5f['step_per_epoch'][()]
    return step_per_epoch


def read_hdf5_file_produce_train_data(file):
    import h5py
    h5f = h5py.File(file, 'r')
    step_per_epoch = h5f['step_per_epoch'][()]
    i = 1
    while True:
        val_x = h5f['item_x_' + str(i)][:]
        val_y = h5f['item_y_' + str(i)][:]
        yield (val_x, val_y)
        i = i + 1
        if i > step_per_epoch:
            i = 1


step_per_epoch = compute_step_per_epoch('/home/yangtb/tmp/xiaodufeat/data_train.h5')
validate_step_per_epoch = compute_step_per_epoch('/home/yangtb/tmp/xiaodufeat/data_validate.h5')
print(str(step_per_epoch))
print(str(validate_step_per_epoch))

callback_list = []
checkpoint = ModelCheckpoint('model_best.cpickle',
                             monitor='val_loss',
                             save_weights_only=True,
                             verbose=1,
                             save_best_only=True,
                             period=1
                            )
callback_list.append(checkpoint)

hist = model.fit_generator(
                    read_hdf5_file_produce_train_data('/home/yangtb/tmp/xiaodufeat/data_train.h5'),
                    steps_per_epoch=step_per_epoch,
                    epochs=20,
                    validation_data=read_hdf5_file_produce_train_data('/home/yangtb/tmp/xiaodufeat/data_validate.h5'),
                    validation_steps=validate_step_per_epoch,
                    verbose=1,
                    shuffle=True,
                    callbacks=callback_list
                )
model.save('add_layers.h5')
res1 = 0

# def get_submean_model():
#     model = Sequential()
#     model.add(Dense(5,input_dim=7))
#     def sub_mean(x):
#         x -= K.mean(x,axis=1,keepdims=True)
#         return x
#     model.add(layers.core.Lambda(sub_mean,output_shape=lambda input_shape:input_shape))
#     model.compile(optimizer='rmsprop',loss='mse')
#     return model
# model = get_submean_model()
# res=model.predict(np.random.random((3,7)))
# model.save('add_layers.h5')
# res1 = res

# model = load_model('add_layers.h5')
# model.summary()
# res=model.predict(np.random.random((3,7)))
# res1 = res