"""see https://machinelearningmastery.com/multivariate-time-series-forecasting-lstms-keras/"""

import matplotlib
matplotlib.use('TkAgg')
import numpy as np
from matplotlib import pyplot
from pandas import read_csv
from pandas import DataFrame
from pandas import concat
from sklearn.preprocessing import MinMaxScaler
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import mean_squared_error
from keras.models import Sequential
from keras.layers import Dropout
from keras.layers import Dense
from keras.layers import LSTM
from keras.callbacks import ModelCheckpoint
from matplotlib import pyplot

from pandas import read_csv


# convert series to supervised learning
def series_to_supervised(data, n_in=1, n_out=1, dropnan=True):
  n_vars = 1 if type(data) is list else data.shape[1]
  df = DataFrame(data)
  cols, names = list(), list()
  # input sequence (t-n, ... t-1)
  for i in range(n_in, 0, -1):
    cols.append(df.shift(i))
    names += [('var%d(t-%d)' % (j + 1, i)) for j in range(n_vars)]
  # forecast sequence (t, t+1, ... t+n)
  for i in range(0, n_out):
    cols.append(df.shift(-i))
    if i == 0:
      names += [('var%d(t)' % (j + 1)) for j in range(n_vars)]
    else:
      names += [('var%d(t+%d)' % (j + 1, i)) for j in range(n_vars)]
  # put it all together
  agg = concat(cols, axis=1)
  agg.columns = names
  # drop rows with NaN values
  if dropnan:
    agg.dropna(inplace=True)
  return agg


values = [x for x in range(10)]
data = series_to_supervised(values, 2)
print(data)

# load dataset
#dataset = read_csv('./data/minitaur_log_latency_0.01.csv')
#dataset = read_csv('./data/minitaur_log_latency_0.003.csv')
dataset = read_csv('./data/minitaur_log_latency_0.006.csv')

values = dataset.values
# integer encode direction
#encoder = LabelEncoder()
#values[:,3] = encoder.fit_transform(values[:,3])
# ensure all data is float
values = values.astype('float32')

# normalize features
useNormalization = False
if useNormalization:
  scaler = MinMaxScaler(feature_range=(0, 1))
  scaled = scaler.fit_transform(values)
else:
  scaled = values

# frame as supervised learning
lag_steps = 5
reframed = series_to_supervised(scaled, lag_steps, 1)
print("reframed before drop=", reframed)

# drop columns we don't want to predict
reframed.drop(reframed.columns[[3, 7, 11, 15, 19]], axis=1, inplace=True)
print("after drop=", reframed.head())

#dummy = scaler.inverse_transform(reframed)
#print(dummy)

groups = [0, 1, 2, 3]

i = 1
# plot each column
doPlot = False
if doPlot:
  pyplot.figure()
  for group in groups:
    pyplot.subplot(len(groups), 1, i)
    pyplot.plot(values[0:25, group])
    pyplot.title(dataset.columns[group], y=0.5, loc='right')
    i += 1
  pyplot.show()

# split into train and test sets
values = reframed.values
n_train_hours = 6000
train = values[:n_train_hours, :]
test = values[n_train_hours:, :]
# split into input and outputs
train_X, train_y = train[:, :-1], train[:, -1]
test_X, test_y = test[:, :-1], test[:, -1]

print("train_X.shape[1]=", train_X.shape[1])

# design network
useLSTM = True
if useLSTM:
  # reshape input to be 3D [samples, timesteps, features]
  train_X = train_X.reshape(
      (train_X.shape[0], lag_steps + 1, int(train_X.shape[1] / (lag_steps + 1))))
  test_X = test_X.reshape((test_X.shape[0], lag_steps + 1, int(test_X.shape[1] / (lag_steps + 1))))
  model = Sequential()
  model.add(LSTM(40, input_shape=(train_X.shape[1], train_X.shape[2])))
  model.add(Dropout(0.05))
  model.add(Dense(8, activation='sigmoid'))
  model.add(Dense(8, activation='sigmoid'))
  model.add(Dropout(0.05))
  model.add(Dense(1, activation='linear'))
else:
  # create model
  model = Sequential()
  model.add(Dense(12, input_dim=train_X.shape[1], activation="relu"))
  model.add(Dense(8, activation="sigmoid"))
  model.add(Dense(1, activation="linear"))

#model.compile(loss='mae', optimizer='adam')
model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mean_squared_error'])

# checkpoint
filepath = '/tmp/keras/weights-improvement-{epoch:02d}-{val_loss:.2f}.hdf5'
checkpoint = ModelCheckpoint(filepath,
                             monitor='val_loss',
                             verbose=1,
                             save_best_only=True,
                             mode='min')
callbacks_list = [checkpoint]

# fit network
history = model.fit(train_X,
                    train_y,
                    epochs=1500,
                    batch_size=32,
                    callbacks=callbacks_list,
                    validation_data=(test_X, test_y),
                    verbose=2,
                    shuffle=False)
# plot history

data = np.array([[[
    1.513535008329887299, 3.234624992847829894e-01, 1.731481043119239782, 1.741165415165205399,
    1.534267104753672228e+00, 1.071354965017878635e+00, 1.712386127673626302e+00
]]])

#prediction = model.predict(data)
#print("prediction=",prediction)

pyplot.plot(history.history['loss'], label='train')
pyplot.plot(history.history['val_loss'], label='test')
pyplot.legend()
pyplot.show()
