from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from absl import app
from absl import flags
from keras.callbacks import ModelCheckpoint
from keras.layers import Dense
from keras.models import Sequential
from keras import optimizers
import numpy

FLAGS = flags.FLAGS

flags.DEFINE_string(
    "input_filename", "data/minitaur_log_latency_0.01.csv", "The name of the input CSV file."
    "Each line in the CSV file will contain the motor position, the "
    "motor speed, action and torques.")


def main(unused_argv):
  # fix random seed for reproducibility
  numpy.random.seed(7)
  # load pima indians dataset
  dataset = numpy.loadtxt(FLAGS.input_filename, delimiter=",")
  # split into input (X) and output (Y) variables
  x = dataset[:, 0:3]
  y = dataset[:, 3]
  print("x=", x)
  print("y=", y)

  # create model
  model = Sequential()
  model.add(Dense(12, input_dim=3, activation="relu"))
  model.add(Dense(8, activation="sigmoid"))
  model.add(Dense(1, activation="linear"))

  # Compile model (use adam or sgd)
  model.compile(loss="mean_squared_error", optimizer="adam", metrics=["mean_squared_error"])

  # checkpoint
  filepath = "/tmp/keras/weights-improvement-{epoch:02d}-{val_loss:.2f}.hdf5"
  checkpoint = ModelCheckpoint(filepath,
                               monitor="val_loss",
                               verbose=1,
                               save_best_only=True,
                               mode="min")
  callbacks_list = [checkpoint]

  # Fit the model
  # model.fit(X, Y, epochs=150, batch_size=10)
  # model.fit(X, Y, epochs=150, batch_size=10, callbacks=callbacks_list)
  model.fit(x,
            y,
            validation_split=0.34,
            epochs=4500,
            batch_size=1024,
            callbacks=callbacks_list,
            verbose=0)

  # evaluate the model
  scores = model.evaluate(x, y)
  print("\n%s: %.2f%%" % (model.metrics_names[1], scores[1] * 100))


if __name__ == "__main__":
  app.run(main)
