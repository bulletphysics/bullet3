from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

#python proto2csv.py --proto_file=/tmp/logs/minitaur_log_2019-01-27-12-59-31 --csv_file=/tmp/logs/out.csv
#each line in csv contains: angle, velocity, action, torque

import tf.compat.v1 as tf
import argparse
import numpy
from pybullet_envs.minitaur.envs import minitaur_logging

flags = tf.app.flags
FLAGS = tf.app.flags.FLAGS

flags.DEFINE_string("proto_file", "logs", "path to protobuf input file")
flags.DEFINE_string("csv_file", "file.csv", "poth to csv output file")


def main(argv):
  del argv

  logging = minitaur_logging.MinitaurLogging()
  episode = logging.restore_episode(FLAGS.proto_file)
  #print(dir (episode))
  #print("episode=",episode)
  fields = episode.ListFields()

  recs = []

  for rec in fields[0][1]:
    #print(rec.time)
    for motorState in rec.motor_states:
      #print("motorState.angle=",motorState.angle)
      #print("motorState.velocity=",motorState.velocity)
      #print("motorState.action=",motorState.action)
      #print("motorState.torque=",motorState.torque)
      recs.append([motorState.angle, motorState.velocity, motorState.action, motorState.torque])

  a = numpy.array(recs)
  numpy.savetxt(FLAGS.csv_file, a, delimiter=",")


if __name__ == "__main__":
  tf.app.run(main)
