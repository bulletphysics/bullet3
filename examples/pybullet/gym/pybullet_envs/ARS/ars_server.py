"""
blaze build -c opt //experimental/users/jietan/ARS:ars_server

blaze-bin/experimental/users/jietan/ARS/ars_server \
--config_name=MINITAUR_GYM_CONFIG
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
from absl import app
from absl import flags
from concurrent import futures
import grpc
from grpc import loas2
from google3.robotics.reinforcement_learning.minitaur.envs import minitaur_gym_env
from google3.robotics.reinforcement_learning.minitaur.envs import minitaur_reactive_env
from google3.robotics.reinforcement_learning.minitaur.envs.env_randomizers import minitaur_env_randomizer
from google3.robotics.reinforcement_learning.minitaur.envs.env_randomizers import minitaur_env_randomizer_from_config as randomizer_config_lib
from google3.experimental.users.jietan.ARS import ars_evaluation_service_pb2_grpc
from google3.experimental.users.jietan.ARS import ars_evaluation_service

FLAGS = flags.FLAGS
flags.DEFINE_integer("server_id", 0, "number of servers")
flags.DEFINE_integer("port", 20000, "port number.")
flags.DEFINE_string("config_name", None, "The name of the config dictionary.")
flags.DEFINE_bool('run_on_borg', False,
                  'Whether the servers are running on borg.')

_ONE_DAY_IN_SECONDS = 60 * 60 * 24


def main(unused_argv):
  servers = []
  server_creds = loas2.loas2_server_credentials()
  port = FLAGS.port
  if not FLAGS.run_on_borg:
    port = 20000 + FLAGS.server_id
  server = grpc.server(
      futures.ThreadPoolExecutor(max_workers=10), ports=(port,))
  servicer = ars_evaluation_service.ParameterEvaluationServicer(
      FLAGS.config_name, worker_id=FLAGS.server_id)
  ars_evaluation_service_pb2_grpc.add_EvaluationServicer_to_server(
      servicer, server)
  server.add_secure_port("[::]:{}".format(port), server_creds)
  servers.append(server)
  server.start()
  print("Start server {}".format(FLAGS.server_id))

  # prevent the main thread from exiting
  try:
    while True:
      time.sleep(_ONE_DAY_IN_SECONDS)
  except KeyboardInterrupt:
    for server in servers:
      server.stop(0)


if __name__ == "__main__":
  app.run(main)
