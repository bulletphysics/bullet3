"""

blaze build -c opt //experimental/users/jietan/ARS:start_ars_servers
blaze-bin/experimental/users/jietan/ARS/start_ars_servers

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import time
import subprocess
from absl import app
from absl import flags

FLAGS = flags.FLAGS
flags.DEFINE_integer("num_servers", 8, "number of servers")

def main(argv):
  del argv  # Unused.
  for server_id in xrange(FLAGS.num_servers):
    args = ["blaze-bin/experimental/users/jietan/ARS/ars_server", "--config_name=MINITAUR_GYM_CONFIG", "--server_id={}".format(server_id), "--run_on_borg=False"]
    subprocess.Popen(args)


if __name__ == '__main__':
  app.run(main)
