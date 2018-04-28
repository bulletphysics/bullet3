"""Tests for google3.experimental.users.jietan.ARS.train_ars.
blaze build -c opt //experimental/users/jietan/ARS:train_ars_test
blaze-bin/experimental/users/jietan/ARS/train_ars_test
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from absl import flags
from google3.testing.pybase import googletest
from google3.experimental.users.jietan.ARS import train_ars
from google3.experimental.users.jietan.ARS import config_ars

FLAGS = flags.FLAGS
MAX_RETURN_AFTER_TWO_ITEATIONS = 0.0890905394617

class TrainArsTest(googletest.TestCase):

  def testArsTwoStepResult(self):
    config = getattr(config_ars, "MINITAUR_REACTIVE_CONFIG")
    config['num_iterations'] = 2
    info = train_ars.run_ars(config=config, logdir=FLAGS.test_tmpdir)
    print (info)
    self.assertAlmostEqual(info["max_reward"], MAX_RETURN_AFTER_TWO_ITEATIONS)


if __name__ == '__main__':
  googletest.main()
