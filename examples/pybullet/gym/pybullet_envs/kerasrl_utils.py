import re
from gym import error
import glob
#  checkpoints/KerasDDPG-InvertedPendulum-v0-20170701190920_actor.h5
weight_save_re = re.compile(r'^(?:\w+\/)+?(\w+-v\d+)-(\w+-v\d+)-(\d+)(?:_\w+)?\.(\w+)$')


def get_fields(weight_save_name):
  match = weight_save_re.search(weight_save_name)
  if not match:
    raise error.Error(
        'Attempted to read a malformed weight save: {}. (Currently all weight saves must be of the form {}.)'
        .format(id, weight_save_re.pattern))
  return match.group(1), match.group(2), int(match.group(3))


def get_latest_save(file_folder, agent_name, env_name, version_number):
  """
	Returns the properties of the latest weight save. The information can be used to generate the loading path
	:return:
	"""
  path = "%s%s" % (file_folder, "*.h5")
  file_list = glob.glob(path)
  latest_file_properties = []
  file_properties = []
  for f in file_list:
    file_properties = get_fields(f)
    if file_properties[0] == agent_name and file_properties[1] == env_name and (
        latest_file_properties == [] or file_properties[2] > latest_file_properties[2]):
      latest_file_properties = file_properties

  return latest_file_properties
