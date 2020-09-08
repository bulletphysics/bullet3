from xml.etree import ElementTree as ET
from os import path
import pybullet as p

def log(txt, level):
  print("log: ", txt)

def only_contains_int(stringlist):
    """Checks if a list of strings contains int numbers exclusively.
    
    To determine whether the number is an int, the function `is_int` is used.
    Args:
      stringlist(list(str): list to check
    Returns:
      : bool -- True if every string in the list can be represented as int, False if not
    """
    for num in stringlist:
        if not is_int(num):
            return False
    return True


def only_contains_float(stringlist):
    """Checks if a list of strings contains float numbers exclusively.
    
    To determine whether the number is a float, the function `is_float` is used.
    Args:
      stringlist(list(str): list to check
    Returns:
      : bool -- True if every string in the list can be represented as float, False if not
    """
    for num in stringlist:
        if not is_float(num):
            return False
    return True


def is_float(text):
    """Tests if the specified string represents a float number.
    Args:
      text(str): text to check
    Returns:
      : bool -- True if the text can be parsed to a float, False if not.
    """
    try:
        float(text)
        return True
    except (ValueError, TypeError):
        return False


def parse_number(text):
    """Returns the specified string parsed to an int or float.
    
    If no number can be parsed, the original string is returned.
    
    To determine whether the number is an int or float, the functions `is_int` and `is_float` are
    used.
    Args:
      text(string): text to parse to a number
    Returns:
      : int/float/str -- depending on successful parsing, a number or a string is returned
    """
    if is_int(text):
        return int(text)
    elif is_float(text):
        return float(text)
    return text

def is_int(text):
    """Tests if the specified string represents an integer number.
    Args:
      text(str): text to check
    Returns:
      : bool -- True if the text can be parsed to an int, False if not.
    """
    try:
        int(text)
        return True
    except ValueError:
        return False


def only_contains_int(stringlist):
    """Checks if a list of strings contains int numbers exclusively.
    
    To determine whether the number is an int, the function `is_int` is used.
    Args:
      stringlist(list(str): list to check
    Returns:
      : bool -- True if every string in the list can be represented as int, False if not
    """
    for num in stringlist:
        if not is_int(num):
            return False
    return True

def parse_text(text):
    """Parses a text by splitting up elements separated by whitespace.
    
    The elements are then parsed to int/float-only lists.
    Args:
      text(str): text with elements seperated by whitespace
    Returns:
      : list(str/float/int) -- list with elements parsed to the same type
    """
    numstrings = text.split()
    if not numstrings:
        return None

    if len(numstrings) > 1:
        # int list
        if only_contains_int(numstrings):
            nums = [int(num) for num in numstrings]
            return nums
        # float list
        elif only_contains_float(numstrings):
            nums = [float(num) for num in numstrings]
            return nums
        # return a string list
        return numstrings
    return parse_number(text)



    
def parseModel(model_xml, urdffilepath):
    print("--------------------")
    print("parsing model")
    newmodel = {a: model_xml.attrib[a] for a in model_xml.attrib}
    newmodel['children'] = []
    include_xml = model_xml.find('include')
    if include_xml is not None:
      uri_xml = include_xml.find('uri')
      if uri_xml is not None:
        uri = uri_xml.text
        
        prefix = "model://"
        if uri.startswith(prefix):
          uri = "models/"+uri[len(prefix):]+"/model.sdf"
        print("uri=", uri)
        newmodel['uri'] = uri
    
    pose_xml = model_xml.find('pose')
    if pose_xml is not None:
      pose = parse_text(pose_xml.text)
    if pose and len(pose)==6:
        newmodel['pose_xyz'] = pose[:3]
        newmodel['pose_rpy'] = pose[3:]
        print("newmodel['pose_xyz']=",newmodel['pose_xyz'])
        print("newmodel['pose_rpy']=",newmodel['pose_rpy'])
    
    
    return newmodel


def parseWorld(p, filepath):
  # load element tree from file
  tree = ET.parse(filepath)
  root = tree.getroot()
  if 'version' in root.attrib:
      print("version=",root.attrib['version'])

  models = {}
  log("Parsing models...", 'INFO')
  for model_xml in root.iter('model'):
      log(" Adding link {}.".format(model_xml.attrib['name']), 'DEBUG')
      newmodel = parseModel(model_xml, "")
      models[model_xml.attrib['name']] = newmodel
      if 'uri' in newmodel.keys() and newmodel['uri'] is not None:
        bodies = p.loadSDF(newmodel['uri'])
        #transform all bodies using the 'pose'
        for b in bodies:
          old_pos,old_orn = p.getBasePositionAndOrientation(b)
          pose_xyz = newmodel['pose_xyz']
          pose_rpy = newmodel['pose_rpy']
          pose_orn = p.getQuaternionFromEuler(pose_rpy)
          new_pos, new_orn = p.multiplyTransforms(pose_xyz, pose_orn, old_pos,old_orn)
          p.resetBasePositionAndOrientation(b, new_pos, new_orn)
          p.changeDynamics(b, -1, mass=0)
        print("bodies=",bodies)
  
  return models    
  
  