import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from pybullet_utils import bullet_client
import panda_sim


import time

useGUI = False
timeStep = 1./60.

# Importing the libraries
import os
import time
import multiprocessing as mp
from multiprocessing import Process, Pipe

pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7


_RESET = 1
_CLOSE = 2
_EXPLORE = 3


def ExploreWorker(rank, num_processes, childPipe, args):
  print("hi:",rank, " out of ", num_processes)  
  import pybullet as op1
  import pybullet_data as pd
  logName=""
  p1=0
  n = 0
  space = 2 
  simulations=[]
  sims_per_worker = 10
  
  offsetY = rank*space
  while True:
    n += 1
    try:
      # Only block for short times to have keyboard exceptions be raised.
      if not childPipe.poll(0.0001):
        continue
      message, payload = childPipe.recv()
    except (EOFError, KeyboardInterrupt):
      break
    if message == _RESET:
      if (useGUI):
        p1 = bullet_client.BulletClient(op1.GUI)
      else:
        p1 = bullet_client.BulletClient(op1.DIRECT)
      p1.setTimeStep(timeStep)
      
      p1.setPhysicsEngineParameter(numSolverIterations=8)
      p1.setPhysicsEngineParameter(minimumSolverIslandSize=100)
      p1.configureDebugVisualizer(p1.COV_ENABLE_Y_AXIS_UP,1)
      p1.configureDebugVisualizer(p1.COV_ENABLE_RENDERING,0)
      p1.setAdditionalSearchPath(pd.getDataPath())
      p1.setGravity(0,-9.8,0)
      logName = str("batchsim")+str(rank)
      for j in range (3):
        offsetX = 0#-sims_per_worker/2.0*space
        for i in range (sims_per_worker):
          offset=[offsetX,0, offsetY]
          sim = panda_sim.PandaSim(p1, offset)
          simulations.append(sim)
          offsetX += space 
        offsetY += space 
      childPipe.send(["reset ok"])
      p1.configureDebugVisualizer(p1.COV_ENABLE_RENDERING,1)
      for i in range (100):
        p1.stepSimulation()
      
      logId = p1.startStateLogging(op1.STATE_LOGGING_PROFILE_TIMINGS,logName)
      continue
    if message == _EXPLORE:
      sum_rewards=rank
      
      if useGUI:
        numSteps = int(20000)
      else:
        numSteps = int(5)
      for i in range (numSteps):
        for s in simulations:
          s.step()
        p1.stepSimulation()
      #print("logId=",logId)
      #print("numSteps=",numSteps)

      childPipe.send([sum_rewards])
      continue
    if message == _CLOSE:
      p1.stopStateLogging(logId)
      childPipe.send(["close ok"])
      break
  childPipe.close()
  

if __name__ == "__main__":
  mp.freeze_support()
  if useGUI:
    num_processes = 1
  else:
    num_processes = 12
  processes = []
  args=[0]*num_processes
  
  childPipes = []
  parentPipes = []

  for pr in range(num_processes):
    parentPipe, childPipe = Pipe()
    parentPipes.append(parentPipe)
    childPipes.append(childPipe)

  for rank in range(num_processes):
    p = mp.Process(target=ExploreWorker, args=(rank, num_processes, childPipes[rank],  args))
    p.start()
    processes.append(p)
      
  
  for parentPipe in parentPipes:
    parentPipe.send([_RESET, "blaat"])
  
  positive_rewards = [0]*num_processes
  for k in range(num_processes):
    #print("reset msg=",parentPipes[k].recv()[0])
    msg = parentPipes[k].recv()[0]
  
  for parentPipe in parentPipes:
    parentPipe.send([_EXPLORE, "blaat"])
  
  positive_rewards = [0]*num_processes
  for k in range(num_processes):
    positive_rewards[k] = parentPipes[k].recv()[0]
    #print("positive_rewards=",positive_rewards[k])

  
  for parentPipe in parentPipes:
    parentPipe.send([_EXPLORE, "blaat"])
  
  positive_rewards = [0]*num_processes
  for k in range(num_processes):
    positive_rewards[k] = parentPipes[k].recv()[0]
    #print("positive_rewards=",positive_rewards[k])
    msg = positive_rewards[k]

  for parentPipe in parentPipes:
    parentPipe.send([_EXPLORE, "blaat"])
  
  positive_rewards = [0]*num_processes
  for k in range(num_processes):
    positive_rewards[k] = parentPipes[k].recv()[0]
    #print("positive_rewards=",positive_rewards[k])

  
  for parentPipe in parentPipes:
    parentPipe.send([_CLOSE, "pay2"])
  
  for p in processes:
    p.join()
  
  #now we merge the separate json files into a single one
  fnameout = 'batchsim.json'
  count = 0
  outfile = open(fnameout, "w+")
  outfile.writelines(["{\"traceEvents\":[\n"])
  numFiles = num_processes
  for num in range(numFiles):
    print("num=",num)
    fname = 'batchsim%d_0.json' % (num)
    with open(fname) as infile:
        for line in infile:
          if "pid" in line:
            line = line.replace('\"pid\":1', '\"pid\":'+str(num))
            if num < (numFiles-1) and not "{}}," in line:
              line = line.replace('{}}', '{}},')
              print("line[",count,"]=",line)
            outfile.write(line)
          count += 1
  print ("count=",count)
  outfile.writelines(["],\n"])
  outfile.writelines(["\"displayTimeUnit\": \"ns\"}\n"])
  outfile.close()
