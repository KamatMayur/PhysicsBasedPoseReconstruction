import mujoco
from mujoco import mjx
import time
import itertools
import numpy as np
from typing import Callable, NamedTuple, Optional, Union, List
import mediapy
import matplotlib

# More legible printing from numpy.
np.set_printoptions(precision=3, suppress=True, linewidth=100)
class MjxEnv(Env):
  """API for driving an MJX system for training and inference in brax."""

