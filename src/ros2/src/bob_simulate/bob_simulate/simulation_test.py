import cv2

from .synthetic_data import SyntheticData

class SimulationTest():

  def __init__(self, synthetic_data: SyntheticData, target_object_diameter=2, loop=True):
    self.synthetic_data = synthetic_data
    self.target_object_diameter = target_object_diameter
    self.loop = loop
    self.initialised = False
    self.counter = 0
    self.path_length = 0
    self.running = True

  @property
  def active(self):
    return self.running

  def run(self, target_frame):

    if not self.initialised:
      self.path_length = len(self.synthetic_data.get_path())
      (h,w) = self.synthetic_data.get_frame_size()
      target_shape = target_frame.shape[:2]
      f_h = target_shape[0]
      f_w = target_shape[1]
      self.factor_h = f_h/h
      self.factor_w = f_w/w
      self.initialised = True

    if self.path_length > 0:
      if self.counter < self.path_length:
        (x,y) = self.synthetic_data.get_path()[self.counter]
        target_y = int(y*self.factor_h)
        target_x = int(x*self.factor_w)                
        cv2.circle(target_frame, (target_x, target_y), self.target_object_diameter, (25,25,25), -1)
        self.counter = self.counter + 1
      else:
        if self.loop:
          self.counter = 0
        else:
          self.running = False
        
