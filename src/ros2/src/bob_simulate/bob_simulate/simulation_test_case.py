import cv2
import numpy as np
from datetime import date
from rclpy.node import Node
from bob_shared.utils import frame_resize, get_optimal_font_scale

class SimulationTestCase():

  def __init__(self, node:Node, tests, dimensions, simulation_name:str='Sky360'):    
    self.node = node
    self.logger = node.get_logger()
    self.tests = tests
    self.running = True
    (self.w, self.h) = dimensions
    self.simulation_name = simulation_name
    self.frame = None
    self.today = date.today()
    self.todayStr = self.today.strftime('%b-%d-%Y')

    if self.frame is None:
      self.frame = np.full((self.h, self.w, 3) , (255, 255, 255), np.uint8)
      
    simulation_message = f"({self.todayStr}-{simulation_name}) Simulation H:{self.h}, W:{self.w}"
    fontScale = get_optimal_font_scale(simulation_message, int(self.w/2))
    cv2.putText(self.frame, simulation_message, (25, int(self.h-25)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (50, 170, 50), 2)
    #cv2.putText(self.frame, simulation_message, (25, int(self.h/2)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (50, 170, 50), 2)

  @property
  def active(self):
    return self.running

  @property
  def image(self):
    return self.frame

  @property
  def dimensions(self):
    return (self.w, self.h)

  def run(self, frame_input = None):
    if self.running:
      running = False

      if frame_input is None:
        frame_synthetic = self.frame.copy()
      else:
        frame_synthetic = frame_resize(frame_input, width=self.w, height=self.h)

      for test in self.tests:
        if test.running:
          test.run(frame_synthetic)
          running = True
    
      self.running = running
      if self.running:
        return frame_synthetic

    return self.frame
