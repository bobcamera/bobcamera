import cv2
import numpy as np
from bob_shared.utils import get_optimal_font_scale

class SimulationTestCaseRunner():

  def __init__(self, test_cases):
    self.test_cases = test_cases
    self.completed_test_cases = []
    self.running_test_case = None
    self.completed_frame = None
    if len(self.test_cases) > 0:
      self.running_test_case = self.test_cases[0]
      self.completed_frame_dimensions = self.running_test_case.dimensions #(w,h)

  @property
  def active(self):
    return self.running_test_case is not None

  @property
  def image(self):
    return self.running_test_case.image

  def run(self, frame_input = None):

    if not self.running_test_case is None:

      if not self.running_test_case.active:
        self.completed_test_cases.append(self.running_test_case)
        self.test_cases.remove(self.running_test_case)
        self.running_test_case = None        

      if self.running_test_case is None:
        if len(self.test_cases) > 0:
          self.running_test_case = self.test_cases[0]
          self.completed_frame_dimensions = self.running_test_case.dimensions #(w,h)

      if not self.running_test_case is None:
        return self.running_test_case.run(frame_input = frame_input)

    return self._completed_frame()

  def _completed_frame(self):

    if self.completed_frame is None:
      (w, h) = self.completed_frame_dimensions
      self.completed_frame = np.full((w, h, 3) , (255, 255, 255), np.uint8)      
      simulation_message = f"Simulation Complete"
      fontScale = get_optimal_font_scale(simulation_message, int(w-50))
      cv2.putText(self.completed_frame, simulation_message, (25, int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (50, 170, 50), 2)

    return self.completed_frame
