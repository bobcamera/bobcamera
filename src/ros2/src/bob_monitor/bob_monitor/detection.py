import math

import cv2
import numpy as np

from bob_interfaces.msg import TrackDetection
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D

# https://stackoverflow.com/questions/36254452/counting-cars-opencv-python-issue/36274515#36274515

class Detection(object):
	def __init__ (self, id, position:Pose2D):
		self.id = id
		
		self.positions = [position]
		self.frames_since_seen = 0
		self.counted = False

		self.speed = None

	@property
	def last_position (self) -> Point2D:
		return self.positions[-1]
	
	def add_position (self, new_position):
		self.positions.append(new_position)
		self.frames_since_seen = 0

class DetectionCounter2 (object):
	def __init__(self, logger):
		self.logger = logger
		self.detections = {}
		self.max_unseen_frames = 10

	@staticmethod
	def get_vector (a, b):
		dx = float(b.x - a.x)
		dy = float(b.y - a.y)

		distance = math.sqrt(dx**2 + dy**2)

		if dy > 0:
			angle = math.degrees(math.atan(-dx/dy))
		elif dy == 0:
			if dx < 0:
				angle = 90.0
			elif dx > 0:
				angle = -90.0
			else:
				angle = 0.0
		else:
			if dx < 0:
				angle = 180 - math.degrees(math.atan(dx/dy))
			elif dx > 0:
				angle = -180 - math.degrees(math.atan(dx/dy))
			else:
				angle = 180.0

		return distance, angle

	def update(self, track_detections):

    	#https://github.com/ronitsinha/speed-detector

		for track_detection in track_detections:
			if track_detection.id in self.detections:
				detection = self.detections[track_detection.id]
			else:
				detection = Detection(track_detection.id, track_detection.bbox.center.position)
				self.detections[track_detection.id] = detection

			vector = self.get_vector(detection.last_position, track_detection.bbox.center.position)
			distance = int(vector[0])
			self.logger.info(f'{detection.id} has a speed of {distance} pixels / frame.')

			detection.add_position(track_detection.bbox.center.position)
