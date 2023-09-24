import cv2
import math
import random
import numpy as np

class ObjectSimulator():

    @staticmethod
    def Generator(settings):
        return CircleFrameGenerator(settings=settings)

    def __init__(self):
        pass

    def generate_frame(self):
        pass


class CircleFrameGenerator(ObjectSimulator):

    def __init__(self, settings, num_circles=10, size_range=(2, 10), step_range=(5, 12), background_color=(255, 255, 255)):
        super().__init__()

        self.num_circles = num_circles
        self.size_range = size_range
        self.step_range = step_range
        self.frame_size = (settings['height'], settings['width'])
        self.background_color = background_color
        self.circles = [MovingCircle(self.frame_size[1], self.frame_size[0], 1, size_range=size_range, step_range=step_range) for _ in range(num_circles)]

    def generate_frame(self):
        frame = np.full((self.frame_size[0], self.frame_size[1], 3), self.background_color, dtype=np.uint8)
        for circle in self.circles:
            circle.draw(frame, circle.color)
            circle.move()
        return frame


class MovingCircle:
    def __init__(self, frame_width, frame_height, path_linearity=0.3, size_range=(10, 20), step_range=(2, 12)):
        self.width = frame_width
        self.height = frame_height
        self.center = (self.width // 2, self.height // 2) 
        self.radiusOfMask = min(self.width, self.height) // 1.5 
        
        valid_position = False
        while not valid_position:
            self.x = random.randint(0, self.width)
            self.y = random.randint(0, self.height)
            distance = math.sqrt((self.x - self.center[0]) ** 2 + (self.y - self.center[1]) ** 2)
            if distance <= self.radiusOfMask:
                valid_position = True

        self.angle = random.uniform(0, 359)
        self.color = (10, 10, 20)
        self.size = random.randint(size_range[0], size_range[1]) 
        self.toRadians = math.pi / 180
        self.linearity = path_linearity 
        self.step_range = step_range
        self.step = random.randint(self.step_range[0], self.step_range[1])

    def move(self):
        if random.random() > self.linearity:
            angle_dist = random.uniform(-11, 6)
        else:
            angle_dist = 0  
        step_dist = random.randint(self.step_range[0], self.step_range[1])
        self.angle += angle_dist
        
        new_x = self.x + self.step * math.cos(self.angle * self.toRadians)
        new_y = self.y + self.step * math.sin(self.angle * self.toRadians)

        distance = math.sqrt((new_x - self.center[0]) ** 2 + (new_y - self.center[1]) ** 2)

        if distance >= self.radiusOfMask:
            random_angle = random.uniform(-20, 20)  
            self.angle += 180 + random_angle
            new_x = self.x + self.step * math.cos(self.angle * self.toRadians)
            new_y = self.y + self.step * math.sin(self.angle * self.toRadians)

        self.x = new_x
        self.y = new_y
        shrink_factor = 0.2
        self.size = int(4 * (1 - shrink_factor * (distance / self.radiusOfMask))) + 1

        if random.randint(0, 121) == 0:
            self.angle = angle_dist
            self.step = step_dist

    def draw(self, frame, color):
        cv2.circle(frame, (int(self.x), int(self.y)), self.size, color, -1)

# if __name__ == '__main__':
#     settings = {'width': 800, 'height': 600}
#     generator = CircleFrameGenerator(settings)

#     while True:
#         frame = generator.generate_frame()
#         cv2.imshow('Circle Simulation', frame)
#         if cv2.waitKey(30) & 0xFF == ord('q'):  # Exit on pressing 'q'
#             break

#     cv2.destroyAllWindows()