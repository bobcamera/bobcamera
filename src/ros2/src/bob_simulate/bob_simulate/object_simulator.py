import cv2
import numpy as np

class ObjectSimulator():

    @staticmethod
    def Generator():
        return CircleFrameGenerator()

    def __init__(self):
        pass

    def generate_frame(self):
        pass


class CircleFrameGenerator(ObjectSimulator):

    def __init__(self, num_circles=5, circle_size=(2, 10), speed_range=(5, 50), frame_size=(1080, 1920)):
        super().__init__()

        self.num_circles = num_circles
        self.circle_size = circle_size
        self.speed_range = speed_range
        self.frame_size = frame_size
        self.circles = [self._random_circle() for _ in range(num_circles)]

    def _random_circle(self):
        x = np.random.randint(0, self.frame_size[1])
        y = np.random.randint(0, self.frame_size[0])
        size = np.random.randint(self.circle_size[0], self.circle_size[1])
        speed = np.random.randint(self.speed_range[0], self.speed_range[1])
        angle = np.random.randint(0, 360)
        return {'center': (x, y), 'size': size, 'speed': speed, 'angle': angle}

    def generate_frame(self):
        frame = np.zeros((self.frame_size[0], self.frame_size[1], 3), dtype=np.uint8)
        for circle in self.circles:
            x, y = circle['center']
            color = (0, 255, 0)
            frame = cv2.circle(frame, (x, y), circle['size'], color, -1)
            dx = circle['speed'] * np.cos(np.radians(circle['angle']))
            dy = circle['speed'] * np.sin(np.radians(circle['angle']))
            circle['center'] = (int(x + dx) % self.frame_size[1], int(y + dy) % self.frame_size[0])
        return frame


if __name__ == '__main__':
    generator = CircleFrameGenerator()

    while True:
        frame = generator.generate_frame()
        cv2.imshow('Circle Simulation', frame)
        if cv2.waitKey(30) & 0xFF == ord('q'):  # Exit on pressing 'q'
            break

    cv2.destroyAllWindows()