import cv2
import numpy as np

def decodeanddisplayvideo(url: str):
    # Create a VideoCapture object
    videoreader = cv2.cudacodec.createVideoReader(url)

    while True:
        # Decode the next frame
        ret, frame = videoreader.nextFrame()

        if not ret:
            print("Error: Could not read frame or end of stream.")
            break

        # Convert the frame to a format that can be displayed
        display_frame = frame.download()

        # Display the resulting frame
        cv2.imshow('CUDA Video Stream', display_frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the video capture object
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Replace with your URL
    video_url = "rtsp://bob:Sky360Sky!@10.20.30.35:554/Streaming/Channels/101"
    decodeanddisplayvideo(video_url)
