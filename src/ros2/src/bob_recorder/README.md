# Recorder Package

## Overview

### Description: 

Uses the RosBag recorder and is responsible for recording tracks. Each track is recorded into its own database file which can then be played back. 

TODO: The recorder should also capture classification information so that it can be stored as a single bundle of information versus reclassifying at playback.

In ROS (Robot Operating System), a rosbag recorder node is a tool used for recording and playing back ROS message data. It allows you to capture the data being transmitted between different nodes in your ROS system and store it in a bag file for later analysis or playback. A rosbag recorder node subscribes to one or more topics in the ROS network and records all messages published on those topics, along with the timestamp and other metadata. The recorded data is saved in a bag file, which is a binary file format that can be easily replayed in ROS using the rosbag play tool. 

### Vision: 

Playback is done via the command line by pushing messages to the same topics that were recorded. It would be nice if this could be initiated by the UI and not the command line. There also was an initial idea of using a non intrusive recorder but that is not a free product.

## Nodes:

- RosBag Recorder Node
- Video Recorder Node

### RosBag Recorder Node

Records a track into a SqlLite file. This has not yet been implemented and is on the TODO list.

### Video Recorder Node

When the tracker is tracking an object, we start recording this event to a video file.