# Tracking Package

## Overview

### Description: 

Package provides track details depending on how many objects are being tracked. It also tried to provide a historical trajectory for each track as well as a future projection using a Kalman filter. It also published a summary of track details i.e. how many objects have been tracked, how many are currently tracked etc.

### Vision: 

It should be responsible for all tracking related data. It’s responsible for determining if an object is a known object or a new object that is just being tracked. You should be able to tell if it needs to ignore a known tracked object as it’s been identified as a plane or a bird by the classifier or do we have another node that makes that decision i.e. ignore this track, focus on that track etc.

## Nodes:

- Track Provider Node

### Track Provider Node

The track provider node 