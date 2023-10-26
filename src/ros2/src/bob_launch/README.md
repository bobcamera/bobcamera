# Launch Package

## Overview

## Launch Files:

- Application Launch File
- Update Config Files Launch File
- Kernel Launch File
- Display Infrastructure Launch File
- Monitoring Infrastructure Launch File
- Web Infrastructure Launch File
- QHY Launch File

### Application Launch File

This is the "entry point" launch file which defines several launch parameters which are driven using environment variables. These environment variables will determine which nodes are run using if launch conditions. Needless to say this launch file will in turn calls all the other launch files.

### Update Config Files Launch File

A launch file whose primary purpose is to update both the application and camera yaml configuration files. The yaml configuration files contain all the node parameters which the nodes use to define their processing behaviour.

### Kernel Launch File

Manages is the main image processing pipeline. It runs as a single container process and all nodes communicate via RPC, not network. As resolutions increase we have found ourselves in a position where the loopback address gets saturated and therefore slows down the processing pipeline. Passing data via IPC avoids this from happening. All images available external to the kernel should be resized appropriately therefore any node that requires the full image will need to be part of the kernel process.

### Display Infrastructure Launch File

Manages all nodes required for display to work. This will include viewer and compressor nodes. Viewer nodes are used outside of the web ui while compressor nodes are used by the web ui to transform frames into compressed jpg images which a browser understands.

### Monitoring Infrastructure Launch File

Manages all nodes dealing with monitoring and observing the system. These include for example nodes that determine if its day or night, what sort of cloud cover are we dealing with, what is the condition of the tracker and and also a node that is able to feed data into prometheus so that you can Visualise that data using Grafana.

### Web Infrastructure Launch File

Manages nodes dealing with web stuff, currently the main ones are the ROS Bridge server as well as the WebApi nodes.

### QHY Launch File
