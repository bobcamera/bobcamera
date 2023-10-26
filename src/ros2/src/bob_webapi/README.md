# WebApi Package

## Overview

The idea behind the Web Api package is to provide a bridging layer between the Web UI and the Ros2 application. 

## Nodes:

- Mask WebApi Node

### Mask WebApi Node
The Mask WebApi Node provides an endpoint for the web ui to call once it is ready to push an updated mask file down to the Ros2 (server) application. This mask file will then be placed in an appropriate location and the application configured to use the new mask file.
