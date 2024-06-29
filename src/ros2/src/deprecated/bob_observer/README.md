# Observer Package

## Overview

### Description: 

This package contains nodes specific to local conditions at present these are the cloud cover estimator and day night classifier. But more will be added over time like cloud direction classifiers etc. Mainly the stuff that David has been working on.

### Vision: 

The vision for this package is to have a collection of nodes that are responsible for monitoring the local tracking environment. Maybe report stuff like temperature, GPS location, time zone, north south offsets etc.

## Nodes:

- Day Night Classifier Node
- Cloud Estimator Node
- Tracker Monitoring Node

### Day Night Classifier Node

A node that observes and determines whether its day or night.

### Cloud Estimator Node

A node that observes and determines how cloudy it currently is.

### Tracker Monitoring Node

A node that observes tracker to see if its configured correctly. We have seen examples of the tracker getting heavily confused by cloud movement, in these sort of situations we are able to report these situation to the end user so that they can for example reduce the tracker's sensitivity.