export default {
    websocketsUrl: 'ws://' + window.location.hostname + ':9090',
    annotatedTopic: { topic: 'bob/frames/annotated/resized/compressed', type: 'sensor_msgs/msg/CompressedImage' },
    bgsTopic: { topic: 'bob/frames/foreground_mask/resized/compressed', type: 'sensor_msgs/msg/CompressedImage' },
  };