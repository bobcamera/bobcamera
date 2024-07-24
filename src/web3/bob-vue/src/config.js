export default {
    websocketsUrl: 'ws://' + window.location.hostname + ':9090',
    plainCameraTopic: { topic: 'bob/frames/allsky/original/resized/compressed', info: 'bob/camera/all_sky/camera_info' },
    annotatedTopic: { topic: 'bob/frames/annotated/resized/compressed', info: 'bob/camera/all_sky/camera_info' },
    bgsTopic: { topic: 'bob/frames/foreground_mask/resized/compressed', info: 'bob/camera/all_sky/camera_info' },
  };