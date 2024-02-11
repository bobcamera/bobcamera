<?php

// This script takes an RTSP URL as a query parameter, uses ffmpeg to get the height, width and fps for the video stream, then returns the parameters as JSON.

// Check if the 'rtspUrl' query parameter is set
if (isset($_GET['rtspUrl'])) {
    // Basic sanitization of the RTSP URL. Consider more rigorous validation based on your use case.
    $rtspUrl = filter_var($_GET['rtspUrl'], FILTER_SANITIZE_URL);

    // Ensure the URL is a valid RTSP URL here. This is a placeholder for actual validation logic.
    // This is crucial for security to prevent command injection.

    // Command to get width
    $widthCommand = "ffprobe -v error -select_streams v:0 -show_entries stream=width -of default=noprint_wrappers=1:nokey=1 '" . escapeshellarg($rtspUrl) . "'";
    $width = shell_exec($widthCommand);

    // Command to get height
    $heightCommand = "ffprobe -v error -select_streams v:0 -show_entries stream=height -of default=noprint_wrappers=1:nokey=1 '" . escapeshellarg($rtspUrl) . "'";
    $height = shell_exec($heightCommand);

    // Command to get frame rate
    $frameRateCommand = "ffprobe -v error -select_streams v:0 -show_entries stream=r_frame_rate -of default=noprint_wrappers=1:nokey=1 '" . escapeshellarg($rtspUrl) . "'";
    $frameRateOutput = shell_exec($frameRateCommand);

    // Frame rate output is usually in the form "num/den". Convert it to a float.
    list($num, $den) = explode('/', $frameRateOutput);
    $frameRate = floatval($num) / floatval($den);

    // Prepare the data as an array
    $data = [
        'width' => trim($width),
        'height' => trim($height),
        'frameRate' => $frameRate,
    ];

    // Set header to indicate JSON response
    header('Content-Type: application/json');

    // Return JSON encoded data
    echo json_encode($data);
} else {
    // Error message for missing RTSP URL parameter
    echo json_encode(['error' => 'RTSP URL parameter is missing.']);
}

?>