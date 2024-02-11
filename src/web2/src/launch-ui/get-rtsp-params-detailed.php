<?php
// This script takes an RTSP URL as a query parameter, uses ffmpeg to get information about the video stream, then returns the parameters as JSON.

// Check if the 'rtspUrl' query parameter is set
if (isset($_GET['rtspUrl'])) {
    // Basic sanitization of the RTSP URL. Consider more rigorous validation based on your use case.
    $rtspUrl = filter_var($_GET['rtspUrl'], FILTER_SANITIZE_URL);

    // Command to get various stream details
    $detailsCommand = "ffprobe -v error -select_streams v:0 -show_entries stream=width,height,r_frame_rate,bit_rate,codec_name,display_aspect_ratio -of json '" . escapeshellarg($rtspUrl) . "'";
    $output = shell_exec($detailsCommand);

    // Decode the JSON output from ffprobe to an array
    $details = json_decode($output, true);

    // Extract individual details. Note that some values might need processing (like frame rate).
    $width = $details['streams'][0]['width'] ?? 'Unknown';
    $height = $details['streams'][0]['height'] ?? 'Unknown';
    $frameRate = $details['streams'][0]['r_frame_rate'] ?? 'Unknown';
    $bitRate = $details['streams'][0]['bit_rate'] ?? 'Unknown';
    $codec = $details['streams'][0]['codec_name'] ?? 'Unknown';
    $aspectRatio = $details['streams'][0]['display_aspect_ratio'] ?? 'Unknown';

    // Frame rate output is usually in the form "num/den". Convert it to a float if not 'Unknown'.
    if ($frameRate !== 'Unknown') {
        list($num, $den) = explode('/', $frameRate);
        $frameRate = floatval($num) / floatval($den);
    }

    // Prepare the data as an array
    $data = [
        'width' => $width,
        'height' => $height,
        'frameRate' => $frameRate,
        'bitRate' => $bitRate,
        'codec' => $codec,
        'aspectRatio' => $aspectRatio,
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
