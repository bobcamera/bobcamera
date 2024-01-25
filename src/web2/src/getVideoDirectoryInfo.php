<?php
// Sanitize and validate input
$date = isset($_GET['date']) ? preg_replace("/[^0-9]/", "", $_GET['date']) : '20240112';

// Directories for video and heatmap files
$videoDirectory = dirname(__FILE__) . '/videos/' . $date . '/allsky/' . $date . '/';
$heatMapDirectory = dirname(__FILE__) . '/videos/' . $date . '/heatmaps/';

// Check if directories exist
if (!is_dir($videoDirectory) || !is_dir($heatMapDirectory)) {
    echo json_encode(['error' => 'Directory does not exist']);
    exit;
}

// Get list of videos
$videos = array_diff(scandir($videoDirectory), array('..', '.'));

// Check if videos are found
if (empty($videos)) {
    echo json_encode(['error' => 'No videos found']);
    exit;
}

// Determine the time either from GET parameter or from the second video file
$time = isset($_GET['time']) ? preg_replace("/[^0-9]/", "", $_GET['time']) : substr($videos[2], 0, -4);

// Construct file paths
$heatMapImagePath = $heatMapDirectory . '/' . $date . '/' . $time . '.png';
$heatMapPath = $heatMapDirectory . '/heatmap-timelapse-' . $date . '.mp4';
$videoPath = $videoDirectory . $time . '.mp4';

// Timezone and DateTime format (unused in this snippet)
$timeZone = new DateTimeZone('America/New_York');
$dateTimeFormatOptions = [
    'European format' => 'Y-m-d H:i:s',
    'American format' => 'm/d/Y H:i:s',
    'Asian format' => 'Y-m-d H:i:s',
    'Australian format' => 'Y-m-d H:i:s',
    'African format' => 'Y-m-d H:i:s',
];
$dateTimeFormat = $dateTimeFormatOptions['European format'];

// Output JSON-encoded data
echo json_encode([
    'videos' => $videos,
    'heatMapImagePath' => $heatMapImagePath,
    'heatMapPath' => $heatMapPath,
    'videoPath' => $videoPath
]);
?>
