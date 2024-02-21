<?php
$date = $_GET['date'] ?? '20240112'; // Default date if not provided
$jsonDirectory = dirname(__FILE__) . '/videos/' . $date . '/json/';

// Initialize detection points array outside the loop
$detectionPoints = [];

// Read JSON files from the directory
$events = [];
$totalDuration = 0; // Initialize total duration variable
$frameWidthAvailable = false;
$frameHeightAvailable = false;

if ($handle = opendir($jsonDirectory)) {
    while (false !== ($file = readdir($handle))) {
        if (pathinfo($file, PATHINFO_EXTENSION) == 'json') {
            $jsonFilePath = $jsonDirectory . '/' . $file;
            $jsonData = json_decode(file_get_contents($jsonFilePath), true);
            if ($jsonData && count($jsonData) >= 3) {
                // Extract relevant information from JSON data
                $cameraInfo = $jsonData[0]['camera_info'];
                $startTimestamp = $jsonData[1]['time_ns'] / 1e9; // Convert nanoseconds to seconds
                $endTimestamp = $jsonData[count($jsonData) - 1]['time_ns'] / 1e9; // Convert nanoseconds to seconds
                $duration = round($endTimestamp - $startTimestamp, 2);  // Calculate duration in seconds, rounded to 2 decimal places
                $eventName = pathinfo($file, PATHINFO_FILENAME);

                // Add duration to total duration
                $totalDuration += $duration;

                // Convert start timestamp to local time
                $startDateTime = new DateTime();
                $startDateTime->setTimestamp((int)$startTimestamp);
                $userTimezone = $_GET['timezone'] ?? 'Europe/London'; // Default to Europe/London if timezone is not provided
                $startDateTime->setTimezone(new DateTimeZone($userTimezone));

                // Extract camera information
                $manufacturer = $cameraInfo['manufacturer'] ?? 'N/A';
                $model = $cameraInfo['model'] ?? 'N/A';
                $fps = $cameraInfo['fps'] ?? 'N/A';

                // Extract unique detection IDs to determine the number of tracks
                $uniqueDetectionIds = [];
                foreach ($jsonData as $item) {
                    if (isset($item['detections'])) {
                        foreach ($item['detections'] as $detection) {
                            $uniqueDetectionIds[$detection['id']] = true;
                        }
                    }
                }
                $numTracks = count($uniqueDetectionIds);

                // Check if frame width and height are available
                if (isset($cameraInfo['frame_width'])) {
                    $frameWidthAvailable = true;
                }

                if (isset($cameraInfo['frame_height'])) {
                    $frameHeightAvailable = true;
                }

                // Add detection points to the array only if frame width and height are available
                if ($frameWidthAvailable && $frameHeightAvailable) {
                    foreach ($jsonData as $item) {
                        if (isset($item['detections'])) {
                            foreach ($item['detections'] as $detection) {
                                if (isset($detection['bbox']['x']) && isset($detection['bbox']['y']) &&
                                    $detection['bbox']['x'] >= 0 && $detection['bbox']['x'] <= $cameraInfo['frame_width'] &&
                                    $detection['bbox']['y'] >= 0 && $detection['bbox']['y'] <= $cameraInfo['frame_height']) {
                                    $detectionPoints[] = ['x' => $detection['bbox']['x'], 'y' => $detection['bbox']['y']];
                                }
                            }
                        }
                    }
                }

                // Construct event data array
                $events[] = [
                    'name' => $eventName,
                    'startDateTime' => $startDateTime->format('Y-m-d H:i:s'), // Format start time as desired
                    'duration' => $duration,
                    'manufacturer' => $manufacturer,
                    'model' => $model,
                    'numTracks' => $numTracks,
                    'fps' => $fps,
                    'frameWidth' => $frameWidthAvailable ? $cameraInfo['frame_width'] : 'N/A',
                    'frameHeight' => $frameHeightAvailable ? $cameraInfo['frame_height'] : 'N/A'
                ];
            }
        }
    }
    closedir($handle);
}

// Calculate total events
$totalEvents = count($events);

// Calculate total tracks
$totalTracks = 0;
foreach ($events as $event) {
    $totalTracks += $event['numTracks'];
}

?>

<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <title>Event Summary</title>
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0-alpha1/dist/css/bootstrap.min.css">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.1/css/all.min.css">
    <style>
        th,
        td {
            white-space: nowrap;
        }
    </style>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/jquery/3.6.0/jquery.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>

<body class="bg-light">

    <main class="container mt-5">
        <h4 class="mb-4">Summary of events for <?php echo $date; ?></h4>

        <!-- Scatter chart canvas -->
        <canvas id="scatterChart" width="800" height="600"></canvas>
    </main>

    <script>
        // Prepare data for the scatter chart
        const detectionPoints = <?php echo json_encode($detectionPoints); ?>;
        const frameWidth = <?php echo isset($events[0]['frameWidth']) ? $events[0]['frameWidth'] : 100; ?>;
        const frameHeight = <?php echo isset($events[0]['frameHeight']) ? $events[0]['frameHeight'] : 100; ?>;

        // Configure and render the scatter chart
        const ctx = document.getElementById('scatterChart').getContext('2d');
        const scatterChart = new Chart(ctx, {
            type: 'scatter',
            data: {
                datasets: [{
                    label: 'Detection Points',
                    data: detectionPoints,
                    backgroundColor: 'rgba(255, 99, 132, 0.5)', // Color of points
                    borderColor: 'rgba(255, 99, 132, 1)', // Border color of points
                    borderWidth: 1,
                    pointRadius: 5, // Radius of points
                }]
            },
            options: {
                scales: {
                    x: {
                        type: 'linear',
                        position: 'bottom',
                        min: 0,
                        max: frameWidth,
                        title: {
                            display: true,
                            text: 'Frame Width'
                        }
                    },
                    y: {
                        type: 'linear',
                        position: 'left',
                        min: 0,
                        max: frameHeight,
                        title: {
                            display: true,
                            text: 'Frame Height'
                        }
                    }
                }
            }
        });
    </script>
</body>

</html>
