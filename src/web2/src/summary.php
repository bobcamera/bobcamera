<?php
$date = $_GET['date'] ?? '20240112'; // Default date if not provided
$jsonDirectory = dirname(__FILE__) . '/videos/' . $date . '/json/';

// Read JSON files from the directory
$events = [];
$totalDuration = 0; // Initialize total duration variable
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
                $frameWidth = $cameraInfo['frame_width'] ?? 'N/A';
                $frameHeight = $cameraInfo['frame_height'] ?? 'N/A';

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

                // Construct event data array
                $events[] = [
                    'name' => $eventName,
                    'startDateTime' => $startDateTime->format('Y-m-d H:i:s'), // Format start time as desired
                    'duration' => $duration,
                    'manufacturer' => $manufacturer,
                    'model' => $model,
                    'numTracks' => $numTracks,
                    'fps' => $fps,
                    'frameWidth' => $frameWidth,
                    'frameHeight' => $frameHeight
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
    <link rel="stylesheet" href="https://cdn.datatables.net/1.11.5/css/jquery.dataTables.min.css">
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0-alpha1/dist/css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.1/css/all.min.css">
    <style>
        th,
        td {
            white-space: nowrap;
        }
    </style>
    <style>
        /* Scale up the image on hover */
        .heatmap-image:hover {
            transform: scale(1.3);
            transition: transform 0.2s ease-in-out;
            z-index: 1; /* Ensure the image is above other content */
        }
    </style>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/jquery/3.6.0/jquery.min.js"></script>
    <script src="https://cdn.datatables.net/1.11.5/js/jquery.dataTables.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0-alpha1/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        $(document).ready(function() {
            $('#event-table').DataTable({
                "paging": true,
                "lengthMenu": [
                    [5, 10, 25, 50, -1],
                    [5, 10, 25, 50, "All"]
                ],
                "pageLength": 10, // Default page length
                "order": [], // Disable initial sorting
                "columnDefs": [{
                    "targets": 'no-sort',
                    "orderable": false,
                }]
            });
        });
    </script>
</head>

<body class="bg-light">

    <main class="container mt-5">
        <h4 class="mb-4">Summary of events for <?php echo $date; ?></h4>

            <!-- Summary table -->
    <table class="table table-striped">
        <thead>
            <tr>
                <th>Total Events</th>
                <th>Total Video Duration (s)</th>
                <th>Average Number of Tracks</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><?php echo $totalEvents; ?></td>
                <td><?php echo $totalDuration; ?></td>
                <td><?php echo round($totalTracks / $totalEvents, 2); ?></td>
            </tr>
        </tbody>
    </table>
    
        <div class="table-responsive">
            <!-- Event table -->
            
            <table id="event-table" class="table table-striped table-bordered">
                <thead>
                    <tr>
                        <th>Event Name</th>
                        <th>Start Time (Local)</th>
                        <th>Duration (s)</th>
                        <th>Frame Width</th>
                        <th>Frame Height</th>
                        <th>Frame Rate (fps)</th>
                        <th>Number of Tracks</th>
                        <th>Mini Heatmap</th>
                    </tr>
                </thead>
                <tbody>
                    <?php foreach ($events as $event): ?>
                        <?php
                        // Get the path to the heatmap image
                        $heatMapDirectory = 'videos/' . $date . '/heatmaps';
                        $heatMapImagePath = $heatMapDirectory . '/' . $event['name'] . '.jpg';

                        // Check if the heatmap image exists
                        $miniHeatmap = '';
                        if (file_exists($heatMapImagePath)) {
                            // Display a mini version of the heatmap image with a link to the video page
                            $miniHeatmap = '<a href="video.php?date=' . $date . '&time=' . $event['name'] . '"><img src="' . $heatMapImagePath . '" width="90" height="90"></a>';
                        }
                        ?>
                        <tr>
                            <td><a href="video.php?date=<?php echo $date; ?>&time=<?php echo $event['name']; ?>"><?php echo $event['name']; ?></a></td>
                            <td><?php echo $event['startDateTime']; ?></td>
                            <td><?php echo $event['duration']; ?></td>
                            <td><?php echo $event['frameWidth']; ?></td>
                            <td><?php echo $event['frameHeight']; ?></td>
                            <td><?php echo $event['fps']; ?></td>
                            <td><?php echo $event['numTracks']; ?></td>
                            <td>
    <?php if (file_exists($heatMapImagePath)): ?>
        <a href="video.php?date=<?php echo $date; ?>&time=<?php echo $event['name']; ?>">
            <img class="heatmap-image" src="<?php echo $heatMapImagePath; ?>" width="90" height="90">
        </a>
    <?php endif; ?>
</td>
                        </tr>
                    <?php endforeach; ?>
                </tbody>
            </table>
        </div>
    </main>

</body>

</html>
