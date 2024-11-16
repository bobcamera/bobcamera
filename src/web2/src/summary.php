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
$minDuration = PHP_INT_MAX; // Initialize minimum duration variable
$maxDuration = 0; // Initialize maximum duration variable
$maxEvents = 1000; // set the max events for the page
$maxCounter = $maxEvents; // do a max number of events otherwise we get memory issues

if ($handle = opendir($jsonDirectory)) {
    while (false !== ($file = readdir($handle)) && $maxCounter > 0) {
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

                // Update minimum duration if needed
                $minDuration = min($minDuration, $duration);

                // Update maximum duration if needed
                $maxDuration = max($maxDuration, $duration);

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
                                if (
                                    $detection['state'] == 2 && isset($detection['bbox']['x']) && isset($detection['bbox']['y']) &&
                                    $detection['bbox']['x'] >= 0 && $detection['bbox']['x'] <= $cameraInfo['frame_width'] &&
                                    $detection['bbox']['y'] >= 0 && $detection['bbox']['y'] <= $cameraInfo['frame_height']
                                ) {
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

            $maxCounter--;
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

// Calculate mean duration
$meanDuration = $totalEvents > 0 ? $totalDuration / $totalEvents : 0;

?>

<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <title>Event Summary</title>
    <link rel="stylesheet" href="/assets/jquery.dataTables.min.css">
    <link href="/lib/bootstrap/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="/assets/all.min.css">
    <style>
        th,
        td {
            white-space: nowrap;
        }
    </style>
    <style>
        /* Scale up the image on hover */
        .heatmap-image:hover {
            transform: scale(2.0);
            transition: transform 0.2s ease-in-out;
            z-index: 1;
            /* Ensure the image is above other content */
        }
    </style>
    <script src="/assets/jquery.min.js"></script>
    <script src="/assets/jquery.dataTables.min.js"></script>
    <script src="/lib/bootstrap/bootstrap.bundle.min.js"></script>
    <script src="/assets/chart.js"></script>
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
        <?php if ($maxCounter == 0) echo "<h6 class=\"mb-3\">You have recorded more than $maxEvents events, if this event level persists impliment/change your detection mask or lower the sensitivity setting.</h6>" ?>

        <!-- Summary table -->
        <table class="table table-striped mt-5">
            <thead>
                <tr>
                    <th>Total Events <?php if ($maxCounter == 0) echo "(max $maxEvents)" ?></th>
                    <th>Mean Event Duration (s)</th>
                    <th>Minimum Event Duration (s)</th>
                    <th>Maximum Event Duration (s)</th>
                    <th>Average Number of Tracks</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td><?php echo $totalEvents; ?></td>
                    <td><?php echo round($meanDuration, 2); ?></td>
                    <td><?php echo $minDuration; ?></td>
                    <td><?php echo $maxDuration; ?></td>
                    <td><?php echo round($totalTracks / $totalEvents, 2); ?></td>
                </tr>
            </tbody>
        </table>

        <!-- Side-by-side charts -->
        <div class="row">
            <div class="col-md-6">
                <!-- Scatter chart canvas -->
                <canvas id="scatterChart" width="400" height="300"></canvas>
            </div>
            <div class="col-md-6">
                <!-- Histogram chart canvas -->
                <canvas id="histogramChart" width="400" height="300"></canvas>
            </div>
        </div>
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
                    <?php foreach ($events as $event) : ?>
                        <?php
                        // Get the path to the heatmap image
                        $heatMapDirectory = 'videos/' . $date . '/heatmaps';
                        $heatMapImagePath = $heatMapDirectory . '/' . $event['name'] . '.jpg';

                        // Store heatmap image path as a data attribute
                        $heatmapDataAttr = file_exists($heatMapImagePath) ? 'data-heatmap="' . $heatMapImagePath . '"' : '';
                        ?>
                        <tr <?php echo $heatmapDataAttr; ?>>
                            <td><a href="video.php?date=<?php echo $date; ?>&time=<?php echo $event['name']; ?>"><?php echo $event['name']; ?></a></td>
                            <td><?php echo $event['startDateTime']; ?></td>
                            <td><?php echo $event['duration']; ?></td>
                            <td><?php echo $event['frameWidth']; ?></td>
                            <td><?php echo $event['frameHeight']; ?></td>
                            <td><?php echo $event['fps']; ?></td>
                            <td><?php echo $event['numTracks']; ?></td>
                            <td><!-- Placeholder for the heatmap image --></td>
                        </tr>
                    <?php endforeach; ?>
                </tbody>
            </table>
        </div>
    </main>
    <script>
        $(document).ready(function() {
            // Load heatmap images for the initial page
            loadHeatmapImages();

            // Initialize DataTable only if it hasn't been initialized yet
            if (!$.fn.DataTable.isDataTable('#event-table')) {
                $('#event-table').DataTable({
                    "paging": true,
                    "lengthMenu": [
                        [5, 10, 25, 50, -1],
                        [5, 10, 25, 50, "All"]
                    ],
                    "pageLength": 10,
                    "order": [],
                    "columnDefs": [{
                        "targets": 'no-sort',
                        "orderable": false,
                    }]
                });
            }

            // Load heatmap images dynamically when table redraws
            $('#event-table').on('draw.dt', function() {
                loadHeatmapImages();
            });

            // Function to load heatmap images dynamically
            function loadHeatmapImages() {
                $('#event-table tr[data-heatmap]').each(function() {
                    var $row = $(this);
                    var heatmapPath = $row.data('heatmap');
                    if (heatmapPath) {
                        var $td = $row.find('td:last');
                        $td.html('<a href="video.php?date=<?php echo $date; ?>&time=' + $row.find('td:first a').text() + '">' +
                            '<img class="heatmap-image" src="' + heatmapPath + '" width="80" height="80"></a>');
                    }
                });
            }
        });
    </script>

    <script>
        <?php
        // Prepare data for the scatter chart
        $detectionPoints = isset($detectionPoints) ? json_encode($detectionPoints) : '[]';
        $frameWidth = isset($events[0]['frameWidth']) ? $events[0]['frameWidth'] : 100;
        $frameHeight = isset($events[0]['frameHeight']) ? $events[0]['frameHeight'] : 100;
        ?>

        // Configure and render the scatter chart if frame width and height are available
        <?php if ($frameWidth !== 'N/A' && $frameHeight !== 'N/A') { ?>
            const scatterCtx = document.getElementById('scatterChart').getContext('2d');

            const scatterChart = new Chart(scatterCtx, {
                type: 'scatter',
                data: {
                    datasets: [{
                        label: 'Active Track Detection Points',
                        data: <?php echo $detectionPoints; ?>,
                        borderColor: 'rgba(255, 99, 132, 1)', // Border color of points
                        borderWidth: 0.5,
                        pointRadius: 4, // Radius of points
                        pointBackgroundColor: 'transparent', // Set point fill color to transparent
                        indexAxis: 'x', // Set index axis to 'x'
                    }]
                },
                options: {
                    parsing: false, // Disable data parsing
                    plugins: {
                        decimation: {
                            enabled: true,
                            algorithm: 'lttb',
                            samples: 8,
                            threshold: 40
                        },
                        tooltip: {
                            enabled: false // Disable tooltip overview
                        }
                    },
                    elements: {
                        point: {
                            drawActiveElementsOnTop: false // Disable drawing active elements on top
                        }
                    },
                    scales: {
                        x: {
                            type: 'linear',
                            position: 'bottom',
                            min: 0,
                            max: <?php echo $frameWidth; ?>,
                            title: {
                                display: true,
                                text: 'Frame Width'
                            }
                        },
                        y: {
                            type: 'linear',
                            position: 'left',
                            min: 0,
                            max: <?php echo $frameHeight; ?>,
                            title: {
                                display: true,
                                text: 'Frame Height'
                            }
                        }
                    }
                }
            });

        <?php } ?>

        // Prepare data for the histogram chart
        const events = <?php echo json_encode($events); ?>;

        // Extract timestamps and round them to nearest hour
        const roundedTimestamps = events.map(event => {
            const timestamp = new Date(event.startDateTime).getTime();
            const roundedTimestamp = Math.floor(timestamp / (1000 * 60 * 60)) * (1000 * 60 * 60);
            return roundedTimestamp;
        });

        // Count events for each rounded timestamp
        const eventCounts = {};
        roundedTimestamps.forEach(roundedTimestamp => {
            eventCounts[roundedTimestamp] = (eventCounts[roundedTimestamp] || 0) + 1;
        });

        // Sort event counts by timestamp
        const sortedEventCounts = Object.entries(eventCounts).sort(([timestamp1], [timestamp2]) => timestamp1 - timestamp2);

        // Extract labels and data for the histogram
        const labels = sortedEventCounts.map(([timestamp]) => {
            const date = new Date(parseInt(timestamp));
            return date.toLocaleString('en-US', {
                hour: 'numeric',
                hour12: true
            });
        });
        const data = sortedEventCounts.map(([, count]) => count);

        // Configure and render the histogram chart
        const histogramCtx = document.getElementById('histogramChart').getContext('2d');
        const histogramChart = new Chart(histogramCtx, {
            type: 'bar',
            data: {
                labels: labels,
                datasets: [{
                    label: 'Event Count',
                    data: data,
                    backgroundColor: 'rgba(54, 162, 235, 0.5)', // Color of bars
                    borderColor: 'rgba(54, 162, 235, 1)', // Border color of bars
                    borderWidth: 1
                }]
            },
            options: {
                scales: {
                    x: {
                        title: {
                            display: true,
                            text: 'Time'
                        }
                    },
                    y: {
                        beginAtZero: true,
                        title: {
                            display: true,
                            text: 'Event Count'
                        }
                    }
                }
            }
        });

        document.addEventListener("DOMContentLoaded", () => {
            const settings = JSON.parse(localStorage.getItem('settings')) || {
                timezone: 'UTC'
            };
            const userTimezone = settings.timezone;
            document.querySelectorAll('#event-table tbody tr').forEach(tr => {
                // Assume the first <td> contains the Unix timestamp
                const unixTimestamp = parseInt(tr.children[0].textContent.trim());
                console.log("Unix Timestamp:", unixTimestamp); // Debugging
                // Convert Unix timestamp (seconds since the epoch) to milliseconds and create a Date object
                const eventDate = new Date(unixTimestamp * 1000);
                // Format the date to the local time zone
                const options = {
                    timeZone: userTimezone,
                    year: 'numeric',
                    month: 'short',
                    day: 'numeric',
                    hour: '2-digit',
                    minute: '2-digit',
                    second: '2-digit',
                    hour12: true // or false for 24-hour format
                };
                const localTime = new Intl.DateTimeFormat('en-US', options).format(eventDate);
                // Update the second <td> with the formatted local time
                tr.children[1].textContent = localTime;
            });
        });
    </script>
</body>

</html>