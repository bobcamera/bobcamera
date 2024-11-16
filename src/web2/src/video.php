<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Video Display</title>
    <link rel="stylesheet" href="/assets/plyr.css" />

    <style>
        body {
            background-color: #333;
            color: #fff;
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
        }

        .video-container {
            position: relative;
            /* max-width: 100%; */
            /* margin: 20px auto; */
        }

        #player {
            height: 90%;
        }
    </style>
</head>
<body>
    <?php
        $date = $_GET['date'] ?? '20240112';
        $time = $_GET['time'] ?? 'defaultTime';

        // Read the JSON file
        $jsonFile = 'videos/' . $date . '/json/' . $time . '.json';
        $jsonData = file_exists($jsonFile) ? json_decode(file_get_contents($jsonFile), true) : [];

        // Extract camera information from the JSON data
        $cameraInfo = isset($jsonData[0]['camera_info']) ? $jsonData[0]['camera_info'] : [];

        // Extract the MP4 file name
        $mp4FileName = $time . '.mp4';
    ?>

    <div class="video-container">
        <video id="player" playsinline controls autoplay muted>
            <source src="<?php echo 'videos/' . $date . '/allsky/' . $mp4FileName; ?>" />
        </video>
    </div>

    <!-- Summary table -->
    <div class="summary-table">
        <h2>Video Summary</h2>
        <table>
            <tr>
                <th>Attribute</th>
                <th>Value</th>
            </tr>
            <?php foreach ($cameraInfo as $key => $value): ?>
                <tr>
                    <td><?php echo ucfirst(str_replace('_', ' ', $key)); ?></td>
                    <td><?php echo $value; ?></td>
                </tr>
            <?php endforeach; ?>
            <tr>
                <td>MP4 File Name</td>
                <td><?php echo $mp4FileName; ?></td>
            </tr>
        </table>
    </div>

    <!-- Plyr.js Library -->
    <script src="/assets/plyr.polyfilled.js"></script>
    <script>
        document.addEventListener('DOMContentLoaded', () => {
            const player = new Plyr('#player');
        });
    </script>
</body>
</html>
