<?php
    $date = $_GET['date'];
    if (!isset($date)) { $date = '20240112'; }

    $videoDirectory = dirname(__FILE__) . '/videos/' . $date . '/allsky/';
    $videoDirectory2 = 'videos/' . $date . '/allsky/';
    $videos = array_diff(scandir($videoDirectory), array('..', '.'));
    $time = substr($videos[2], 0, -4);

    if (!isset($_GET['time'])) {
        $time = substr($videos[2], 0, -4);
    } else {
        $time = $_GET['time'];
    }

    $heatMapDirectory = dirname(__FILE__) . '/videos/' . $date . '/heatmaps';
    $heatMapImagePath = $heatMapDirectory . '/' . $time . '.jpg';
    $heatMapPath = $heatMapDirectory . '/heatmap-timelapse-' . $date . '.mp4';
    $videoPath = $videoDirectory . $time . '.mp4';

    $heatmapPath2 = '/videos/' . $date . '/heatmaps';

    $timeZone = new DateTimeZone('America/New_York');
    $dateTimeFormatOptions = [
        'Eurpoean format' => 'Y-m-d H:i:s',
        'American format' => 'm/d/Y H:i:s',
        'Asian format' => 'Y-m-d H:i:s',
        'Australian format' => 'Y-m-d H:i:s',
        'African format' => 'Y-m-d H:i:s',
    ];
    $dateTimeFormat = 'Eurpoean format';
    $dateTimeFormat = $dateTimeFormatOptions[$dateTimeFormat];

    // JSON FILES
    // Define the JSON directory path
    $jsonDirectory = dirname(__FILE__) . '/videos/' . $date . '/json/';
?>

<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <title>BOB the Universal Object Tracker</title>
    <link rel="stylesheet" href="https://cdn.plyr.io/3.6.2/plyr.css" />
    <link rel="stylesheet" href="https://golden-layout.com/files/latest/css/goldenlayout-base.css" />
    <!-- <link rel="stylesheet" href="https://golden-layout.com/files/latest/css/goldenlayout-light-theme.css" /> -->
    <link type="text/css" rel="stylesheet" href="https://golden-layout.com/assets/css/goldenlayout-dark-theme.css" />
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.1/css/all.min.css" integrity="sha512-DTOQO9RWCH3ppGqcWaEA1BIZOC6xxalwEsw9c2QQeAIftl+Vegovlnee1c9QX4TctnWMn13TZye+giMm8e2LwA==" crossorigin="anonymous" referrerpolicy="no-referrer" />
    <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/echarts@latest/dist/echarts.min.js"></script>
    <script src="https://cdn.plyr.io/3.6.2/plyr.polyfilled.js"></script>
    <script src="https://golden-layout.com/files/latest/js/goldenlayout.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/dat-gui/0.7.9/dat.gui.min.js" integrity="sha512-WoO4Ih0CDOSLYafy22wZD/mcJ7k0ESLqtQsFa6zFKnEUrbtuGU+GkLtVhgt93xa2qewG5gKEC6CWlN8OaCTSVg==" crossorigin="anonymous" referrerpolicy="no-referrer"></script>
    <style>
        /* CSS for the loading spinner */
        .spinner-overlay {
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background-color: rgba(255, 255, 255, 0.5); /* Semi-transparent white overlay */
            display: flex;
            justify-content: center;
            align-items: center;
            z-index: 9999; /* Ensure it's above other content */
        }

        .spinner {
            width: 40px;
            height: 40px;
            background-color: #333;
            border-radius: 100%;
            -webkit-animation: sk-scaleout 1.0s infinite ease-in-out;
            animation: sk-scaleout 1.0s infinite ease-in-out;
        }

        @-webkit-keyframes sk-scaleout {
            0% {
                -webkit-transform: scale(0);
            }
            100% {
                -webkit-transform: scale(1.0);
                opacity: 0;
            }
        }

        @keyframes sk-scaleout {
            0% {
                -webkit-transform: scale(0);
                transform: scale(0);
            }
            100% {
                -webkit-transform: scale(1.0);
                transform: scale(1.0);
                opacity: 0;
            }
        }
        body .lm_content {
            overflow: scroll;
        }
        #timeline {
            width: 100%;
            height: 100%;
        }
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
        }

        #layoutContainer {
            width: 100%;
            height: 100vh;
        }

        .horizontal-scroll {
            display: flex;
            overflow-x: scroll;
            overflow-y: hidden;
            white-space: nowrap;
        }

        .horizontal-scroll a {
            margin-right: 10px;
        }
        .plyr__play-large {
            display: none;
        }
        .controls {
            font: 12px Arial, sans-serif;
            color:#fff;
            padding: 10px;
        }
        #brightnessSlider {
            width: 100%;
        }

        #plyr-overlay {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            z-index: 1;
            pointer-events: none;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        #bigHeatmapImage {
            opacity: 0.3;
            max-height: 100%;
        }

        .highlighted {
            border: 1px solid greenyellow; /* Change as needed */
            border-radius: 5px;
        }
        /* Add your CSS styles here */
        #player-container {
            position: relative;
            width: 100%; /* Adjust width as needed */
            height: 600px; /* Adjust height as needed */
        }

        #contolContainer{
            position: absolute;
            top: 5px;
            left: 18px;
            z-index: 10;
            color: rgba(255, 255, 255, 1);
            font-family: Arial, sans-serif;
            font-size: 13px;
            font-variant-numeric: tabular-nums;
            font-weight: 300;
            text-shadow: none;
        }

         #fisheyeViewer{
            position: absolute;
            top: 20px;
            right: 18px;
            z-index: 10;
        }

        .plyr__control {
            color: white;
            background: none;
            border: none;
            padding: 10px;
            cursor: pointer;
        }
        .custom-control {
            background: none; /* Removes the background */
            border: none; /* Removes the border */
            padding: 0; /* Removes padding */
            margin: 0; /* Adjust as needed */
            cursor: pointer; /* Changes the cursor to a pointer to indicate it's clickable */
            outline: none; /* Removes the outline on focus */
        }
        .custom-control i {
            color: white; /* Or set a specific color */
            font-size: inherit; /* Adjust the size as needed */
        }
        .divider-style {
            border-left: 2px solid darkgrey; /* Adjust the width and color as needed */
            padding-left: 10px; /* Adds some space between the border and the content */
        }

        #player-container {
            position: relative;
            width: 100%; /* Adjust width as needed */
            height: 600px; /* Adjust height as needed */
            font-family: Arial, sans-serif; /* Ensures text is legible */
        }

        #video-overlay {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            padding: 10px;
            background: rgba(0, 0, 0, 0.5); /* Semi-transparent black for text visibility */
            color: white;
            font-size: 14px;
            z-index: 20; /* Above the video but below interactive elements */
        }

    </style>
</head>
<body>

<div class="spinner-overlay">
    <div class="spinner"></div>
</div>

<div id="layoutContainer"></div>

<script>
    var myChart;
    let currentVideo = '<?php echo $videoPath; ?>';
    let currentVideoStartTimestamp = '<?php echo $time; ?>';
    const frameRate = 25; // Replace with your video's frame rate
    const frameDuration = 1 / frameRate;
    var date = <?php echo $date; ?>;
    var jsonData = '';
    var jsonDirectory = '<?php echo $jsonDirectory; ?>';
    let isDragging = false;
    let startX, startY;
    let translateX = 0, translateY = 0;
    let brightnessSlider;

    document.addEventListener('DOMContentLoaded', function() {
        // Initialize GoldenLayout
        var config = {
            content: [{
                type: 'column',
                content: [{
                        type: 'row',
                        content: [{
                                type: 'stack',
                                width: 80,
                                content: [{
                                        type: 'component',
                                        componentName: 'Video Player',
                                        componentState: {
                                            label: 'Video Player'
                                        }
                                    }
                                ]
                            },
                            {
                                type: 'stack',
                                content: [{
                                        type: 'component',
                                        componentName: 'Recordings',
                                        componentState: {
                                            label: 'Recordings'
                                        }
                                    },
                                    {
                                        type: 'component',
                                        componentName: 'Controls',
                                        id: 'videoInfoPanel',
                                        componentState: {
                                            label: 'Controls'
                                        }
                                    }
                                ]
                            }
                        ]
                    },
                    {
                        type: 'row',
                        height: 25,
                        content: [{
                                type: 'stack',
                                content: [{
                                        type: 'component',
                                        componentName: 'Timeline',
                                        componentState: {
                                            label: 'Timeline'
                                        }
                                    }
                                ]
                            }
                        ]
                    }
                ]
            }]
        };

        var myLayout = new GoldenLayout(config, document.getElementById('layoutContainer'));

        // Register a component
        myLayout.registerComponent('Timeline', function(container, componentState) {
            container.getElement().html('<div id="timeline"></div>');
        });

        // Register CONTROLLS
        myLayout.registerComponent('Controls', function(container, componentState) {
            container.getElement().html(`
                <div id="videoInfoContent">Video info will be displayed here</div>
            `);
        });

        myLayout.registerComponent('Video Player', function(container, componentState) {
            container.getElement().html(`

                <div id="player-container">

                    <video
                        id="player"
                        playsinline controls>
                            <source
                            src="<?php echo 'videos/' . $date . '/allsky/' . $time . '.mp4'; ?>" />
                    </video>
                    <canvas id="screenshot-canvas" style="display:none;"></canvas>

                    <div
                        id="video-overlay"
                        style="
                            position: absolute;
                            top: 0;
                            left: 0;
                            width: 100%;
                            padding: 10px;
                            background: rgba(0, 0, 0, 0.5);
                            color: white;
                            font-size: 14px;
                            z-index: 10;"
                        >
                    </div>
                    <div
                        id="plyr-overlay"
                        style="
                        position: absolute;
                        top: 0;
                        left: 0;
                        width: 100%;
                        height: 100%;
                        z-index: 1;
                        pointer-events: none;
                        ">
                            <img
                            id="bigHeatmapImage"
                            src=""
                            style="height: 100%;
                            opacity: 0.3;">
                        </div>

                        <div id="contolContainer">

                            <br>
                            File: <span id="displayFileName"><?php echo $time; ?>.mp4</span><br>
                            Recording Time: <?php
                            function getValuesFromJsonFile($filePath, $keys) {
                                if (!file_exists($filePath)) {
                                    return array_fill_keys($keys, null);
                                }
                                $jsonString = file_get_contents($filePath);
                                $jsonData = json_decode($jsonString);
                                $results = [];
                                foreach ($keys as $key) {
                                    $results[$key] = isset($jsonData->$key) ? $jsonData->$key : null;
                                }
                                return $results;
                            }
                            $filePath = 'json/stationSettings.json';
                            $keys = ['timezone', 'lat', 'long'];
                            $data = getValuesFromJsonFile($filePath, $keys);
                            $timezone = $data['timezone'] ?? 'America/New_York';
                            $latitude = $data['lat'] ?? '0.0000';
                            $longitude = $data['long'] ?? '0.0000';
                            $dateTimeStamp = new DateTime('@' . $time);
                            $dateTimeStamp->setTimezone(new DateTimeZone($timezone));
                            echo "Time: <span id='displayRecordingTime'>" . $dateTimeStamp->format('h:i:s A') . "</span><br>";
                            ?>
                            Recording Date: <span id='displayRecordingDate'><?php echo date('Y-m-d', strtotime($date)); ?></span><br>
                            <div style="margin-bottom: 9px;">
                            TimeZone: <?php echo $timezone . "<br>"; ?>
                            <?php echo "Latitude: " . $latitude . "<br>"; ?>
                            <?php echo "Longitude: " . $longitude . "<br>"; ?>
                            </div>

                            Opacity:
                            <br>
                            <input
                            type="range"
                            id="opacitySlider"
                            title="Heatmap Overlay Opacity"
                            min="0"
                            max="100"
                            value="30"
                            style="
                                z-index: 10;
                                width: 140px;
                                margin-left: 0px;
                                margin-bottom: 9px;"
                            >
                            <br>
                            Brightness:
                            <br>
                            <input
                            type="range"
                            id="brightnessSlider"
                            title="Video Brightness"
                            min="0"
                            max="200"
                            value="100"
                            style="
                                z-index: 10;
                                width: 140px;
                                margin-bottom: 8px;
                                margin-left: 0px;"
                            >
                            <br>

                        </div>

                        <button
                        id="fisheyeViewer"
                        class="custom-control"
                        title="Open in the Fisheye Viewer">
                            <i
                            class="fa-solid fa-expand"
                            style="font-size: 20px;
                            background-color: rgba(255, 255, 255, 0.15);
                            margin: 5px;
                            padding: 5px;
                            border-radius: 4px;"
                            ></i>
                        </button>
            </div>
            `);
            container.on('open', function() {

                const video = document.getElementById('player');

                const brightnessSlider = document.getElementById('brightnessSlider');

                const opacitySlider = document.getElementById('opacitySlider');

                const heatmapImage = document.getElementById('bigHeatmapImage');

                // Brightness slider for the video
                brightnessSlider.addEventListener('input', function() {
                    video.style.filter = `brightness(${this.value}%)`;
                });

                // Opacity slider for the heatmap overlay
                opacitySlider.addEventListener('input', function() {
                    heatmapImage.style.opacity = `${this.value / 100}`;
                });

                // after page load
                document.addEventListener('DOMContentLoaded', function() {
                    // Custom controls - One frame back
                    document.getElementById('goBackOneFrame').addEventListener('click', () => {
                        goBackOneFrame();
                    });

                    // Custom controls - One frame forward
                    document.getElementById('advanceOneFrame').addEventListener('click', () => {
                        advanceOneFrame();
                    });

                    // Custom controls - One frame forward
                    document.getElementById('fisheyeViewer').addEventListener('click', () => {
                        window.open('html/fisheye-video-viewer-v6.html?date=<?php echo $date; ?>&video=<?php echo $time; ?>', '_blank');
                    });
                });

            });
        });

        // Register the Video List component
        myLayout.registerComponent('Recordings', function(container, componentState) {
            <?php if (is_dir($videoDirectory)): ?>
            let htmlContent = "<div class='divider-style' style='display: grid; grid-template-columns: 1fr; gap: 10px;'>";
            <?php foreach ($videos as $video): ?>
                <?php $videoName = substr($video, 0, -4); ?>
                htmlContent += "<div id='videoListContainer2' style='width: 100%;' class='horizontal-scroll'>";
                htmlContent += "<a href='#' onclick='changeVideo(\"<?= 'videos/'. $date . '/allsky/' . $video ?>\")' title='<?= $videoName ?>'>";
                htmlContent += '<img class="heatmap-thumbnail" src="videos/<?= $date ?>/heatmaps/<?= $videoName ?>.jpg" style="width: 100%; height: auto;" title="Video: <?= $videoName ?>">'; // Adjusted image style here
                htmlContent += "</a></div>";
            <?php endforeach; ?>
            htmlContent += "</div>";
            <?php else: ?>
                let htmlContent = "The directory does not exist.";
            <?php endif; ?>
            container.getElement().html(htmlContent);
        });

        myLayout.init();

        const player = new Plyr('#player', {
            controls: [
                'restart', // Restart playback
                'rewind', // Rewind by the seek time (default 10 seconds)
                'play', // Play/pause playback
                'fast-forward', // Fast forward by the seek time (default 10 seconds)
                'progress', // The progress bar and scrubber for playback and buffering
                'current-time', // The current time of playback
                'duration', // The full duration of the media
                'mute', // Toggle mute
                'volume', // Volume control
                'captions', // Toggle captions
                'settings', // Settings menu
                'pip', // Picture-in-picture (currently Safari only)
                'airplay', // Airplay (currently Safari only)
                'download', // Show a download button with a link
                // 'fullscreen' // Toggle fullscreen
            ]
        });

        // Update video info when the video data is loaded
        player.on('loadeddata', () => {
            let videoData = {
                duration: player.duration,
                currentTime: player.currentTime,
            };
            updateVideoInfoPanel(videoData);
        });

        function updateVideoInfoPanel(videoData) {
            var videoInfoComponent = myLayout.root.getItemsById('videoInfoPanel')[0];
            if (videoInfoComponent) {
                let content = `
                <div class="controls">
                    <p>Recording File Name: <?php echo $time; ?>.mp4</p>
                    <p>Recording File Location: <?php echo 'videos/' . $date . '/allsky/' . $date; ?></p>
                    <p>Duration: ${videoData.duration} seconds</p>
                    <p>Current Time: ${videoData.currentTime} seconds</p>
                    <p><input type="number" id="frameInput" placeholder="Go to frame #">
                    <button onclick="goToFrame(document.getElementById('frameInput').value)">Go to Frame</button></p>
                    <p><div>
                        <input type="number" id="timeInput" placeholder="Time in seconds">
                        <button onclick="goToTime(document.getElementById('timeInput').value)">Go to Time</button>
                    </div></p>
                    <p><div>
                        <input type="number" id="frameRateInput" placeholder="Frame Rate" value="25">
                        <button onclick="setFrameRate(document.getElementById('frameRateInput').value)">Enter Frame Rate of Video</button>
                    </div></p>
                    </div>
                `;
                videoInfoComponent.container.getElement().html(content);
            }
        }
    });

        document.addEventListener('DOMContentLoaded', function() {
            <?php
                $directory = $jsonDirectory;
                $videos = [];
                $logFilePath = 'logfile.txt';
                function extractTimestampFromFilename($filename) {
                    return intval(pathinfo($filename, PATHINFO_FILENAME));
                }
                $earliestTimestamp = PHP_INT_MAX;
                $latestTimestamp = 0;
                if ($handle = opendir($directory)) {
                    while (false !== ($file = readdir($handle))) {
                        if (pathinfo($file, PATHINFO_EXTENSION) == 'json') {
                            $jsonFilePath = $directory . '/' . $file;
                            $jsonData = json_decode(file_get_contents($jsonFilePath), true);
                            if ($jsonData && count($jsonData) >= 3) {
                                $timestamp = extractTimestampFromFilename($file);
                                $startTimestamp = $jsonData[1]['time_ns'];
                                $endTimestamp = $jsonData[count($jsonData) - 1]['time_ns'];
                                $duration = ($endTimestamp - $startTimestamp) / 1e9;  // ns to s
                                $videoName = str_replace('.json', '.mp4', $file);
                                // Construct video data array
                                $videos[] = [
                                    'name' => $videoName,
                                    'startTimestamp' => $timestamp,
                                    'duration' => $duration
                                ];
                                $earliestTimestamp = min($earliestTimestamp, $timestamp);
                                $latestTimestamp = max($latestTimestamp, $timestamp + $duration);
                            } else {
                                // Log error if JSON data doesn't match expected structure
                            }
                        }
                    }
                    closedir($handle);
                } else {
                }
                $adjustedStart = $earliestTimestamp - 60; // One minute before the first clip
                $adjustedEnd = $latestTimestamp + 60; // One minute after the last clip
                $adjustedStartMillis = round($adjustedStart * 1000);
                $adjustedEndMillis = round($adjustedEnd * 1000);
                echo 'var videoData = ' . json_encode($videos) . ';';
                echo 'var adjustedStart = ' . $adjustedStartMillis . ';';
                echo 'var adjustedEnd = ' . $adjustedEndMillis . ';';
            ?>

            updateHeatmapSrc('<?php echo 'videos/'. $date .'/heatmaps/'. $time . '.jpg'; ?>');
            myChart = echarts.init(document.getElementById('timeline'));
            var option = {
                tooltip: {
                    trigger: 'item',
                    formatter: function(params) {
                        return 'Video: ' + params.name
                        + '<br>Start: ' + new Date(params.value[0]).toLocaleString()
                        + '<br>Duration: ' + params.value[2] + ' seconds';
                    }
                },
                xAxis: {
                    type: 'time',
                    min: adjustedStart,
                    max: adjustedEnd,
                    axisLabel: {
                        margin: 5,
                    }
                },
                yAxis: {
                    type: 'category',
                    data: ['Videos'],
                    show: false
                },
                series: [{
                    name: 'Videos',
                    type: 'scatter',
                    symbolSize: function (data) {
                        return 10;
                    },
                    data: videoData.map(function (item) {
                        return {
                            name: item.name,
                            value: [
                                item.startTimestamp * 1000, // Convert to milliseconds
                                0, // Dummy Y-axis value
                                item.duration
                            ],
                            itemStyle: {
                                color: 'rgba(0, 120, 178, 0.8)'
                            }
                        };
                    }),
                    emphasis: {
                        itemStyle: {
                            color: 'greenyellow',
                            borderColor: '#000',
                            borderWidth: 1
                        },
                        label: {
                            show: true
                        }
                    }
                }]
            };
            myChart.setOption(option);
            myChart.on('click', function (params) {
                if (params.componentType === 'series' && params.seriesType === 'scatter') {
                    let videoName = params.data.name; // Get the video file name
                    let newSource = '<?php echo $videoDirectory2; ?>' + videoName;
                    changeVideo(newSource);
                }
            });
            let currentFrameNumber = getCurrentFrame();

            player.addEventListener('timeupdate', function() {
                const currentFrameNumber = getCurrentFrame();
            });
        });

        function goToFrame(frameNumber) {
            const newTime = frameNumber / frameRate;
            player.currentTime = newTime;
        }

        function getCurrentVideoJsonData() {
            $.ajax({
                url: 'videos/'
                + date + '/json/' + currentVideoStartTimestamp + '.json',
                dataType: 'json',
                success: function(data) {
                    jsonData = data;
                }
            });
        }

        function getCurrentVideoName(text) {
            return currentVideoStartTimestamp;
        }

        function advanceOneFrame() { player.currentTime += frameDuration; }

        function goBackOneFrame() { player.currentTime -= frameDuration; }

        function goToTime(timeInSeconds) { player.currentTime = timeInSeconds; }

        function unhighlightAllPoints() {
                var option = myChart.getOption();
                for (var seriesIndex = 0; seriesIndex < option.series.length; seriesIndex++) {
                    myChart.dispatchAction({
                        type: 'downplay',
                        seriesIndex: seriesIndex
                    });
                }
            }

        function updateHeatmapSrc(newSrc) {
            $('#heatmap').attr('src', newSrc);
            $('#heatmap').css('max-width', '100%');
            $('#heatmap').css('max-height', '100%');
            var heatmapImg = document.getElementById('bigHeatmapImage');
            heatmapImg.src = newSrc;
            heatmapImg.style.opacity = '0.3';
        }

        function changeVideo(videoName) {
            heatmapName = videoName.split(".")[0];
            heatmapName = heatmapName.split("/").pop();
            let heatmapPath = 'videos/<?php echo $date; ?>/heatmaps/' + heatmapName + '.jpg';
            updateHeatmapSrc(heatmapPath);
            var heatmapName = videoName.split(".")[0].split("/").pop();
            $('img').removeClass('highlighted');
            var selector = ".heatmap-thumbnail[src*='" + heatmapName + "']";
            $(selector).addClass('highlighted');
            unhighlightAllPoints();
            var specificTimestamp = heatmapName;
            currentVideoStartTimestamp = specificTimestamp;
            highlightPointByTimestamp1(specificTimestamp);
            let newSource = videoName;
            let videoElement = document.querySelector('#player source');
            videoElement.setAttribute('src', newSource);
            player.load();
            player.play();
            getCurrentVideoJsonData();
            updateDownloadLinks(newSource);
            updateFisheyeViewerLink(heatmapName);
            updateRecordingInformationInOverlay();
        }

        function updateRecordingInformationInOverlay() {
            let displayFileNameElement = document.getElementById('displayFileName');
            let fileName = currentVideoStartTimestamp + '.mp4';
            displayFileNameElement.textContent = fileName;
            let displayRecordingTimeElement = document.getElementById('displayRecordingTime');
            let videoTimeStamp = currentVideoStartTimestamp;
            let displayTimeElement = document.getElementById('displayTime');
            let displayRecordingDateElement = document.getElementById('displayRecordingDate');
            let convertedDate = new Date(videoTimeStamp * 1000);
            let date = convertedDate.toDateString();
            let time = convertedDate.toLocaleTimeString();
            displayRecordingTimeElement.textContent = time;
            displayRecordingDateElement.textContent = date;
            displayTimeElement.textContent = time;
        }

        function updateDownloadLinks(newSource) {
            let downloadButton = document.querySelector('.plyr__controls [data-plyr="download"]');
            if (downloadButton) {downloadButton.setAttribute('href', newSource);}
            let heatmapDownloadButton = document.getElementById('heatmap-btn');
            heatmapDownloadButton.setAttribute('href', 'heatmap-' + newSource + '.jpg');
            let screenshotDownloadButton = document.getElementById('screenshot-btn');
            let screenShotTime = getCurrentTime();
            screenshotDownloadButton.setAttribute('href', 'screenshot-' + screenShotTime + '-' +  '.png');
        }

        function updateFisheyeViewerLink(heatmapName) {
            let fisheyeViewerButton = document.getElementById('fisheyeViewer');
            if (fisheyeViewerButton) {
                var new_element = fisheyeViewerButton.cloneNode(true);
                fisheyeViewerButton.parentNode.replaceChild(new_element, fisheyeViewerButton);
                fisheyeViewerButton = new_element;
                fisheyeViewerButton.addEventListener('click', function() {
                    window.open('html/fisheye-video-viewer-v6.html?date=<?php echo $date; ?>&video='+heatmapName, '_blank');
                });
            }
        }

        function highlightPointByTimestamp(timestamp) {
            var seriesIndex = 0;
            var dataIndex = -1;
            var seriesData = myChart.getOption().series[seriesIndex].data;
            for (var i = 0; i < seriesData.length; i++) {
                if (seriesData[i].value[0] === timestamp) {
                    dataIndex = i;
                    break;
                }
            }
            if (dataIndex !== -1) {
                myChart.dispatchAction({
                    type: 'highlight',
                    seriesIndex: seriesIndex,
                    dataIndex: dataIndex
                });
                myChart.dispatchAction({
                    type: 'showTip',
                    seriesIndex: seriesIndex,
                    dataIndex: dataIndex
                });
            }
        }

        function highlightPointByTimestamp1(timestamp) {
            var seriesIndex = 0;
            var dataIndex = -1;
            var seriesData = myChart.getOption().series[seriesIndex].data;
            for (var i = 0; i < seriesData.length; i++) {
                if (seriesData[i].value[0] === (parseInt(timestamp+'000'))) {
                    dataIndex = i;
                    break;
                }
            }
            if (dataIndex !== -1) {
                highlightPoint(seriesIndex, dataIndex);
            }
        }

        function highlightPoint(seriesIndex, dataIndex) {
            myChart.dispatchAction({
                type: 'highlight',
                seriesIndex: seriesIndex,
                dataIndex: dataIndex
            });
        }

        function getCurrentFrame() {
            const currentTime = player.currentTime;
            return Math.floor(currentTime * frameRate);
        }

        // Adding Custom Control Buttons to the Video Player
        document.addEventListener('DOMContentLoaded', function() {
            addCustomPlyrButton('fa fa-camera', takeScreenshot, 'Download Video Frame', 'screenshot-btn');
            addCustomPlyrButton('fa fa-thermometer-full', downloadHeatmap, 'Download Heatmap', 'heatmap-btn');
            addCustomPlyrButton('fas fa-step-forward', advanceOneFrame, 'Advance 1 Frame', 'advanceOneFrame');
            addCustomPlyrButton('fas fa-step-backward', goBackOneFrame, 'Back 1 Frame', 'goBackOneFrame');
        });

        // use plyr to download the video
        function downloadVideo() {
            const video = document.querySelector('#player source');
            const videoUrl = video.getAttribute('src');
            const downloadLink = document.createElement('a');
            downloadLink.href = videoUrl;
            downloadLink.download = videoUrl.split('/').pop();
            document.body.appendChild(downloadLink);
            downloadLink.click();
            document.body.removeChild(downloadLink);
        }

        // download heatmap function
        function downloadHeatmap() {
            const heatmap = document.getElementById('bigHeatmapImage');
            const heatmapUrl = heatmap.src;
            const downloadLink = document.createElement('a');
            downloadLink.href = heatmapUrl;
            downloadLink.download = 'heatmap-'+ currentVideoStartTimestamp +'.jpg';
            document.body.appendChild(downloadLink);
            downloadLink.click();
            document.body.removeChild(downloadLink);
        }

        function takeScreenshot() {
            const video = document.querySelector('#player html5');
            const canvas = document.getElementById('screenshot-canvas');
            const context = canvas.getContext('2d');
            if (video.readyState >= 2) {
                canvas.width = video.videoWidth;
                canvas.height = video.videoHeight;
                context.drawImage(video, 0, 0, canvas.width, canvas.height);
                const img = document.getElementById('screenshot-img');
                img.src = canvas.toDataURL('image/png');
                img.style.display = 'block';
            } else {
                alert('Video is not ready for screenshots.');
            }
        }

        function addCustomPlyrButton(faIconClass, actionFunction, buttonLabel, elementId) {
            const plyrControls = document.querySelector('.plyr__controls');
            if (plyrControls) {
                const button = document.createElement('button');
                button.className = 'plyr__controls__item plyr__control';
                button.type = 'button';
                button.id = elementId;
                button.style = 'margin-top: 2px; margin-bottom: 2px;'; // Adjust as needed
                button.innerHTML = `<i class="${faIconClass}" aria-hidden="true"></i><span class="plyr__sr-only">${buttonLabel}</span>`;
                button.onclick = actionFunction;
                plyrControls.insertBefore(button, plyrControls.firstChild);
            }
        }

        function getCurrentTime() {
            if (!player) {
                return 0;
            } else {
                return player.currentTime;
            }
        }

        document.addEventListener('DOMContentLoaded', function() {
            const video = document.getElementById('player');
            const canvas = document.getElementById('screenshot-canvas');
            const screenshotButton = document.getElementById('screenshot-btn');
            if (!screenshotButton.clicked) {
                screenshotButton.addEventListener('click', function(event) {
                    event.preventDefault();
                    captureAndDownloadScreenshot();
                });
                screenshotButton.clicked = true;
            }

            function captureAndDownloadScreenshot() {
                if (canvas && video.readyState >= 2) {
                    if (!video.paused) video.pause();
                    const context = canvas.getContext('2d');
                    canvas.width = video.videoWidth;
                    canvas.height = video.videoHeight;
                    context.drawImage(video, 0, 0, canvas.width, canvas.height);
                    const imageUrl = canvas.toDataURL('image/png');
                    const downloadLink = document.createElement('a');
                    downloadLink.href = imageUrl;
                    secondsIntoVideo = getCurrentTime();
                    let seconds = currentVideoStartTimestamp + secondsIntoVideo;
                    downloadLink.download = 'screenshot-' + seconds + '.png';
                    document.body.appendChild(downloadLink);
                    downloadLink.click();
                    document.body.removeChild(downloadLink);
                } else {
                    alert('Video is not ready for screenshots.');
                }
            }
        });

</script>

<script>
        window.addEventListener('load', function () {
            document.querySelector('.spinner-overlay').style.display = 'none';
        });
</script>
</body>
</html>
