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
    // Construct the JSON file path
    $jsonFilePath = $jsonDirectory . $time . '.json';
    // Initialize a variable to hold the JSON data
    $jsonData = null;
    // Check if the JSON file exists and then read it
    if (file_exists($jsonFilePath)) {
        $jsonContent = file_get_contents($jsonFilePath);

        $jsonData = json_decode($jsonContent, true); // 'true' converts it to an associative array
    } else {
        echo "JSON file not found: " . $jsonFilePath;
    }
?>

<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <title>Bob Player</title>
    <link rel="stylesheet" href="https://cdn.plyr.io/3.6.2/plyr.css" />
    <link rel="stylesheet" href="https://golden-layout.com/files/latest/css/goldenlayout-base.css" />
    <!-- <link rel="stylesheet" href="https://golden-layout.com/files/latest/css/goldenlayout-light-theme.css" /> -->
    <link type="text/css" rel="stylesheet" href="https://golden-layout.com/assets/css/goldenlayout-dark-theme.css" />
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
    var jsonData = <?php echo json_encode($jsonData); ?>;
    var jsonFilePath = '<?php echo $jsonFilePath; ?>';
    var jsonDirectory = '<?php echo $jsonDirectory; ?>';
    var jsonContent = <?php echo $jsonContent; ?>;
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
                                width: 66,
                                content: [{
                                        type: 'component',
                                        componentName: 'Video Player',
                                        componentState: {
                                            label: 'Video Player'
                                        }
                                    }
                                    // ,
                                    // {
                                    //     type: 'component',
                                    //     componentName: 'Heatmap',
                                    //     componentState: {
                                    //         label: 'Heatmap'
                                    //     }
                                    // },
                                    // {
                                    //     type: 'component',
                                    //     componentName: 'JSON',
                                    //     componentState: {
                                    //         label: 'JSON'
                                    //     }
                                    // }
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

        // // Register a component
        // myLayout.registerComponent('Heatmap', function(container, componentState) {
        //     container.getElement().html('<img id="heatmap" src="" alt="Heatmap" width="100%" height="100%">');
        // });

        // // Register a component
        // myLayout.registerComponent('JSON', function(container, componentState) {
        //     container.getElement().html('<div id="json" class="controls" style="white-space: pre-wrap; font-family: monospace;"></div>');
        // });

        // Register a component
        myLayout.registerComponent('Timeline', function(container, componentState) {
            container.getElement().html('<div id="timeline"></div>');
        });

        // Register CONTROLLS
        myLayout.registerComponent('Controls', function(container, componentState) {
            container.getElement().html(`
            <div id="videoInfoContent">Video info will be displayed here</div>
            <input type="range" id="brightnessSlider" min="0" max="200" value="100" style="position: absolute; z-index: 10; width: 150px;">
            `);
        });


        // myLayout.registerComponent('Video Player', function(container, componentState) {
        //     container.getElement().html(`
        //     <video id="player" playsinline controls>
        //         <source src="<?php echo 'videos/' . $date . '/allsky/' . $time . '.mp4'; ?>" />
        //     </video>
        //     <input type="range" id="brightnessSlider" min="0" max="200" value="100" style="position: absolute; z-index: 10; width: 150px;">
        //     <div id="plyr-overlay" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 1; pointer-events: none;">
        //         <img id="bigHeatmapImage" src="" style="height: 100%; opacity: 0.3;">
        //     </div>`);

        //     container.on('open', function() {
        //         const video = document.getElementById('player');
        //         const slider = document.getElementById('brightnessSlider');
        //         slider.addEventListener('input', function() {
        //             video.style.filter = `brightness(${this.value}%)`;
        //         });
        //     });
        // });

        myLayout.registerComponent('Video Player', function(container, componentState) {
            container.getElement().html(`
            <div id="player-container">
            <video id="player" playsinline controls>
                <source src="<?php echo 'videos/' . $date . '/allsky/' . $time . '.mp4'; ?>" />
            </video>
            <input type="range" id="brightnessSlider" min="0" max="200" value="100" style="position: absolute; z-index: 10; width: 150px; top: 55px; left: 10px;">
            <input type="range" id="opacitySlider" min="0" max="100" value="30" style="position: absolute; z-index: 10; width: 150px; top: 20px; left: 10px;">
            <div id="plyr-overlay" style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 1; pointer-events: none;">
                <img id="bigHeatmapImage" src="" style="height: 100%; opacity: 0.3;">
            </div></div>`);

            container.on('open', function() {
                const video = document.getElementById('player');
                const brightnessSlider = document.getElementById('brightnessSlider');
                const opacitySlider = document.getElementById('opacitySlider');
                const heatmapImage = document.getElementById('bigHeatmapImage');

                brightnessSlider.addEventListener('input', function() {
                    video.style.filter = `brightness(${this.value}%)`;
                });

                opacitySlider.addEventListener('input', function() {
                    heatmapImage.style.opacity = `${this.value / 100}`;
                });
            });
        });


        // Register the Video List component
        myLayout.registerComponent('Recordings', function(container, componentState) {
            <?php if (is_dir($videoDirectory)): ?>
            let htmlContent = "<div style='display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 10px;'>";
            <?php foreach ($videos as $video): ?>
                <?php $videoName = substr($video, 0, -4); ?>
                htmlContent += "<div id='videoListContainer2' style='height: 100%;' class='horizontal-scroll'>";
                htmlContent += "<a href='#' onclick='changeVideo(\"<?= 'videos/'. $date . '/allsky/' . $video ?>\")' title='<?= $videoName ?>'>";
                htmlContent += '<img class="heatmap-thumbnail" src="videos/<?= $date ?>/heatmaps/<?= $videoName ?>.jpg" height="100" title="Video: <?= $videoName ?>">'; // Title added here
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
                'fullscreen' // Toggle fullscreen
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
                    <p>(To calculate the recording's frame count)</p>
                    <p><button onclick="goBackOneFrame()">Back 1 Frame</button>
                    <button onclick="advanceOneFrame()">Forward 1 Frame</button></p>
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
                $logFilePath = 'logfile.txt'; // Specify the path to your log file

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
                        return 10; // Adjust the size of the symbol if needed
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
                            borderColor: '#000', // Optional: border color on hover
                            borderWidth: 1 // Optional: border width on hover
                        },
                        label: {
                            show: true // Optional: show label on hover
                        }
                    }
                }]
            };
            myChart.setOption(option);
            myChart.on('click', function (params) {
                if (params.componentType === 'series' && params.seriesType === 'scatter') {
                    let videoName = params.data.name; // Get the video file name
                    let newSource = '<?php echo $videoDirectory2; ?>' + videoName;
                    console.log(newSource);
                    changeVideo(newSource);
                }
            });
            let currentFrameNumber = getCurrentFrame();
            displayJSON(currentFrameNumber);

            player.addEventListener('timeupdate', function() {
                const currentFrameNumber = getCurrentFrame();
                console.log("Current Frame: ", currentFrameNumber);
                displayJSON(currentFrameNumber);
            });

            // const video = document.getElementById('player');
            // console.log(video);
            // brightnessSlider = document.getElementById('brightnessSlider');
            // console.log(brightnessSlider);
            // brightnessSlider.addEventListener('input', function() {
            //     video.style.filter = `brightness(${this.value}%)`;
            // });



        });

        function displayJSON(frameNumber) {
            console.log("displayJSON");
            console.log(frameNumber);
            console.log("frameNumber: ",frameNumber);
            console.log("document.getElementById('json'): ",document.getElementById("json"));
            console.log("jsonData: ",jsonData);
            console.log("jsonData[1]: ",jsonData[1]);
            console.log("jsonData[1].frames: ",jsonData[1].frames);
            console.log("jsonData[1].frames[frameNumber]: ",jsonData[1].frames[frameNumber]);
            console.log("jsonData[1].frames[frameNumber].detections: ",jsonData[1].frames[frameNumber].detections);
            var formattedJson = JSON.stringify(jsonData[1].frames[frameNumber].detections, undefined, 2);
            document.getElementById("json").innerHTML = `<pre>${formattedJson}</pre>`;
            // document.getElementById("json").innerHTML = JSON.stringify(jsonData[1].frames[frameNumber].detections, undefined, 2);
        }

        function goToFrame(frameNumber) {
            const newTime = frameNumber / frameRate;
            player.currentTime = newTime;
            displayJSON(frameNumber);
        }

        // function to use ajax to get the json file needed for currentVideoStartTimestamp
        function getcurrentVideoJsonData() {
            $.ajax({
                url: 'videos/'
                + date + '/json/' + currentVideoStartTimestamp + '.json',
                dataType: 'json',
                success: function(data) {
                    console.log(data);
                    jsonData = data;
                }
            });
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
            //  max width of the heatmap is 100% of the container
            $('#heatmap').css('max-width', '100%');
            // max height of the heatmap is 100% of the container
            $('#heatmap').css('max-height', '100%');
            var heatmapImg = document.getElementById('bigHeatmapImage');
            heatmapImg.src = newSrc;
            heatmapImg.style.opacity = '0.3'; // Set the opacity to 50%
        }

        function changeVideo(videoName) {
            // update the heatmap
            heatmapName = videoName.split(".")[0];
            heatmapName = heatmapName.split("/").pop();
            let heatmapPath = 'videos/<?php echo $date; ?>/heatmaps/' + heatmapName + '.jpg';
            updateHeatmapSrc(heatmapPath);
            // Remove highlight from all thumbnails
            // $('.thumbnail-container img').removeClass('highlighted');
            // Add highlight to the clicked thumbnail
            var heatmapName = videoName.split(".")[0].split("/").pop();
            $('img').removeClass('highlighted');
            var selector = ".heatmap-thumbnail[src*='" + heatmapName + "']";
            $(selector).addClass('highlighted');
            // Highlight the corresponding point
            unhighlightAllPoints();
            var specificTimestamp = heatmapName;
            currentVideoStartTimestamp = specificTimestamp;
            // highlightPointByTimestamp(specificTimestamp);
            highlightPointByTimestamp1(specificTimestamp);
            // update the video player
            let newSource = videoName;
            console.log(newSource);
            let videoElement = document.querySelector('#player source');
            videoElement.setAttribute('src', newSource);
            player.load();
            player.play();
            getcurrentVideoJsonData();
            displayJSON(getCurrentFrame());
        }

        function highlightPointByTimestamp(timestamp) {
            console.log("highlightPointByTimestamp");
            console.log(timestamp);
            console.log(myChart);
            var seriesIndex = 0;
            var dataIndex = -1;
            // Loop through the data to find the matching timestamp
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
            console.log("highlightPointByTimestamp1");
            console.log(timestamp);
            console.log(myChart);
            var seriesIndex = 0;
            var dataIndex = -1;
            var seriesData = myChart.getOption().series[seriesIndex].data;
            console.log(seriesData);
            for (var i = 0; i < seriesData.length; i++) {
                console.log(seriesData[i].value[0]);
                console.log(timestamp);
                if (seriesData[i].value[0] === (parseInt(timestamp+'000'))) {
                    dataIndex = i;
                    console.log("dataIndex");
                    console.log(dataIndex);
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

        function getDetectionsForFrame(frameNumber) {
            if (jsonData && jsonData[1] && jsonData[1].frames) {
                for (let frame of jsonData[1].frames) {
                    if (frame.time === frameNumber) {
                        return frame.detections;
                    }
                }
            }
            return null;
        }



</script>

<script>
        // Hide loading spinner overlay once the page content is fully loaded
        window.addEventListener('load', function () {
            document.querySelector('.spinner-overlay').style.display = 'none';
        });
    </script>
</body>
</html>