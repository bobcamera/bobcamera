<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Video Display</title>
    <link rel="stylesheet" href="https://cdn.plyr.io/3.6.2/plyr.css" />

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

        /* #brightness-control {
            position: absolute;
            bottom: 135px;
            right: 41px;
            -webkit-appearance: none;
            width: 150px;
            height: 5px;
            transform: rotate(-90deg);
            transform-origin: right bottom;
            display: none;
        }

        .fab {
            position: absolute;
            bottom: -69px;
            right: 20px;
            width: 50px;
            height: 50px;
            border-radius: 50%;
            background-color: #007bff;
            color: white;
            text-align: center;
            line-height: 50px;
            font-size: 24px;
            cursor: pointer;
            z-index: 10;
        } */
    </style>
</head>
<body>
    <?php
        $date = $_GET['date'] ?? '20240112';
        $time = $_GET['time'] ?? 'defaultTime';
    ?>

    <div class="video-container">
        <!-- <div class="fab" onclick="toggleSlider()">☀</div>  -->
        <video id="player" playsinline controls autoplay muted>
            <source src="<?php echo 'videos/' . $date . '/allsky/' . $time . '.mp4'; ?>" />
        </video>
        <!-- <input type="range" id="brightness-control" min="0" max="200" value="100"> -->
    </div>

    <!-- Plyr.js Library -->
    <script src="https://cdn.plyr.io/3.7.2/plyr.polyfilled.js"></script>
    <script>
        document.addEventListener('DOMContentLoaded', () => {
            const player = new Plyr('#player');
            const video = document.getElementById('player');
            // const brightnessControl = document.getElementById('brightness-control');
            // brightnessControl.addEventListener('input', function() {
            //     video.style.filter = `brightness(${this.value}%)`;
            // });
        });

        // function toggleSlider() {
        //     const slider = document.getElementById('brightness-control');
        //     slider.style.display = slider.style.display === 'none' ? 'block' : 'none';
        // }
    </script>
</body>
</html>
