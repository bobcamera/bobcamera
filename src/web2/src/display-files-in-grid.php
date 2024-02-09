<?php
    $date = $_GET['date'] ?? '20240112';
    $videoDirectory = 'videos/' . $date . '/allsky/';
    $videos = array_diff(scandir($videoDirectory), array('..', '.'));

    $time = $_GET['time'] ?? substr($videos[2], 0, -4);
    $heatMapDirectory = 'videos/' . $date . '/heatmaps';
    $heatMapImagePath = $heatMapDirectory . '/' . $date . '/' . $time . '.png';
    $heatMapPath = $heatMapDirectory . '/heatmap-timelapse-' . $date . '.mp4';
    $videoPath = $videoDirectory . $time . '.mp4';

    if (is_dir($videoDirectory)) {
        echo "<div style='display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 10px;'>";
        foreach ($videos as $video) {
            $videoName = substr($video, 0, -4);
            echo "<div id='videoListContainer2' style='height: 100%;' class='horizontal-scroll'>";
            echo "<a href='video.php?date=" . $date . "&time=" . $videoName . "'>";
            echo '<img src="videos/' . $date . '/heatmaps/' .  $videoName . '.jpg" height="120">';
            echo "</a>";
            echo "</div>";
        }
        echo "</div>";
    } else {
        echo "The directory does not exist.";
    }
?>
