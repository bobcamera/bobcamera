<?php
    $videosDirectory = 'videos/';
    $videoDirs = array_diff(scandir($videosDirectory), array('..', '.'));
    echo "<h4>Select Heatmap Navigation or Recording Grid from the date list below</h4>";
    echo "<ul>";
    foreach ($videoDirs as $videoDir) {
        if (is_dir('videos/' . $videoDir)) {
            echo "<li>";
            echo $videoDir;
            echo "&nbsp;";
            echo "|";
            echo "&nbsp;";
            echo "<a href='heatmaps.php?date=" . $videoDir . "#'>Heatmaps Navigation</a>";
            echo "&nbsp;";
            echo "|";
            echo "&nbsp;";
            echo "<a href='display-files-in-grid.php?date=" . $videoDir . "#'>Recording Grid</a>";
            echo "</li>";
        }
    }
    echo "</ul>";
    echo "<h4>Dan would you please so the styling to that it looks something like this:</h4>";
    echo '<img src="assets/tj-allsky-navigation.png" title="tj-allsky">';
?>