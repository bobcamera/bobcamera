<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Event Recordings By Date</title>
    <link rel="stylesheet" href="/assets/all.min.css">
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #ffffff;
            /* Set body background to white */
        }

        .styled-table {
            border-collapse: collapse;
            width: 100%;
            font-size: 0.9em;
            background-color: #ffffff;
            /* Set table background to white */
        }

        .styled-table thead tr {
            background-color: #ffffff;
            /* Set header background to white */
            color: #000000;
            /* Set header text to black */
        }

        .styled-table thead th {
            padding: 12px 15px;
            border-bottom: 2px solid #000000;
            /* Add black horizontal line */
        }

        .styled-table th,
        .styled-table td {
            text-align: center;
            /* Center-align text */
            padding: 12px 15px;
        }

        .styled-table tbody td {
            border: none;
            /* Remove borders inside table */
        }

        .icon {
            font-size: 1.6em;
            /* Increase size of icons */
        }

        .delete-btn {
            font-size: 0.9em;
            color: #ffffff;
            background-color: #e74c3c;
            border: none;
            padding: 10px 15px;
            border-radius: 5px;
            cursor: pointer;
            text-align: center;
            /* Center-align button text */
            display: inline-block;
            /* Align the button properly */
            vertical-align: middle;
            /* Align the button properly */
        }
    </style>
</head>
<body>
<?php

$videosDirectory = 'videos/';
$videoDirs = array_diff(scandir($videosDirectory), array('..', '.'));
// Table Header
echo "<h4>Event Recordings By Date | Navigate Event Recordings By Heatmap, Event Grid, or Event Table</h4>";
echo "<table class='styled-table'>";
echo "<thead>";
echo "<tr>";
echo "<th>Day</th>";
echo "<th>Event Grid</th>";
echo "<th>Event Player</th>";
echo "<th>Event Table</th>";
echo "<th>Action</th>";
echo "</tr>";
echo "</thead>";

// Loop through each Day's Recording Directory and display a row for each day
echo "<tbody>";
foreach ($videoDirs as $videoDir) {
    if (is_dir($videosDirectory . $videoDir)) {
        $formattedDate = DateTime::createFromFormat('Ymd', $videoDir)->format('Y-m-d');
        echo "<tr>";
        echo "<td>" . htmlspecialchars($formattedDate) . "</td>";
        echo "<td><a href='display-files-in-grid.php?date=" . urlencode($videoDir) . "'><i class='fas fa-images icon' style='color: #337AB7;'></i></a></td>";
        echo "<td><a href='heatmaps.php?date=" . urlencode($videoDir) . "'><i class='fas fa-play-circle icon' style='color: #337AB7;'></i></a></td>";
        echo "<td><a href='summary.php?date=" . urlencode($videoDir) . "'><i class='fas fa-table icon' style='color: #337AB7;'></i></a></td>";
        echo "<td><a href='php-scripts/delete-directory.php?date=" . urlencode($videoDir) . "' class='delete-btn' onclick=\"return confirm('Are you sure you want to delete this directory and all its contents?');\">
        <i class='fas fa-trash'></i> Delete</a></td>";
        echo "</tr>";
    }
}
echo "</tbody>";
echo "</table>";

if (isset($_GET['message'])) {
    $message = $_GET['message'];
    echo "<script type='text/javascript'>alert('$message');</script>";
}

?>
</body>
</html>
