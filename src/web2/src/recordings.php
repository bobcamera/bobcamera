<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Styled Table</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.1/css/all.min.css">
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

echo "<h4>Select Heatmap Navigation or Recording Grid from the date list below</h4>";
echo "<table class='styled-table'>";
echo "<thead>";
echo "<tr>";
echo "<th>Day</th>";
echo "<th>Images</th>";
//echo "<th>Timelapse</th>";
//echo "<th>Keogram</th>";
//echo "<th>Startrails</th>";
echo "<th>Heatmaps</th>";
echo "<th>Action</th>";
echo "</tr>";
echo "</thead>";
echo "<tbody>";

foreach ($videoDirs as $videoDir) {
    if (is_dir($videosDirectory . $videoDir)) {
        // Format the date from YYYYMMDD to a more readable format if needed
        $formattedDate = DateTime::createFromFormat('Ymd', $videoDir)->format('Y-m-d');

        echo "<tr>";
        echo "<td>" . htmlspecialchars($formattedDate) . "</td>";
        echo "<td><a href='display-files-in-grid.php?date=" . urlencode($videoDir) . "'><i class='fas fa-images icon' style='color: #337AB7;'></i></a></td>";
        //echo "<td><i class='fas fa-film icon' style='color: grey;'></i></td>";
        //echo "<td><i class='fas fa-chart-bar icon' style='color: grey;'></i></td>";
        //echo "<td><i class='fas fa-star icon' style='color: grey;'></i></td>";
        echo "<td><a href='heatmaps.php?date=" . urlencode($videoDir) . "'><i class='fas fa-fire icon' style='color: #337AB7;'></i></a></td>";
        echo "<td><button class='delete-btn'><i class='fas fa-trash'></i> Delete</button></td>";
        echo "</tr>";
    }
}

echo "</tbody>";
echo "</table>";
?>
</body>
</html>