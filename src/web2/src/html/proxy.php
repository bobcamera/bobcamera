<?php
header("Access-Control-Allow-Origin: *");

// Collect the parameters from the incoming request
$lat = $_GET['lat'] ?? 38.62149068118758;
$lon = $_GET['lon'] ?? -90.42175434684431;
$dist = $_GET['dist'] ?? 250;

// Construct the URL for the adsb.lol API
$url = "https://api.adsb.lol/v2/lat/$lat/lon/$lon/dist/$dist";

// Use cURL to fetch the data
$ch = curl_init();
curl_setopt($ch, CURLOPT_URL, $url);
curl_setopt($ch, CURLOPT_RETURNTRANSFER, 1);
$response = curl_exec($ch);
curl_close($ch);

// Return the fetched data
echo $response;
?>
