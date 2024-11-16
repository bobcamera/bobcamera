<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Settings Page</title>
    <script src="/assets/moment.min.js"></script>
    <script src="/assets/moment-timezone-with-data-10-year-range.min.js"></script>
    <link rel="stylesheet" href="/assets/leaflet.css" />
    <script src="/assets/leaflet.js"></script>

</head>

<body>
    <h2>Settings</h2>
    <div id="settingsForm">

        <label for="stationName">Station Name:</label>
        <input type="text" id="stationName" name="stationName"><br><br>

        <label for="timezone">Timezone:</label>
        <select class="form-select" id="timezone" name="timezone">
            <option value="Pacific/Midway">Pacific/Midway (UTC-11:00)</option>
            <option value="America/Adak">America/Adak (UTC-10:00)</option>
            <option value="Pacific/Honolulu">Pacific/Honolulu (UTC-10:00)</option>
            <option value="America/Anchorage">America/Anchorage (UTC-09:00)</option>
            <option value="America/Los_Angeles">America/Los Angeles (UTC-08:00)</option>
            <option value="America/Denver">America/Denver (UTC-07:00)</option>
            <option value="America/Chicago">America/Chicago (UTC-06:00)</option>
            <option value="America/New_York">America/New York (UTC-05:00)</option>
            <option value="America/Caracas">America/Caracas (UTC-04:00)</option>
            <option value="America/Halifax">America/Halifax (UTC-04:00)</option>
            <option value="Atlantic/Azores">Atlantic/Azores (UTC-01:00)</option>
            <option value="Europe/London">Europe/London (UTC+00:00)</option>
            <option value="Europe/Berlin">Europe/Berlin (UTC+01:00)</option>
            <option value="Europe/Athens">Europe/Athens (UTC+02:00)</option>
            <option value="Europe/Moscow">Europe/Moscow (UTC+03:00)</option>
            <option value="Asia/Tehran">Asia/Tehran (UTC+03:30)</option>
            <option value="Asia/Dubai">Asia/Dubai (UTC+04:00)</option>
            <option value="Asia/Kabul">Asia/Kabul (UTC+04:30)</option>
            <option value="Asia/Karachi">Asia/Karachi (UTC+05:00)</option>
            <option value="Asia/Kolkata">Asia/Kolkata (UTC+05:30)</option>
            <option value="Asia/Kathmandu">Asia/Kathmandu (UTC+05:45)</option>
            <option value="Asia/Dhaka">Asia/Dhaka (UTC+06:00)</option>
            <option value="Asia/Yangon">Asia/Yangon (UTC+06:30)</option>
            <option value="Asia/Bangkok">Asia/Bangkok (UTC+07:00)</option>
            <option value="Asia/Shanghai">Asia/Shanghai (UTC+08:00)</option>
            <option value="Asia/Tokyo">Asia/Tokyo (UTC+09:00)</option>
            <option value="Australia/Adelaide">Australia/Adelaide (UTC+09:30)</option>
            <option value="Pacific/Guam">Pacific/Guam (UTC+10:00)</option>
            <option value="Australia/Sydney">Australia/Sydney (UTC+10:00)</option>
            <option value="Pacific/Noumea">Pacific/Noumea (UTC+11:00)</option>
            <option value="Pacific/Auckland">Pacific/Auckland (UTC+12:00)</option>
            <option value="Pacific/Fiji">Pacific/Fiji (UTC+12:00)</option>
            <option value="Pacific/Tongatapu">Pacific/Tongatapu (UTC+13:00)</option>
        </select><br><br>
        <label for="dst">Daylight Savings:</label>
        <input type="checkbox" id="dst" name="dst"><br><br>
        <label for="timeFormat24hr">24-Hour Time Format:</label>
        <input type="checkbox" id="timeFormat24hr" name="timeFormat24hr"><br><br>

        <label for="dateFormat">Date Format:</label>
        <select id="dateFormat" name="dateFormat">
            <option value="YYYY-MM-DD">YYYY-MM-DD (ISO, e.g., 2023-03-28)</option>
            <option value="MM/DD/YYYY">MM/DD/YYYY (e.g., 03/28/2023)</option>
            <option value="DD/MM/YYYY">DD/MM/YYYY (e.g., 28/03/2023)</option>
        </select><br><br>

        Current Local Time: <span id="currentTime"></span><br><br>
        <!-- dsiplay UTC Time -->
        UTC Time: <span id="utcTime"></span><br><br>

        <label for="latitude">Latitude:</label>
        <input type="text" id="latitude" name="latitude">

        <label for="longitude">Longitude:</label>
        <input type="text" id="longitude" name="longitude"><br><br>
        <div id="stationMapSettings" style="width: 50%; height: 250px;"></div>
        <br>
        <button onclick="saveSettings()">Save Settings</button>
        <br><br>
        <button onclick="importSettings()">Import Settings</button>
        <input type="file" id="importFile" hidden onchange="loadFile(event)">
        <button onclick="exportSettings()">Export Settings</button>
    </div>
    <script>
        // Default coordinates for UTC center
        const defaultCoords = [51.4769, 0];

        // Check if Leaflet is loaded before initializing the map and settings
        if (typeof L !== 'undefined') {
            document.addEventListener('DOMContentLoaded', function() {
                loadSettings();
            });
        }

        // display UTC time
        setInterval(() => {
            document.getElementById('utcTime').textContent = moment.utc().format('HH:mm:ss YYYY-MM-DD');
        }, 1000);

        function saveSettings() {
            const settings = {
                stationName: document.getElementById('stationName').value,
                latitude: document.getElementById('latitude').value,
                longitude: document.getElementById('longitude').value,
                timezone: document.getElementById('timezone').value,
                dst: document.getElementById('dst').checked,
                timeFormat24hr: document.getElementById('timeFormat24hr').checked,
                dateFormat: document.getElementById('dateFormat').value
            };
            localStorage.setItem('settings', JSON.stringify(settings));
            alert('Settings saved!');
            updateMapCenter(settings.latitude, settings.longitude);
            updateTimeZone(settings.timezone);
        }

        function loadSettings() {
            const settings = JSON.parse(localStorage.getItem('settings'));
            if (settings) {
                document.getElementById('stationName').value = settings.stationName;
                document.getElementById('latitude').value = settings.latitude;
                document.getElementById('longitude').value = settings.longitude;
                document.getElementById('timezone').value = settings.timezone;
                document.getElementById('dst').checked = settings.dst;
                document.getElementById('timeFormat24hr').checked = settings.timeFormat24hr;
                document.getElementById('dateFormat').value = settings.dateFormat;
                updateMapCenter(settings.latitude, settings.longitude);
                updateTimeZone(settings.timezone);
            }
        }

        function exportSettings() {
            const settings = localStorage.getItem('settings');
            const blob = new Blob([settings], {
                type: 'application/json'
            });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'settings.json';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
        }

        function importSettings() {
            document.getElementById('importFile').click();
        }

        function loadFile(event) {
            const file = event.target.files[0];
            if (file) {
                const reader = new FileReader();
                reader.onload = function(e) {
                    const settings = JSON.parse(e.target.result);
                    localStorage.setItem('settings', JSON.stringify(settings));
                    loadSettings();
                    alert('Settings imported!');
                };
                reader.readAsText(file);
            }
        }

        function updateMapCenter(lat = defaultCoords[0], lng = defaultCoords[1]) {
            const stationCoords = [lat, lng];
            const mapElement = document.getElementById('stationMapSettings');
            if (!mapElement._leaflet_id) {
                window.stationMapSettings = L.map('stationMapSettings').setView(stationCoords, 13);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {}).addTo(window.stationMapSettings);
                window.stationMarker = L.marker(stationCoords).addTo(window.stationMapSettings);
            } else {
                window.stationMapSettings.setView(stationCoords, 13);
                if (window.stationMarker) {
                    window.stationMapSettings.removeLayer(window.stationMarker);
                }
                window.stationMarker = L.marker(stationCoords).addTo(window.stationMapSettings);
            }
        }

        function updateTimeZone(timezone) {
            let timeFormat = document.getElementById('timeFormat24hr').checked ? 'HH:mm:ss' : 'h:mm:ss A';
            let dateFormat = document.getElementById('dateFormat').value;
            let format = `${timeFormat} ${dateFormat}`;
            document.getElementById('currentTime').textContent = moment.tz(timezone).format(format);
            if (window.timeInterval) {
                clearInterval(window.timeInterval);
            }
            window.timeInterval = setInterval(() => {
                document.getElementById('currentTime').textContent = moment.tz(timezone).format(format);
            }, 1000);
        }

        document.getElementById('timezone').addEventListener('change', function() {
            updateTimeZone(this.value);
        });

        document.getElementById('timeFormat24hr').addEventListener('change', function() {
            updateTimeZone(document.getElementById('timezone').value);
        });

        document.getElementById('dst').addEventListener('change', function() {
            updateTimeZone(document.getElementById('timezone').value);
        });

        document.getElementById('dateFormat').addEventListener('change', function() {
            updateTimeZone(document.getElementById('timezone').value);
        });

    </script>

</body>

</html>