<?php
// Check if the form has been submitted
if ($_SERVER["REQUEST_METHOD"] == "POST") {
    // Process form data and execute command
    // Collect and sanitize input. Use escapeshellarg() for safety.
    $bob_source = escapeshellarg($_POST["BOB_SOURCE"]);
    $bob_rtsp_url = escapeshellarg($_POST["BOB_RTSP_URL"]);
    $bob_rtsp_width = escapeshellarg($_POST["BOB_RTSP_WIDTH"]);
    $bob_rtsp_height = escapeshellarg($_POST["BOB_RTSP_HEIGHT"]);
    $bob_fps = escapeshellarg($_POST["BOB_FPS"]);
    $bob_camera_id = escapeshellarg($_POST["BOB_CAMERA_ID"]);
    // For checkboxes, check if they are set and then use the value or default to "False"
    $bob_enable_visualiser = isset($_POST["BOB_ENABLE_VISUALISER"]) ? "True" : "False";
    $bob_optimised = isset($_POST["BOB_OPTIMISED"]) ? "True" : "False";
    $bob_enable_recording = isset($_POST["BOB_ENABLE_RECORDING"]) ? "True" : "False";
    // Continue for all other fields...
    // Construct the command string with environment variables
    $command = "export BOB_SOURCE=$bob_source; export BOB_RTSP_URL=$bob_rtsp_url; export BOB_RTSP_WIDTH=$bob_rtsp_width; export BOB_RTSP_HEIGHT=$bob_rtsp_height; export BOB_FPS=$bob_fps; export BOB_CAMERA_ID=$bob_camera_id; export BOB_ENABLE_VISUALISER='$bob_enable_visualiser'; export BOB_OPTIMISED='$bob_optimised'; export BOB_ENABLE_RECORDING='$bob_enable_recording'; ./launcher.sh application_launch.py";
    // Add all other exported variables to this string
    // Execute the command
    $output = shell_exec($command);
    // Display the output
    echo "<pre>$output</pre>";
} else {
    // Display the form if the request method is not POST
?>
<!DOCTYPE html>
<html>
<head>
    <title>Launch Configuration</title>
    <script>

        document.addEventListener('DOMContentLoaded', function() {
            loadConfigList(); // Load the saved configurations list
            document.getElementById('deleteConfigBtn').addEventListener('click', deleteSelectedConfig);
        });

        function saveConfig() {
            const configName = document.getElementById('configName').value.trim();
            if (!configName) {
                alert('Please enter a configuration name.');
                return;
            }

            let configs = JSON.parse(localStorage.getItem('configs')) || [];
            let formData = {};
            document.querySelectorAll('form input, form select').forEach(input => {
                formData[input.name] = input.type === 'checkbox' ? input.checked : input.value;
            });

            const existingConfigIndex = configs.findIndex(config => config.name === configName);
            if (existingConfigIndex > -1) {
                if (!confirm('This configuration name already exists. Update it?')) {
                    return;
                }
                configs[existingConfigIndex].data = formData;
            } else {
                configs.push({ name: configName, data: formData });
            }

            localStorage.setItem('configs', JSON.stringify(configs));
            loadConfigList();
            alert('Configuration saved!');
        }

        function deleteSelectedConfig() {
            const selectedConfigName = document.getElementById('configList').value;
            if (!selectedConfigName) {
                alert('Please select a configuration to delete.');
                return;
            }

            if (!confirm('Are you sure you want to delete this configuration?')) {
                return;
            }

            let configs = JSON.parse(localStorage.getItem('configs')) || [];
            configs = configs.filter(config => config.name !== selectedConfigName);
            localStorage.setItem('configs', JSON.stringify(configs));
            loadConfigList();
            document.querySelector('form').reset();
            alert('Configuration deleted.');
        }

        function loadConfigList() {
            let configs = JSON.parse(localStorage.getItem('configs')) || [];
            const configList = document.getElementById('configList');
            configList.innerHTML = '<option value="">Select a saved launch configuration...</option>';
            configs.forEach(config => {
                let option = new Option(config.name, config.name);
                configList.appendChild(option);
            });
        }

        function loadSelectedConfig() {
            const selectedConfigName = document.getElementById('configList').value;
            document.getElementById('configName').value = selectedConfigName;

            let configs = JSON.parse(localStorage.getItem('configs')) || [];
            const selectedConfig = configs.find(config => config.name === selectedConfigName);
            if (selectedConfig) {
                Object.entries(selectedConfig.data).forEach(([key, value]) => {
                    let input = document.querySelector(`[name="${key}"]`);
                    if (input) {
                        if (input.type === 'checkbox') {
                            input.checked = value === 'true' || value === true;
                        } else {
                            input.value = value;
                        }
                    }
                });
            }
        }

        function fetchRTSPInfo() {
            var rtspUrl = document.querySelector('input[name="BOB_RTSP_URL"]').value;
            if (!rtspUrl) {
                alert('Please enter an RTSP URL.');
                return;
            }
            fetch('get-rtsp-params-detailed.php?rtspUrl=' + encodeURIComponent(rtspUrl))
            .then(response => {
                if (!response.ok) {
                    throw new Error('Network response was not ok');
                }
                return response.json();
            })
            .then(data => {
                if (data.error) {
                    alert(data.error);
                } else {
                    document.querySelector('input[name="BOB_RTSP_WIDTH"]').value = data.width;
                    document.querySelector('input[name="BOB_RTSP_HEIGHT"]').value = data.height;
                    // Assuming FPS is a simple number here; adjust if your data is different
                    document.querySelector('input[name="BOB_FPS"]').value = data.frameRate.toFixed(2);
                }
            })
            .catch((error) => {
                console.error('Error:', error);
                alert('Failed to fetch RTSP info. Please check the console for more details.');
            });
        }

    </script>
</head>
<body>
    <h2>Launch Configuration</h2>
    <!-- <form action="" method="post">

        <select id="configList" onchange="loadSelectedConfig()">
            <option value="">Select a launch configuration...</option>
        </select>
        <br>
        or create a new one below:
        <br><br>
        Launch Configuration Name: <input type="text" id="configName" value="">
        <br>
        Image Source Type:
        <select name="BOB_SOURCE">
            <option value="rtsp" selected>RTSP Stream</option>
            <option value="usb">USB Camera</option>
            <option value="video">Video</option>
            <option value="simulate">Simulate</option>
            <option value="rtsp_overlay">RTSP Overlay</option>
            <option value="video_overlay">Video Overlay</option>
        </select>
        <br>
        RTSP URL: <input type="text" name="BOB_RTSP_URL" value="" style="width: 350px;">
<button type="button" onclick="fetchRTSPInfo()">Fetch Details</button>
<br>

        RTSP Width: <input type="text" name="BOB_RTSP_WIDTH" value="2560"><br>
        RTSP Height: <input type="text" name="BOB_RTSP_HEIGHT" value="2560"><br>
        FPS: <input type="text" name="BOB_FPS" value="25.0"><br>
        Camera ID: <input type="text" name="BOB_CAMERA_ID" value="0"><br>
        Enable Visualiser: <input type="checkbox" name="BOB_ENABLE_VISUALISER" value="False"><br>
        Optimised: <input type="checkbox" name="BOB_OPTIMISED" value="False"><br>
        Enable Recording: <input type="checkbox" name="BOB_ENABLE_RECORDING" value="False"><br>
        RMW Implementation: <input type="text" name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"><br>
        FASTRTPS Default Profiles File: <input type="text" name="FASTRTPS_DEFAULT_PROFILES_FILE" value="/workspaces/bobcamera/src/ros2/config/fastdds.xml"><br>
        BGS Algorithm:
        <select name="BOB_BGS_ALGORITHM">
            <option value="vibe" selected>VIBE</option>
            <option value="wmv">WMV</option>
        </select>
        <br>
        Tracking Sensitivity:
        <select name="BOB_TRACKING_SENSITIVITY">
            <option value="minimal">Minimal</option>
            <option value="low">Low</option>
            <option value="medium" selected>Medium</option>
            <option value="high">High</option>
        </select>
        <br>
        Tracking UseMask: <input type="checkbox" name="BOB_TRACKING_USEMASK" value="False"><br>
        Tracking Mask File: <input type="text" name="BOB_TRACKING_MASK_FILE" value="assets/masks/mask.jpg"><br>
        Simulation Width: <input type="text" name="BOB_SIMULATION_WIDTH" value="1920"><br>
        Simulation Height: <input type="text" name="BOB_SIMULATION_HEIGHT" value="1080"><br>
        Simulation Num Objects: <input type="text" name="BOB_SIMULATION_NUM_OBJECTS" value="5"><br>
        <br>
        <input type="submit" value="Launch">
        <button type="button" onclick="saveConfig()">Save Config</button>
        <button type="button" id="deleteConfigBtn">Delete Config</button>
    </form> -->
    <form action="" method="post">
        <!-- Configuration Management -->
        <fieldset>
            <legend>Configuration Management</legend>
            Load a saved configuration:
            <select id="configList" onchange="loadSelectedConfig()">
                <option value="">Select a launch configuration...</option>
            </select>
            <br>or create a new one below:
            <br><br>
            Launch Configuration Name: <input type="text" id="configName" value=""><br>
            <button type="button" onclick="saveConfig()">Save Config</button>
            <button type="button" id="deleteConfigBtn">Delete Config</button>
        </fieldset><br>

        <!-- Image Source Configuration -->
        <fieldset>
            <legend>Image Source Configuration</legend>
            Image Source Type:
            <select name="BOB_SOURCE">
                <option value="rtsp" selected>RTSP Stream</option>
                <option value="usb">USB Camera</option>
                <option value="video">Video</option>
                <option value="simulate">Simulate</option>
                <option value="rtsp_overlay">RTSP Overlay</option>
                <option value="video_overlay">Video Overlay</option>
            </select>
            <br>
            RTSP URL: <input type="text" name="BOB_RTSP_URL" value="rtsp://[username]:[password]@[ipaddress]:[port]/" style="width: 350px;">
            <button type="button" onclick="fetchRTSPInfo()">Fetch Details</button>
            <br>
            RTSP Width: <input type="text" name="BOB_RTSP_WIDTH" value="2560"><br>
            RTSP Height: <input type="text" name="BOB_RTSP_HEIGHT" value="2560"><br>
            Frames Per Second (FPS): <input type="text" name="BOB_FPS" value="25.0"><br>
            Camera ID: <input type="text" name="BOB_CAMERA_ID" value="0"><br>
        </fieldset><br>
        <fieldset>
            <legend>Additional Configuration</legend>
            Enable Visualiser: <input type="checkbox" name="BOB_ENABLE_VISUALISER" value="False"><br>
            Optimised: <input type="checkbox" name="BOB_OPTIMISED" value="False"><br>
            Enable Recording: <input type="checkbox" name="BOB_ENABLE_RECORDING" value="False"><br>
            RMW Implementation: <input type="text" name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"><br>
            FAST RTPS Default Profiles File: <input type="text" name="FASTRTPS_DEFAULT_PROFILES_FILE" value="/workspaces/bobcamera/src/ros2/config/fastdds.xml"><br>
            BGS Algorithm:
            <select name="BOB_BGS_ALGORITHM">
                <option value="vibe" selected>VIBE</option>
                <option value="wmv">WMV</option>
            </select>
        </fieldset><br>
        <fieldset>
            <legend>Tracking</legend>
            Tracking Sensitivity:
            <select name="BOB_TRACKING_SENSITIVITY">
                <option value="minimal">Minimal</option>
                <option value="low">Low</option>
                <option value="medium" selected>Medium</option>
                <option value="high">High</option>
            </select>
            <br>
            Use Image Mask: <input type="checkbox" name="BOB_TRACKING_USEMASK" value="False"><br>
            Mask File Path: <input type="text" name="BOB_TRACKING_MASK_FILE" value="assets/masks/mask.jpg"><br>
        </fieldset><br>
        <fieldset>
            <legend>Simulation</legend>
            Simulation Width: <input type="text" name="BOB_SIMULATION_WIDTH" value="1920"><br>
            Simulation Height: <input type="text" name="BOB_SIMULATION_HEIGHT" value="1080"><br>
            Simulation Num Objects: <input type="text" name="BOB_SIMULATION_NUM_OBJECTS" value="5"><br>
        </fieldset>

        <br>
        <input type="submit" value="Launch">
    </form>

<?php } ?>