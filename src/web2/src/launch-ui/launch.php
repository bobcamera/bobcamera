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
    // Load form data from local storage
    loadFormData();

    // Attach event listeners to form fields for saving data on change
    attachChangeListeners();
});

function attachChangeListeners() {
    document.querySelectorAll('form input, form select').forEach(input => {
        input.addEventListener('change', function() {
            // Check if the input is a checkbox and handle accordingly
            if (input.type === 'checkbox') {
                localStorage.setItem(input.name, input.checked ? 'True' : 'False');
            } else {
                localStorage.setItem(input.name, input.value);
            }
        });
    });
}

function loadFormData() {
    document.querySelectorAll('form input, form select').forEach(input => {
        if (input.type === 'checkbox') {
            input.checked = localStorage.getItem(input.name) === 'True';
        } else {
            const value = localStorage.getItem(input.name);
            if (value) {
                input.value = value;
            }
        }
    });
}
</script>


</head>
<body>
    <h2>RTSP Configuration Form</h2>
    <form action="" method="post">
        Source: <input type="text" name="BOB_SOURCE" value="video"><br>
        RTSP URL: <input type="text" name="BOB_RTSP_URL" value=""><br>
        RTSP Width: <input type="text" name="BOB_RTSP_WIDTH" value="2560"><br>
        RTSP Height: <input type="text" name="BOB_RTSP_HEIGHT" value="2560"><br>
        FPS: <input type="text" name="BOB_FPS" value="25.0"><br>
        Camera ID: <input type="text" name="BOB_CAMERA_ID" value="0"><br>
        Enable Visualiser: <input type="checkbox" name="BOB_ENABLE_VISUALISER" value="True"><br>
        Optimised: <input type="checkbox" name="BOB_OPTIMISED" value="True"><br>
        Enable Recording: <input type="checkbox" name="BOB_ENABLE_RECORDING" value="True"><br>
        RMW Implementation: <input type="text" name="RMW_IMPLEMENTATION" value="rmw_fastrtps_cpp"><br>
        FASTRTPS Default Profiles File: <input type="text" name="FASTRTPS_DEFAULT_PROFILES_FILE" value=""><br>
        BGS Algorithm: <input type="text" name="BOB_BGS_ALGORITHM" value="vibe"><br>
        Tracking Sensitivity: <input type="text" name="BOB_TRACKING_SENSITIVITY" value="medium"><br>
        Tracking UseMask: <input type="checkbox" name="BOB_TRACKING_USEMASK" value="True"><br>
        Tracking Mask File: <input type="text" name="BOB_TRACKING_MASK_FILE" value=""><br>
        Simulation Width: <input type="text" name="BOB_SIMULATION_WIDTH" value="1920"><br>
        Simulation Height: <input type="text" name="BOB_SIMULATION_HEIGHT" value="1080"><br>
        Simulation Num Objects: <input type="text" name="BOB_SIMULATION_NUM_OBJECTS" value="5"><br>
        <input type="submit" value="Submit">
    </form>
<?php
// Close the else block
}
?>
