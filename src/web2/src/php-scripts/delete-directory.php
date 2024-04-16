<?php

if (isset($_GET['date'])) {
    $dirName = $_GET['date'];
    $directoryPath = '../videos/' . basename($dirName);

    function deleteDirectory($dir) {
        if (!file_exists($dir)) {
            return true;
        }

        if (!is_dir($dir)) {
            return unlink($dir);
        }

        foreach (scandir($dir) as $item) {
            if ($item == '.' || $item == '..') {
                continue;
            }

            if (!deleteDirectory($dir . DIRECTORY_SEPARATOR . $item)) {
                return false;
            }
        }

        return rmdir($dir);
    }

    if (deleteDirectory($directoryPath)) {
        $message = "Directory [$dirName] and all its contents have been deleted.";
    } else {
        $message = "Failed to delete directory.";
    }

    header("Location: ../recordings.php?message=" . urlencode($message));
    exit();
}
?>