<?php
$file = $_GET['file'] ?? 'index.php';
$date = $_GET['date'] ?? '20240112';
$time = $_GET['time'] ?? '1705000021';
?>

<!doctype html>
<html lang="en" data-bs-theme="auto">

<head>
    <script src="lib/bootstrap/color-modes.js"></script>
    <meta charset="utf-8">
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate" />
    <meta http-equiv="Pragma" content="no-cache" />
    <meta http-equiv="Expires" content="0" />
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta name="description" content="">
    <title>BOB the Universal Object Tracker</title>
    <link rel="stylesheet" href="/assets/css@3.css">
    <link href="lib/bootstrap/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" href="/assets/leaflet.css" />
    <link rel="stylesheet" href="/assets/all.min.css"  crossorigin="anonymous">
    <script src="/assets/leaflet.js"></script>
    <script src="/assets/moment.min.js"></script>
    <script src="/assets/moment-timezone-with-data-10-year-range.min.js"></script>


    <style>
        .bd-placeholder-img {
            font-size: 1.125rem;
            text-anchor: middle;
            -webkit-user-select: none;
            -moz-user-select: none;
            user-select: none;
        }

        @media (min-width: 768px) {
            .bd-placeholder-img-lg {
                font-size: 3.5rem;
            }
        }

        .b-example-divider {
            width: 100%;
            height: 3rem;
            background-color: rgba(0, 0, 0, .1);
            border: solid rgba(0, 0, 0, .15);
            border-width: 1px 0;
            box-shadow: inset 0 .5em 1.5em rgba(0, 0, 0, .1), inset 0 .125em .5em rgba(0, 0, 0, .15);
        }

        .b-example-vr {
            flex-shrink: 0;
            width: 1.5rem;
            height: 100vh;
        }

        .bi {
            vertical-align: -.125em;
            fill: currentColor;
        }

        .nav-scroller {
            position: relative;
            z-index: 2;
            height: 2.75rem;
            overflow-y: hidden;
        }

        .nav-scroller .nav {
            display: flex;
            flex-wrap: nowrap;
            padding-bottom: 1rem;
            margin-top: -1px;
            overflow-x: auto;
            text-align: center;
            white-space: nowrap;
            -webkit-overflow-scrolling: touch;
        }

        .btn-bd-primary {
            --bd-violet-bg: #712cf9;
            --bd-violet-rgb: 112.520718, 44.062154, 249.437846;

            --bs-btn-font-weight: 600;
            --bs-btn-color: var(--bs-white);
            --bs-btn-bg: var(--bd-violet-bg);
            --bs-btn-border-color: var(--bd-violet-bg);
            --bs-btn-hover-color: var(--bs-white);
            --bs-btn-hover-bg: #6528e0;
            --bs-btn-hover-border-color: #6528e0;
            --bs-btn-focus-shadow-rgb: var(--bd-violet-rgb);
            --bs-btn-active-color: var(--bs-btn-hover-color);
            --bs-btn-active-bg: #5a23c8;
            --bs-btn-active-border-color: #5a23c8;
        }

        .bd-mode-toggle {
            z-index: 1500;
        }

        .bd-mode-toggle .dropdown-menu .active .bi {
            display: block !important;
        }
    </style>
    <!-- Custom styles for this template -->
    <link href="/assets/bootstrap-icons.min.css" rel="stylesheet">
    <!-- Custom styles for this template -->
    <link href="lib/bootstrap/dashboard.css" rel="stylesheet">
</head>

<body>
    <?php include_once 'php-components/header.php'; ?>
    <div class="container-fluid">
        <div class="row p-0">
            <div class="sidebar border border-right col-md-2 col-lg-2 p-0 bg-body-tertiary">
                <div class="offcanvas-md offcanvas-end bg-body-tertiary" tabindex="-1" id="sidebarMenu" aria-labelledby="sidebarMenuLabel">
                    <div class="offcanvas-header">
                        <h6 class="offcanvas-title" id="sidebarMenuLabel">BOB the Universal Object Tracker</h6>
                        <button type="button" class="btn-close" data-bs-dismiss="offcanvas" data-bs-target="#sidebarMenu" aria-label="Close"></button>
                    </div>
                    <div class="offcanvas-body d-md-flex flex-column p-0 pt-lg-3 overflow-y-auto">
                        <?php include_once 'php-components/sidebar.php'; ?>
                    </div>
                </div>
            </div>
            <main class="col-md-10 ms-sm-auto col-lg-10 p-0">
                <iframe src="<?php echo $file . '?date=' . $date . '&time=' . $time; ?>" frameborder="0" style="width: 100%; height: 100vh;" allowfullscreen></iframe>
            </main>
        </div>
    </div>
    <script src="lib/bootstrap/bootstrap.bundle.min.js"></script>
    <script src="/assets/chart.umd.js" integrity="sha384-eI7PSr3L1XLISH8JdDII5YN/njoSsxfbrkCTnJrzXt+ENP5MOVBxD+l6sEG4zoLp" crossorigin="anonymous"></script>
    <script src="lib/bootstrap/dashboard.js"></script>
</body>

</html>