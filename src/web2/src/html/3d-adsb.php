<!DOCTYPE html>
<html>

<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, user-scalable=no, minimum-scale=1.0, maximum-scale=1.0">
    <link type="text/css" rel="stylesheet" href="../lib/main.css">
    <script src="../lib/gsap.min.js"></script>
</head>
<body>
    <div id="container"></div>
    <div id="menu"></div>
    <script type="module">
        import * as THREE from '../lib/three.module.js';
        import { OrbitControls } from '../lib/OrbitControls2.js';
        var camera, scene, renderer;
        var controls;
        var objects = [];
        const radiusFromCenter = 30;
        const centerPointlat = 38.62149068118758;
        const centerPointlon = -90.42175434684431;
        const scalingParams = {
            horizontalScale: 1,
            verticalScale: 1,
            sphereRadius: 0.1,
            mapSizeInMiles: 15
        };
        init();
        animate();
        function init() {
            camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, .01, 10000);
            camera.position.z = -25;
            camera.position.y = 25;
            camera.lookAt(new THREE.Vector3(0, 0, 0));
            scene = new THREE.Scene();

            // Add X Y Z Axis Helpers to scene
                // const axesHelper = new THREE.AxesHelper(5);
                // scene.add(axesHelper);

            // Add a grid to the floor
                // const gridHelper = new THREE.GridHelper(scalingParams.mapSizeInMiles * scalingParams.horizontalScale * 4, 50);
                // scene.add(gridHelper);

            renderer = new THREE.WebGLRenderer();
            renderer.setSize(window.innerWidth, window.innerHeight);
            document.getElementById('container').appendChild(renderer.domElement);
            controls = new OrbitControls(camera, renderer.domElement);
            controls.minDistance = -100;
            controls.maxDistance = 10000;
            controls.addEventListener('change', render);

            const loader = new THREE.TextureLoader();
            loader.load(
                '../assets/map.png',
                function(texture) {
                    const planeWidth = scalingParams.mapSizeInMiles * scalingParams.horizontalScale;
                    const planeHeight = scalingParams.mapSizeInMiles * scalingParams.horizontalScale;
                    const planeGeometry = new THREE.PlaneGeometry(planeWidth, planeHeight, 10, 10);
                    const planeMaterial = new THREE.MeshBasicMaterial({ map: texture });
                    const plane = new THREE.Mesh(planeGeometry, planeMaterial);
                    plane.rotation.x = Math.PI / 2;
                    plane.rotation.y = Math.PI;
                    plane.position.y = .1;
                    // plane.rotation.z = Math.PI;
                    scene.add(plane);
                },
                undefined,
                function(err) {
                    console.error('An error occurred while loading the texture.');
                }
            );

            function altitudeToColor(altitude) {
                const minAlt = 1000;  // Minimum altitude in feet
                const maxAlt = 37000; // Maximum altitude in feet

                // Colors at key altitudes (from dark orange to violet)
                const minColor = new THREE.Color(0xFF8C00);
                const maxColor = new THREE.Color(0x8A2BE2);

                // Normalize the altitude
                const t = Math.min(Math.max((altitude - minAlt) / (maxAlt - minAlt), 0), 1);

                // Linearly interpolate between the min and max colors based on the normalized altitude
                const color = new THREE.Color();
                color.copy(minColor).lerpHSL(maxColor, t);

                return color.getHex();
            }

            console.log("modify fetch(`proxy.php?... based on the actual filepath This is around line 94 in the 3d-adsb.php file");
            fetch(`proxy.php?lat=${centerPointlat}&lon=${centerPointlon}&dist=${radiusFromCenter}`)
            .then(response => response.json())
            .then(data => {
                console.log("JSON Response:", data);  // Log the entire JSON response
                const aircrafts = data.ac;
                aircrafts.forEach((plane, index) => {
                    const geometry = new THREE.SphereGeometry(scalingParams.sphereRadius, 16, 16);
                    const color = altitudeToColor(plane.alt_baro);
                    const material = new THREE.MeshBasicMaterial({ color: color });  // Green
                    const sphere = new THREE.Mesh(geometry, material);
                    const position = geographicToCartesian(plane.lat, plane.lon, plane.alt_baro);
                    sphere.position.set(position.x, position.y, position.z);
                    scene.add(sphere);
                    var distance = haversineDistance(plane.lat, plane.lon, centerPointlat, centerPointlon)
                    console.log(`Lat: ${plane.lat} Lon: ${plane.lon} Alt: ${plane.alt_baro} Miles: ${distance}`);
                });
                camera.updateProjectionMatrix();
                render();
            })
            .catch(error => console.error('Error fetching data:', error));

            window.addEventListener('resize', onWindowResize, false);
        }

        function geographicToCartesian(lat, lon, alt) {
            console.log(`Plane: `);
            console.log(`Lat: ${lat} Lon: ${lon} Alt: ${alt}`);
            const lat_NauticalMiles = haversineDistance(lat, 0, centerPointlat, 0);
            const lon_NauticalMiles = haversineDistance(0, lon, 0, centerPointlon);
            console.log(`Lat Miles: ${lat_NauticalMiles} Lon Miles: ${lon_NauticalMiles}`);
            const altInMiles = alt * 0.000189394 * scalingParams.verticalScale;
            const lonInMiles = lon_NauticalMiles * 1.15078;
            const latInMiles = lat_NauticalMiles * 1.15078;
            console.log(`Lat Miles relative to Center: ${latInMiles} Lon Miles relative to Center: ${lonInMiles} Altitude (miles): ${altInMiles} Altitude (feet): ${alt}` );
            const scaledlonInMiles = lonInMiles * scalingParams.horizontalScale;
            const scaledlatInMiles = latInMiles * scalingParams.horizontalScale;
            console.log(`Scaled Lat Miles relative to Center: ${scaledlatInMiles} Scaled Lon Miles relative to Center: ${scaledlonInMiles} Alt: ${altInMiles}`);
            return {
                x: lonInMiles,
                y: altInMiles,
                z: latInMiles
            };
        }

        function onWindowResize() {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
            render();
        }

        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            render();
        }

        function render() {
            renderer.render(scene, camera);
        }

        function haversineDistance(lat1, lon1, lat2, lon2) {
            const R = 3958.8;  // Radius of Earth in miles
            const dLat = toRadians(lat2 - lat1);
            const dLon = toRadians(lon2 - lon1);
            const a =
                Math.sin(dLat/2) * Math.sin(dLat/2) +
                Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2)) *
                Math.sin(dLon/2) * Math.sin(dLon/2)
            ;
            const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            const distance = R * c;
            return distance;
        }

        function toRadians(angleInDegrees) {
            return angleInDegrees * (Math.PI / 180);
        }

    </script>
</body>
</html>
