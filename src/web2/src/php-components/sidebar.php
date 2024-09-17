<select id="cameraCountSelect">
    <option value="1">1 Camera</option>
    <option value="2">2 Cameras</option>
    <option value="3">3 Cameras</option>
    <option value="4">4 Cameras</option>
    <option value="5">5 Cameras</option>
</select>
<br>
<div id="bobSidebarMenu" ></div>
<br>
<div id="stationInfo" class="mt-3 p-3 border rounded bg-body-tertiary text-body-secondary">
    <p id="stationName">Unknown</p>
    <p id="stationTime">Loading...</p>
    <!-- <div id="currentTime">Loading time...</div> -->

    <p id="stationTimezone">Loading...</p>
    <p id="stationLatLong">Loading...</p>
    <select id="languageSelect">
        <option value="English">English</option>
        <option value="NorwegianBokmal">Norsk Bokmål</option>
        <option value="NorwegianNynorsk">Norsk Nynorsk</option>
        <option value="Spanish">Español</option>
        <option value="German">Deutsch</option>
        <option value="Italian">Italiano</option>
        <option value="Serbian">Serbian</option>
        <option value="French">Français</option>
        <option value="Portuguese">Português</option>
        <option value="Bengali">বাংলা</option>
        <option value="Indonesian">Bahasa Indonesia</option>
        <option value="Japanese">日本語</option>
        <option value="Korean">한국어</option>
        <option value="Swahili">Kiswahili</option>
        <option value="Turkish">Türkçe</option>
        <option value="Vietnamese">Tiếng Việt</option>
        <option value="Telugu">తెలుగు</option>
        <option value="Marathi">मराठी</option>
        <option value="Urdu">اردو</option>
        <option value="Hindi">हिन्दी</option>
    </select>
</div>
<div id="stationMap" style="width: 100%; height: 100px;"></div>

<script>
    // let preferredLanguage; // Example - ensure this is correctly set
    document.addEventListener('DOMContentLoaded', () => {
        // loadStationSettings();
        loadStationSettingsFromLocalStorage();
        // Listen for changes directly on DST and 24-hour format checkboxes or relevant settings
        if (document.getElementById('dst')) {
            document.getElementById('dst').addEventListener('change', updateSettingsBasedOnUserInput);
        }
        // Assuming there's a checkbox or switch for 24-hour time format preference
        if (document.getElementById('use24hr')) {
            document.getElementById('use24hr').addEventListener('change', updateSettingsBasedOnUserInput);
        }
        // Preferred Launguage
        const savedLanguage = localStorage.getItem('preferredLanguage');
        const languageSelect = document.getElementById('languageSelect');
        if (savedLanguage) {
            preferredLanguage = savedLanguage;
            // Set the select dropdown to match the saved language
            languageSelect.value = preferredLanguage;
        } else {
            // Default language if none is saved
            preferredLanguage = 'English';
        }        
        // Event listener for language selection
        languageSelect.addEventListener('change', function() {
                changeLanguage(this.value);
            });
        // Apply custom styles
        languageSelect.style.border = '1px solid #495057'; // Darker border color
        languageSelect.style.borderRadius = '3px'; // Rounded corners
        languageSelect.style.padding = '0'; // Padding
        languageSelect.style.marginLeft = '0px'; // Margin
        languageSelect.style.marginBottom = '0px'; // Margin
        languageSelect.style.fontSize = '14px'; // Font size
        languageSelect.style.lineHeight = '1'; // Line height
        languageSelect.style.color = '#ffffff'; // Lighter text color for contrast
        languageSelect.style.backgroundColor = '#343a40'; // Darker background color
        languageSelect.style.display = 'block'; // Ensure it's block level for full width
        languageSelect.style.width = '60%'; // Narrower width
        languageSelect.style.height = '25px'; // Adjusted height
        languageSelect.style.transition = 'border-color .15s ease-in-out,box-shadow .15s ease-in-out'; // Smooth transition for focus
        // Adjusted focus styles for reversed coloration
        languageSelect.addEventListener('focus', function() {
            this.style.borderColor = '#adb5bd';
            this.style.outline = '0';
            this.style.boxShadow = '0 0 0 0.2rem rgba(108,117,125,.25)';
        });
        languageSelect.addEventListener('blur', function() {
            this.style.borderColor = '#495057';
            this.style.boxShadow = 'none';
        });

        // Camera Count
        const savedCameraCount = localStorage.getItem('cameraCount');
        const cameraCountSelect = document.getElementById('cameraCountSelect');
        if (savedCameraCount) {
            cameraCount = savedCameraCount;
            // Set the select dropdown to match the saved Camera Count
            cameraCountSelect.value = cameraCount;
        } else {
            // Default language if none is saved
            cameraCount = 1;
        }
        // Event listener for language selection
        cameraCountSelect.addEventListener('change', function() {
            changeCameraCount(this.value);
            });
        // Apply custom styles
        cameraCountSelect.style.border = '1px solid #495057'; // Darker border color
        cameraCountSelect.style.borderRadius = '3px'; // Rounded corners
        cameraCountSelect.style.padding = '0'; // Padding
        cameraCountSelect.style.marginLeft = '0px'; // Margin
        cameraCountSelect.style.marginBottom = '0px'; // Margin
        cameraCountSelect.style.fontSize = '14px'; // Font size
        cameraCountSelect.style.lineHeight = '1'; // Line height
        cameraCountSelect.style.color = '#ffffff'; // Lighter text color for contrast
        cameraCountSelect.style.backgroundColor = '#343a40'; // Darker background color
        cameraCountSelect.style.display = 'block'; // Ensure it's block level for full width
        cameraCountSelect.style.width = '60%'; // Narrower width
        cameraCountSelect.style.height = '25px'; // Adjusted height
        cameraCountSelect.style.transition = 'border-color .15s ease-in-out,box-shadow .15s ease-in-out'; // Smooth transition for focus
        // Adjusted focus styles for reversed coloration
        cameraCountSelect.addEventListener('focus', function() {
            this.style.borderColor = '#adb5bd';
            this.style.outline = '0';
            this.style.boxShadow = '0 0 0 0.2rem rgba(108,117,125,.25)';
        });
        cameraCountSelect.addEventListener('blur', function() {
            this.style.borderColor = '#495057';
            this.style.boxShadow = 'none';
        });

        loadMenuAndTranslations(preferredLanguage, cameraCount);

        // Example function to update station information
        // updateStationInfo("Example", "America/Chicago", "41.8781", "87.6298");
        // Update time every minute
        setInterval(() => {
            updateStationTime("America/Chicago");
        }, 60000);
        // Listener for changes in settings that would affect the time display
        window.addEventListener('storage', (event) => {
            if (event.key === 'settings') {
                const newSettings = JSON.parse(event.newValue);
                updateStationTime(newSettings.timezone, newSettings.dst);
            }
        });
        // Update the time immediately and then every second
        updateCurrentDateTime();
        setInterval(updateCurrentDateTime, 1000);
    });

    function loadStationSettingsFromLocalStorage() {
        const settings = JSON.parse(localStorage.getItem('settings'));
        if (settings) {
            updateStationInfo(settings.stationName, settings.timezone, settings.latitude, settings.longitude);
            updateStationTime(settings.timezone, settings.dst);
            updateMapCenter(settings.latitude, settings.longitude);
        } else {
            console.log("No settings found in LocalStorage.");
        }
    }

    function updateStationInfo(name, timezone, lat, long) {
        const stationInfo = document.getElementById('stationInfo');
        // Applying dark theme styles
        stationInfo.style.backgroundColor = '#343a40'; // Dark background
        stationInfo.style.color = '#ffffff'; // Light text
        stationInfo.style.border = '1px solid #495057'; // Darker border for contrast
        // Applying text styles
        stationInfo.style.fontFamily = 'system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", "Noto Sans", "Liberation Sans", Arial, sans-serif, "Apple Color Emoji", "Segoe UI Emoji", "Segoe UI Symbol", "Noto Color Emoji"';
        stationInfo.style.fontSize = '12px';
        stationInfo.style.fontWeight = '500';
        stationInfo.style.lineHeight = '10px';
        document.getElementById('stationName').textContent = name || 'Unknown Station';
        document.getElementById('stationTimezone').textContent = 'Timezone: ' + timezone;
        document.getElementById('stationLatLong').textContent = 'Coordinates: ' + lat + ', ' + long;
    }

    // Adjusted `updateStationTime` function to dynamically respond to DST and 24-hour format changes
    function updateStationTime(timezone, useDST) {
        const now = new Date();
        const formatOptions = {
            timeZone: timezone,
            year: 'numeric', month: '2-digit', day: '2-digit',
            hour: '2-digit', minute: '2-digit', second: '2-digit',
            hour12: !(localStorage.getItem('use24hr') === 'true') // Adjust based on user preference
        };
        const formattedTime = new Intl.DateTimeFormat('en-US', formatOptions).format(now);
        document.getElementById('stationTime').textContent = formattedTime;
    }

    function fetchJson(url) {
        return fetch(url).then(response => response.json());
    }

    function applyTranslations(menuData, translationsData, lang) {
        console.log("Translations Data:", translationsData); // Debugging
        const translatedMenu = menuData.map(section => {
            console.log("Translating section:", section.key, "to", lang); // Debugging
            const sectionTrans = translationsData[section.key] || {};
            const title = sectionTrans[lang] || section.title;
            console.log("Translated title:", title); // Debugging
            const items = section.items.map(item => {
                const itemTrans = translationsData[item.key] || {};
                const name = itemTrans[lang] || item.name;
                return {
                    ...item,
                    name
                };
            });
            return {
                ...section,
                title,
                items
            };
        });
        return translatedMenu;
    }

    function buildMenu(data) {
        const sidebar = document.getElementById('bobSidebarMenu');
        sidebar.innerHTML = ''; // Clear previous content
        data.forEach(section => {
            const sectionTitle = document.createElement('h6');
            sectionTitle.className = 'sidebar-heading d-flex justify-content-between align-items-center px-3 mt-1 mb-1 text-body-secondary text-uppercase';
            sectionTitle.innerHTML = `<span>${section.title}</span>`;
            sidebar.appendChild(sectionTitle);
            const list = document.createElement('ul');
            list.className = 'nav flex-column';
            section.items.forEach(item => {
                const listItem = document.createElement('li');
                listItem.className = 'nav-item';
                listItem.innerHTML = `
                    <a class="nav-link d-flex align-items-center gap-2" href="${item.url}" ${item.target ? `target="${item.target}"` : ''}>
                        <i class="bi ${item.icon}"></i>${item.name}
                    </a>`;
                list.appendChild(listItem);
            });
            sidebar.appendChild(list);
        });
    }

    function loadMenuAndTranslations(lang, cameraCount) {
        Promise.all([
            fetchJson(`json/sidebarMenu${cameraCount}.json`),
            fetchJson('json/translations.json')
        ]).then(([menuData, translationsData]) => {
            const translatedMenu = applyTranslations(menuData.menu, translationsData, lang);
            buildMenu(translatedMenu);
        }).catch(error => console.error('Error loading the menu or translations:', error));
    }

    function changeLanguage(newLanguage) {
        preferredLanguage = newLanguage;
        localStorage.setItem('preferredLanguage', newLanguage); // Save the new language preference
        loadMenuAndTranslations(preferredLanguage, cameraCount);
    }

    function changeCameraCount(newCameraCount) {
        cameraCount = newCameraCount;
        localStorage.setItem('cameraCount', newCameraCount); // Save the new language preference
        loadMenuAndTranslations(preferredLanguage, cameraCount);
    }

    // This function consolidates updates based on user input, assuming you're storing these preferences in local storage
    function updateSettingsBasedOnUserInput() {
        const settings = JSON.parse(localStorage.getItem('settings')) || {};
        settings.dst = document.getElementById('dst').checked;

        // Assume 'use24hr' is the ID for the 24-hour format preference checkbox
        settings.use24hr = document.getElementById('use24hr').checked;
        localStorage.setItem('settings', JSON.stringify(settings));
        updateStationTime(settings.timezone, settings.dst);
    }
    function updateMapCenter(lat, lng) {
        const stationCoords = [lat, lng];
        // Function to initialize the map
        function initializeMap() {
            window.stationMap = L.map('stationMap').setView(stationCoords, 13);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {}).addTo(window.stationMap);
            window.stationMarker = L.marker(stationCoords).addTo(window.stationMap);
        }
        // Check if the map is already initialized by checking for the _leaflet_id
        // This is an internal property used by Leaflet, but it's not recommended to rely on it for production code.
        // It's used here for demonstration purposes.
        if (!window.stationMap || !window.stationMap._leaflet_id) {
            initializeMap();
        } else {
            // The map is already initialized, so we just update its view and marker position
            window.stationMap.setView(stationCoords, 13);
            if (window.stationMarker) {
                window.stationMarker.setLatLng(stationCoords);
            } else {
                window.stationMarker = L.marker(stationCoords).addTo(window.stationMap);
            }
        }
    }

    function updateCurrentDateTime() {
        // Pull timezone and format preferences from local storage
        const settings = JSON.parse(localStorage.getItem('settings')) || {};
        const timezone = settings.timezone || 'UTC'; // Default to UTC if not set
        const use24hr = settings.use24hr === true; // Assume false if not explicitly set to true

        // Format options for time
        const timeOptions = {
            timeZone: timezone,
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            hour12: !use24hr
        };

        // Format options for date
        const dateOptions = {
            timeZone: timezone,
            year: 'numeric',
            month: '2-digit',
            day: '2-digit'
        };

        // Determine the current time and date in the specified timezone
        const currentTime = new Date().toLocaleTimeString('en-US', timeOptions);
        const currentDate = new Date().toLocaleDateString('en-US', dateOptions);

        // Combine time and date in the preferred order
        const currentDateTime = currentTime + ' ' + currentDate;

        // Display the current time and date in the designated element
        const currentDateTimeElement = document.getElementById('stationTime'); // Ensure this is the correct ID
        if (currentDateTimeElement) {
            currentDateTimeElement.textContent = currentDateTime;
        }
    }

</script>