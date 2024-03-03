<div id="bobSidebarMenu"></div>

<br>



<div id="stationInfo" class="mt-3 p-3 border rounded bg-body-tertiary text-body-secondary">
    <p id="stationName">Unknown</p>
    <p id="stationTime">Loading...</p>
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
        loadStationSettings();

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

        loadMenuAndTranslations(preferredLanguage);
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

        // Example function to update station information
        updateStationInfo("Example", "America/Chicago", "41.8781", "87.6298");
        // Update time every minute
        setInterval(() => {
            updateStationTime("America/Chicago");
        }, 60000);
    });

    function loadStationSettings() {
        fetch('json/stationSettings.json')
            .then(response => response.json())
            .then(data => {
                // Using the fetched data to update station info and time
                updateStationInfo(data.stationName, data.timezone, data.lat, data.long);
                // Update time every minute based on the fetched timezone
                updateMapCenter(data.lat, data.long);
                setInterval(() => {
                    updateStationTime(data.timezone);
                }, 60000);
            })
            .catch(error => console.error('Failed to load station settings:', error));
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
        document.getElementById('stationName').textContent = `${name}`;
        document.getElementById('stationTimezone').textContent = `${timezone}`;
        document.getElementById('stationLatLong').textContent = `${lat} N, ${long} W`;
        updateStationTime(timezone);
    }

    function updateStationTime(timezone) {
        const options = {
            year: 'numeric', month: '2-digit', day: '2-digit',
            hour: '2-digit', minute: '2-digit',
            timeZone: timezone, hour12: false
        };
        const formatter = new Intl.DateTimeFormat('en-US', options);
        const now = new Date();
        const dateString = formatter.format(now);
        document.getElementById('stationTime').textContent = `${dateString}`;
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

    function loadMenuAndTranslations(lang) {
        Promise.all([
            fetchJson('json/sidebarMenu.json'),
            fetchJson('json/translations.json')
        ]).then(([menuData, translationsData]) => {
            const translatedMenu = applyTranslations(menuData.menu, translationsData, lang);
            buildMenu(translatedMenu);
        }).catch(error => console.error('Error loading the menu or translations:', error));
    }

    function changeLanguage(newLanguage) {
        preferredLanguage = newLanguage;
        localStorage.setItem('preferredLanguage', newLanguage); // Save the new language preference
        loadMenuAndTranslations(preferredLanguage);
    }

    function updateMapCenter(lat, lng) {
        const stationCoords = [lat, lng];
        const map = L.map('stationMap').setView(stationCoords, 13);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        }).addTo(map);
        L.marker(stationCoords).addTo(map);
    }

</script>