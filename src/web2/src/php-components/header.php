<!-- START DARK/LIGHT MODE SELECT BOX -->
<div class="dropdown position-fixed top-3 end-0 mb-2 me-3 bd-mode-toggle" style="display: flex; align-items: center;">
    <button class="btn py-2 dropdown-toggle d-flex align-items-center" id="#bd-theme" type="button" aria-expanded="false" data-bs-toggle="dropdown" aria-label="Toggle theme (auto)" style="color: #2470dc;">
        <i class="fas fa-adjust my-1 theme-icon-active" style="width: 1em; height: 1em;"></i>
        <span class="visually-hidden">Toggle theme</span>
    </button>
    <ul class="dropdown-menu dropdown-menu-end shadow" aria-labelledby="#bd-theme">
        <li>
            <button type="button" class="dropdown-item d-flex align-items-center" data-bs-theme-value="light" aria-pressed="false">
                <i class="fas fa-sun theme-icon me-2 opacity-50" style="width: 1em; height: 1em;"></i>
                Light
                <i class="fas fa-check ms-auto d-none"></i>
            </button>
        </li>
        <li>
            <button type="button" class="dropdown-item d-flex align-items-center" data-bs-theme-value="dark" aria-pressed="false">
                <i class="fas fa-moon theme-icon me-2 opacity-50" style="width: 1em; height: 1em;"></i>
                Dark
                <i class="fas fa-check ms-auto d-none"></i>
            </button>
        </li>
        <li>
            <button type="button" class="dropdown-item d-flex align-items-center active" data-bs-theme-value="auto" aria-pressed="true">
                <i class="fas fa-adjust theme-icon me-2 opacity-50" style="width: 1em; height: 1em;"></i>
                Auto
                <i class="fas fa-check ms-auto d-none"></i>
            </button>
        </li>
    </ul>
    <!-- Settings Icon -->
    <a href="iframe.php?file=settings.php" class="ms-3" aria-label="Settings">
        <i class="fas fa-cog" style="color: #2470dc; font-size: 1.5em;"></i>
    </a>
</div>
<!-- END DARK/LIGHT MODE SELECT BOX -->

<!-- START HEADER -->
<header class="navbar sticky-top bg-dark flex-md-nowrap p-0 shadow" data-bs-theme="dark">
    <a class="navbar-brand col-md-2 col-lg-2 me-0 px-3 fs-6 text-white" href="#"><h6>BOB the Universal Object Tracker</h6></a>
</header>
