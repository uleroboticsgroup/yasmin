(function() {
    const currentTheme = localStorage.getItem('theme');
    if (currentTheme === 'dark-mode') {
        document.documentElement.classList.add('dark-mode');
    }
})();

document.addEventListener('DOMContentLoaded', () => {
    const toggleButton = document.getElementById('theme-toggle');
    const html = document.documentElement;
    
    // Set initial button state
    if (html.classList.contains('dark-mode')) {
        toggleButton.textContent = '☀️';
    } else {
        toggleButton.textContent = '🌙';
    }
    
    toggleButton.addEventListener('click', () => {
        html.classList.toggle('dark-mode');
        
        let theme = 'light';
        if (html.classList.contains('dark-mode')) {
            theme = 'dark-mode';
            toggleButton.textContent = '☀️';
        } else {
            toggleButton.textContent = '🌙';
        }
        
        localStorage.setItem('theme', theme);
    });
});
