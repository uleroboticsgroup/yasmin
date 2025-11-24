// Make sidebar sections collapsible
document.addEventListener('DOMContentLoaded', function() {
    const sidebarHeaders = document.querySelectorAll('.sidebar h3');
    const currentPath = window.location.pathname;
    
    // Load saved sidebar state from sessionStorage
    const savedState = sessionStorage.getItem('sidebarState');
    let sidebarState = savedState ? JSON.parse(savedState) : {};
    
    // Highlight active link based on current URL
    const links = document.querySelectorAll('.sidebar a');
    const currentUrl = window.location.href.split('#')[0].split('?')[0];

    links.forEach(link => {
        // Use the absolute href property which resolves relative paths
        const linkUrl = link.href.split('#')[0].split('?')[0];
        
        if (linkUrl === currentUrl) {
            link.classList.add('active');
        }
    });
    
    sidebarHeaders.forEach(header => {
        const headerText = header.textContent.trim();
        
        // Only make "C++ Tutorials" and "Python Tutorials" sections collapsible
        if (!['C++ Tutorials', 'Python Tutorials'].includes(headerText)) {
            return;
        }
        
        // Mark as collapsible to show arrow (if not already marked in HTML)
        if (!header.classList.contains('collapsible')) {
            header.classList.add('collapsible');
        }
        
        // All of these are nested collapsibles
        const isNested = true;
        
        // Check if any child section contains the active link
        let hasActiveLink = false;
        let currentElement = header.nextElementSibling;
        
        // Collect all elements until next h3 (simple for nested collapsibles)
        const childElements = [];
        while (currentElement) {
            if (currentElement.tagName === 'H3') {
                break;
            }
            childElements.push(currentElement);
            if (currentElement.tagName === 'UL') {
                const activeLink = currentElement.querySelector('a.active');
                if (activeLink) {
                    hasActiveLink = true;
                }
            }
            currentElement = currentElement.nextElementSibling;
        }
        
        // Check saved state or default to collapsed
        const isCollapsed = sidebarState[headerText] !== undefined ? sidebarState[headerText] : true;
        
        if (isCollapsed) {
            header.classList.add('collapsed');
            childElements.forEach(el => {
                el.style.display = 'none';
            });
        } else {
            header.classList.remove('collapsed');
            childElements.forEach(el => {
                el.style.display = 'block';
            });
        }
        
        // Add click handler
        header.addEventListener('click', function(e) {
            e.preventDefault(); // Prevent default anchor behavior
            e.stopPropagation(); // Prevent event bubbling
            
            this.classList.toggle('collapsed');
            const isNowCollapsed = this.classList.contains('collapsed');
            
            // Save state
            sidebarState[headerText] = isNowCollapsed;
            sessionStorage.setItem('sidebarState', JSON.stringify(sidebarState));
            
            // Toggle child elements
            let nextElement = this.nextElementSibling;
            while (nextElement) {
                if (nextElement.tagName === 'H3') {
                    break;
                }
                nextElement.style.display = isNowCollapsed ? 'none' : 'block';
                nextElement = nextElement.nextElementSibling;
            }
        });
    });

    // Add copy buttons to code blocks
    const codeBlocks = document.querySelectorAll('pre');
    codeBlocks.forEach(pre => {
        // Wrap in a container
        const wrapper = document.createElement('div');
        wrapper.className = 'code-block-wrapper';
        pre.parentNode.insertBefore(wrapper, pre);
        wrapper.appendChild(pre);
        
        // Add copy button
        const button = document.createElement('button');
        button.className = 'copy-button';
        button.textContent = 'Copy';
        wrapper.appendChild(button);
        
        button.addEventListener('click', async () => {
            const code = pre.querySelector('code')?.textContent || pre.textContent;
            try {
                await navigator.clipboard.writeText(code);
                button.textContent = 'Copied!';
                setTimeout(() => {
                    button.textContent = 'Copy';
                }, 2000);
            } catch (err) {
                console.error('Failed to copy:', err);
            }
        });
    });

    // Generate table of contents
    const mainContent = document.querySelector('.main-content');
    if (mainContent) {
        const headings = mainContent.querySelectorAll('h2');
        if (headings.length > 0) {
            const toc = document.createElement('nav');
            toc.className = 'toc';
            toc.innerHTML = '<h3>On This Page</h3><ul></ul>';
            const tocList = toc.querySelector('ul');
            
            headings.forEach((heading, index) => {
                // Add ID to heading if it doesn't have one
                if (!heading.id) {
                    heading.id = 'heading-' + index;
                }
                
                const li = document.createElement('li');
                const a = document.createElement('a');
                a.href = '#' + heading.id;
                a.textContent = heading.textContent;
                a.className = heading.tagName.toLowerCase();
                
                // Add click handler for smooth scrolling
                a.addEventListener('click', function(e) {
                    e.preventDefault();
                    const target = document.getElementById(heading.id);
                    if (target) {
                        target.scrollIntoView({ behavior: 'smooth', block: 'start' });
                        // Update URL without scrolling
                        history.pushState(null, null, '#' + heading.id);
                        // Update active state
                        tocList.querySelectorAll('a').forEach(link => link.classList.remove('active'));
                        this.classList.add('active');
                    }
                });
                
                li.appendChild(a);
                tocList.appendChild(li);
            });
            
            document.body.appendChild(toc);
            
            // Highlight active section on scroll with improved logic
            let ticking = false;
            
            function updateActiveLink() {
                // Get all headings with their positions
                const headingPositions = Array.from(headings).map(heading => ({
                    id: heading.id,
                    top: heading.getBoundingClientRect().top,
                    element: heading
                }));
                
                // Find the current active heading
                // Priority: heading closest to top of viewport (but below top offset)
                const scrollOffset = 120; // Offset from top
                let activeHeading = null;
                
                // Check if we're at the bottom of the page
                if ((window.innerHeight + window.scrollY) >= document.body.offsetHeight - 100) {
                    activeHeading = headingPositions[headingPositions.length - 1];
                } else {
                    // Find the heading that's currently in view or just passed
                    for (let i = headingPositions.length - 1; i >= 0; i--) {
                        if (headingPositions[i].top <= scrollOffset) {
                            activeHeading = headingPositions[i];
                            break;
                        }
                    }
                    
                    // If no heading is past the offset, use the first one if we're near top
                    if (!activeHeading && window.scrollY < 200 && headingPositions.length > 0) {
                        activeHeading = headingPositions[0];
                    }
                }
                
                // Update active state
                tocList.querySelectorAll('a').forEach(a => a.classList.remove('active'));
                if (activeHeading) {
                    const activeLink = tocList.querySelector(`a[href="#${activeHeading.id}"]`);
                    if (activeLink) {
                        activeLink.classList.add('active');
                    }
                }
                
                ticking = false;
            }
            
            function requestTick() {
                if (!ticking) {
                    window.requestAnimationFrame(updateActiveLink);
                    ticking = true;
                }
            }
            
            // Update on scroll
            window.addEventListener('scroll', requestTick);
            
            // Initial update
            updateActiveLink();
        }
    }
});
