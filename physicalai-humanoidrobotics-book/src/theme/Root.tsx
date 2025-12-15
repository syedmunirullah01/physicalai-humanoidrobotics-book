import React, { useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function Root({children}) {
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }

    // Add theme toggle functionality using direct DOM manipulation
    const handleThemeToggle = () => {
      const html = document.documentElement;
      const currentTheme = html.getAttribute('data-theme') || 'dark';
      const newTheme = currentTheme === 'dark' ? 'light' : 'dark';

      // Update data-theme attribute
      html.setAttribute('data-theme', newTheme);

      // Store preference in localStorage
      try {
        localStorage.setItem('theme', newTheme);
      } catch (err) {
        console.error('Failed to save theme preference:', err);
      }
    };

    // Reorder navbar items: Search, Sign In, Sign Up, Theme Toggle (at end)
    const reorderNavbarItems = () => {
      const navbarRight = document.querySelector('.navbar__items--right');
      if (!navbarRight || navbarRight.hasAttribute('data-reordered')) return;

      // Find all navbar items
      const searchBox = navbarRight.querySelector('.navbar__search, [class*="searchBox"]');
      const signInButton = Array.from(navbarRight.querySelectorAll('.navbar__item')).find(
        item => item.querySelector('.navbar-auth-button')
      );
      const signUpButton = Array.from(navbarRight.querySelectorAll('.navbar__item')).find(
        item => item.querySelector('.navbar-cta-button')
      );
      const themeToggle = Array.from(navbarRight.querySelectorAll('.navbar__item')).find(
        item => item.querySelector('.navbar-theme-toggle')
      );

      // Reorder: Search first, then Sign In, Sign Up, Theme Toggle at the end
      if (searchBox) navbarRight.appendChild(searchBox);
      if (signInButton) navbarRight.appendChild(signInButton);
      if (signUpButton) navbarRight.appendChild(signUpButton);
      if (themeToggle) navbarRight.appendChild(themeToggle);

      navbarRight.setAttribute('data-reordered', 'true');
    };

    // Add event listener to theme toggle button
    const addThemeToggleListener = () => {
      const themeToggle = document.querySelector('.navbar-theme-toggle');
      if (themeToggle && !themeToggle.hasAttribute('data-listener-attached')) {
        themeToggle.addEventListener('click', handleThemeToggle);
        themeToggle.setAttribute('data-listener-attached', 'true');
      }
    };

    // Custom cursor circle tracking
    const cursorCircle = document.getElementById('cursor-circle');
    let mouseX = 0;
    let mouseY = 0;
    let cursorX = 0;
    let cursorY = 0;

    const handleMouseMove = (e: MouseEvent) => {
      mouseX = e.clientX;
      mouseY = e.clientY;

      if (cursorCircle) {
        cursorCircle.style.opacity = '1';
      }
    };

    const handleMouseEnter = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      if (target.tagName === 'A' || target.tagName === 'BUTTON' || target.closest('a, button')) {
        cursorCircle?.classList.add('cursor-hover');
      }
    };

    const handleMouseLeave = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      if (target.tagName === 'A' || target.tagName === 'BUTTON' || target.closest('a, button')) {
        cursorCircle?.classList.remove('cursor-hover');
      }
    };

    // Smooth animation loop for cursor
    const animateCursor = () => {
      // Smooth following with easing
      const speed = 0.15;
      cursorX += (mouseX - cursorX) * speed;
      cursorY += (mouseY - cursorY) * speed;

      if (cursorCircle) {
        cursorCircle.style.transform = `translate(${cursorX}px, ${cursorY}px)`;
      }

      requestAnimationFrame(animateCursor);
    };

    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseover', handleMouseEnter);
    document.addEventListener('mouseout', handleMouseLeave);
    animateCursor();

    // Initial setup with delay to ensure DOM is ready
    const timeoutId = setTimeout(() => {
      reorderNavbarItems();
      addThemeToggleListener();
    }, 100);

    // Re-attach listener and reorder on navigation (for SPA routing)
    const observer = new MutationObserver(() => {
      reorderNavbarItems();
      addThemeToggleListener();
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true,
    });

    return () => {
      clearTimeout(timeoutId);
      observer.disconnect();
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseover', handleMouseEnter);
      document.removeEventListener('mouseout', handleMouseLeave);
    };
  }, []);

  return (
    <>
      {/* Custom Cursor Circle */}
      <div className="custom-cursor-circle" id="cursor-circle"></div>

      {/* Animated Background Elements */}
      <div className="hex-pattern"></div>
      <div className="scan-line"></div>
      <div className="corner-brackets"></div>

      {/* Glowing Dots */}
      <div className="glow-dots">
        <div className="glow-dot"></div>
        <div className="glow-dot"></div>
        <div className="glow-dot"></div>
        <div className="glow-dot"></div>
        <div className="glow-dot"></div>
      </div>

      {/* Data Streams */}
      <div className="data-stream"></div>
      <div className="data-stream"></div>
      <div className="data-stream"></div>
      <div className="data-stream"></div>
      <div className="data-stream"></div>

      {/* Floating Particles */}
      <div className="floating-particles">
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
        <div className="particle"></div>
      </div>

      {children}
    </>
  );
}
