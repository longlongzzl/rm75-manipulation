"""Dependency checks shared by the local browser acceptance entry points.

Checks do not create evidence directories, install software or launch browsers.
"""
from pathlib import Path
import os
import shutil
import sys


def playwright_api():
    try:
        from playwright.sync_api import sync_playwright, expect
    except ImportError as exc:
        raise RuntimeError(
            f'Playwright is unavailable in {sys.executable}. Run this tool with '
            'the existing browser-acceptance venv (for example '
            '~/.venvs/console-acceptance/bin/python). No evidence directory was created.'
        ) from exc
    return sync_playwright, expect


def chromium_options(playwright, explicit=None):
    if explicit is not None:
        path = Path(explicit).expanduser().resolve()
    else:
        bundled = Path(playwright.chromium.executable_path)
        installed = next((shutil.which(name) for name in ('chromium', 'chromium-browser', 'google-chrome')
                          if shutil.which(name)), None)
        path = bundled if bundled.is_file() else Path(installed) if installed else bundled
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(
            'No executable Chromium found. Pass --chromium with the existing '
            'Playwright/distro browser path. No evidence directory was created.'
        )
    return {'headless': True, 'executable_path': str(path)}
