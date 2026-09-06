#!/usr/bin/env python3
"""Unified web entrypoint: existing scene workbench + guarded RM75 lifecycle API.

Run this module instead of ``rm75_app.web.control_panel`` for physical bring-up.
The existing UI/routes are preserved; the additional `/api/robot/*` endpoints are
registered before Flask starts.  Scenario-specific sorting/magnetic/Push-T pages
can consume the same manager without creating another robot connection.
"""

from __future__ import annotations

from rm75_app.web.control_panel import app, main
from rm75_app.web.realman_hardware_api import (
    RealManHardwareManager,
    create_realman_hardware_blueprint,
)


realman_hardware_manager = RealManHardwareManager()
app.register_blueprint(
    create_realman_hardware_blueprint(realman_hardware_manager)
)


if __name__ == "__main__":
    main()
