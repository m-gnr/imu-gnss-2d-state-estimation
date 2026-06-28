"""World-to-screen transform for the minimap (heading-up or north-up)."""

import math

import numpy as np


class Camera:
    """
    Maps world coordinates (metres) to minimap screen pixels.

    The vehicle is always at the minimap centre. In heading-up mode the whole
    map is rotated so the vehicle heading points to screen-up; in north-up mode
    the map stays axis-aligned and only the vehicle marker rotates.
    """

    def __init__(self, cx: float, cy: float, px_per_m: float, orientation: str):
        self.cx = cx
        self.cy = cy
        self.px_per_m = px_per_m
        self.orientation = orientation  # "heading_up" | "north_up"

    def _theta(self, veh_psi: float) -> float:
        if self.orientation == "heading_up":
            return math.pi / 2.0 - veh_psi
        return 0.0

    def world_to_screen(self, X, Y, veh_x, veh_y, veh_psi):
        """
        Transform scalar or array world coords to screen coords.
        Returns (sx, sy) of the same shape as the inputs.
        """
        dx = np.asarray(X, dtype=float) - veh_x
        dy = np.asarray(Y, dtype=float) - veh_y

        theta = self._theta(veh_psi)
        ct, st = math.cos(theta), math.sin(theta)

        rx = dx * ct - dy * st
        ry = dx * st + dy * ct

        sx = self.cx + rx * self.px_per_m
        sy = self.cy - ry * self.px_per_m  # screen y grows downward
        return sx, sy

    def screen_points(self, X, Y, veh_x, veh_y, veh_psi):
        """Return an (N, 2) int array of screen points for a polyline."""
        sx, sy = self.world_to_screen(X, Y, veh_x, veh_y, veh_psi)
        return np.column_stack([sx, sy])

    def vehicle_heading_screen(self, veh_psi: float) -> float:
        """
        Screen-space heading angle of the vehicle marker, in radians measured
        from screen-up, clockwise. Heading-up -> always 0 (points up).
        """
        if self.orientation == "heading_up":
            return 0.0
        # north_up: convert world heading (CCW from +x) to screen-up clockwise
        return math.pi / 2.0 - veh_psi
