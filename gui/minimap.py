"""Minimap rendering: route, trail, GNSS points, vehicle marker, compass."""

import math

import numpy as np
import pygame


class Minimap:
    def __init__(self, rect: pygame.Rect, camera, data, colors: dict, cfg: dict):
        self.rect = rect
        self.camera = camera
        self.data = data
        self.c = colors
        self.cfg = cfg
        self.trail_seconds = float(cfg["map"]["trail_seconds"])
        # Heavy decimation of the full route for the faint reference line.
        step = max(1, data.n // 1500)
        self._route_idx = np.arange(0, data.n, step)

    def draw(self, surface, sim_t, veh, method_idx, layers, estimate_color=None):
        estimate_color = estimate_color or self.c["estimate"]
        prev_clip = surface.get_clip()
        surface.set_clip(self.rect)
        surface.fill(self.c["background"], self.rect)
        self._draw_grid(surface)

        vx, vy, vpsi = veh["x"], veh["y"], veh["psi"]

        if layers["ground_truth"]:
            self._draw_route(surface, vx, vy, vpsi)

        if layers["gnss_points"]:
            self._draw_gnss(surface, sim_t, vx, vy, vpsi)

        idx = int(round(self.data.frac_to_index(sim_t)))
        i0 = max(0, idx - int(self.trail_seconds / self.data.dt))

        if layers["ground_truth"]:
            self._draw_trail(surface, self.data.true_x, self.data.true_y,
                             i0, idx, vx, vy, vpsi, self.c["ground_truth"], 3)
        if layers["estimate"]:
            self._draw_trail(surface, self.data.est_x[method_idx],
                             self.data.est_y[method_idx],
                             i0, idx, vx, vy, vpsi, estimate_color, 2)

        self._draw_vehicle(surface, vpsi)
        surface.set_clip(prev_clip)
        self._draw_compass(surface, vpsi)

    def _draw_grid(self, surface):
        col = self.c["grid"]
        r = self.rect
        spacing = 60
        faint = (col[0], col[1], col[2])
        grid = pygame.Surface((r.width, r.height), pygame.SRCALPHA)
        for gx in range(0, r.width, spacing):
            pygame.draw.line(grid, (*faint, 26), (gx, 0), (gx, r.height))
        for gy in range(0, r.height, spacing):
            pygame.draw.line(grid, (*faint, 26), (0, gy), (r.width, gy))
        surface.blit(grid, r.topleft)

    def _draw_route(self, surface, vx, vy, vpsi):
        pts = self.camera.screen_points(
            self.data.true_x[self._route_idx], self.data.true_y[self._route_idx],
            vx, vy, vpsi,
        )
        pts = self._clip_run(pts)
        if len(pts) >= 2:
            col = self.c["ground_truth"]
            faint = (col[0] // 3 + 30, col[1] // 3 + 30, col[2] // 3 + 30)
            pygame.draw.aalines(surface, faint, False, pts.tolist())

    def _draw_trail(self, surface, X, Y, i0, i1, vx, vy, vpsi, color, width):
        if i1 - i0 < 2:
            return
        pts = self.camera.screen_points(X[i0:i1 + 1], Y[i0:i1 + 1], vx, vy, vpsi)
        pts = self._clip_run(pts)
        if len(pts) >= 2:
            pygame.draw.lines(surface, color, False, pts.tolist(), width)

    def _draw_gnss(self, surface, sim_t, vx, vy, vpsi):
        mask = self.data.gnss_t <= sim_t
        if not np.any(mask):
            return
        gx = self.data.gnss_x[mask]
        gy = self.data.gnss_y[mask]
        valid = self.data.gnss_valid[mask]
        sx, sy = self.camera.world_to_screen(gx, gy, vx, vy, vpsi)
        r = self.rect
        inside = (sx >= r.left) & (sx <= r.right) & (sy >= r.top) & (sy <= r.bottom)
        for x, y, v in zip(sx[inside], sy[inside], valid[inside]):
            color = self.c["gnss_valid"] if v else self.c["gnss_invalid"]
            pygame.draw.circle(surface, color, (int(x), int(y)), 3 if v else 4)

    def _draw_vehicle(self, surface, vpsi):
        cx, cy = self.camera.cx, self.camera.cy
        ang = self.camera.vehicle_heading_screen(vpsi)  # 0 = up, clockwise
        size = 11
        # Triangle pointing up, rotated by -ang for screen (clockwise positive).
        base = [(0, -size), (size * 0.7, size * 0.8),
                (0, size * 0.45), (-size * 0.7, size * 0.8)]
        ca, sa = math.cos(-ang), math.sin(-ang)
        pts = [(cx + px * ca - py * sa, cy + px * sa + py * ca) for px, py in base]
        # Subtle range ring around the vehicle.
        ring = pygame.Surface((self.rect.width, self.rect.height), pygame.SRCALPHA)
        pygame.draw.circle(ring, (*self.c["vehicle"], 28),
                           (int(cx - self.rect.left), int(cy - self.rect.top)), 58, 1)
        surface.blit(ring, self.rect.topleft)
        pygame.draw.polygon(surface, self.c["vehicle"], pts)
        pygame.draw.polygon(surface, self.c["background"], pts, 1)

    def _draw_compass(self, surface, vpsi):
        r = self.rect
        cx, cy = r.right - 44, r.top + 44
        pygame.draw.circle(surface, self.c["panel"], (cx, cy), 26)
        pygame.draw.circle(surface, self.c["grid"], (cx, cy), 26, 1)
        # North direction in screen space.
        theta = self.camera._theta(vpsi)  # rotation applied to the world
        # World north is +y; after rotation by theta its screen angle:
        nx = -math.sin(theta)
        ny = -math.cos(theta)
        tip = (cx + nx * 20, cy + ny * 20)
        pygame.draw.line(surface, self.c["warning"], (cx, cy), tip, 2)

    def _clip_run(self, pts):
        """Drop points far outside the map rect to keep draw calls cheap."""
        r = self.rect
        margin = 200
        inside = (
            (pts[:, 0] >= r.left - margin) & (pts[:, 0] <= r.right + margin)
            & (pts[:, 1] >= r.top - margin) & (pts[:, 1] <= r.bottom + margin)
        )
        return pts[inside]
