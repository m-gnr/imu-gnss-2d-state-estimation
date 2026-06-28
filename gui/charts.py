"""Live time-series strip charts drawn natively in Pygame.

Each strip caches its static curves to a Surface once, then every frame only
the moving playhead cursor and the current numeric readouts are drawn on top.
"""

import numpy as np
import pygame


class StripChart:
    def __init__(self, rect, title, t, series, colors, value_fmt="{:.1f}",
                 yclip_pct=99.5):
        self.rect = rect
        self.title = title
        self.t = t
        self.t0, self.t1 = float(t[0]), float(t[-1])
        self.series = series                     # list of {label, color, y}
        self.c = colors
        self.value_fmt = value_fmt

        all_y = np.concatenate([s["y"] for s in series])
        self.ymin = float(np.min(all_y))
        # Robust max so rare fault spikes do not flatten the nominal detail;
        # spikes are still drawn (clipped) and the exact value shows in readout.
        self.ymax = float(np.percentile(all_y, yclip_pct))
        if self.ymax <= self.ymin:
            self.ymax = self.ymin + 1.0

        self._cache = None
        self._build_cache()

    def _x(self, t):
        r = self.rect
        return r.left + (t - self.t0) / (self.t1 - self.t0) * r.width

    def _y(self, v):
        r = self.rect
        f = (v - self.ymin) / (self.ymax - self.ymin)
        f = min(max(f, 0.0), 1.0)
        return r.bottom - f * r.height

    def _build_cache(self):
        r = self.rect
        surf = pygame.Surface((r.width, r.height), pygame.SRCALPHA)
        # zero baseline if within range
        if self.ymin < 0 < self.ymax:
            y0 = (self.rect.bottom - (0 - self.ymin) / (self.ymax - self.ymin)
                  * r.height) - r.top
            pygame.draw.line(surf, (*self.c["grid"], 60), (0, y0), (r.width, y0))

        step = max(1, len(self.t) // r.width)
        idx = np.arange(0, len(self.t), step)
        xs = (self._x(self.t[idx]) - r.left)
        for s in self.series:
            ys = np.array([self._y(v) for v in s["y"][idx]]) - r.top
            pts = np.column_stack([xs, ys]).tolist()
            if len(pts) >= 2:
                col = s["color"]
                pygame.draw.aalines(surf, (col[0], col[1], col[2]), False, pts)
        self._cache = surf

    def draw(self, surface, sim_t, font, font_small):
        r = self.rect
        pygame.draw.rect(surface, self.c["panel"], r, border_radius=6)
        surface.blit(self._cache, r.topleft)
        pygame.draw.rect(surface, self.c["grid"], r, 1, border_radius=6)

        # playhead cursor
        cx = self._x(sim_t)
        pygame.draw.line(surface, self.c["text_dim"], (cx, r.top), (cx, r.bottom), 1)

        # title
        surface.blit(font.render(self.title, True, self.c["text"]),
                     (r.left + 8, r.top + 5))

        # current values (stacked top-right)
        idx = int(round((sim_t - self.t0) / (self.t1 - self.t0) * (len(self.t) - 1)))
        idx = min(max(idx, 0), len(self.t) - 1)
        ty = r.top + 5
        for s in self.series:
            val = self.value_fmt.format(s["y"][idx])
            label = font_small.render(f"{s['label']} {val}", True, s["color"])
            surface.blit(label, (r.right - label.get_width() - 8, ty))
            ty += 15


def build_strip_charts(data, cfg, colors, band_rect, selected_idx=0):
    """
    Create the configured strip charts laid out side by side in band_rect.

    When charts.compare_all_methods is false (default) only the selected method
    is drawn, in the estimate colour, so the strips match the map trail; the
    caller rebuilds them when the active method changes. When true, all methods
    are overlaid for an at-a-glance comparison.
    """
    signals = cfg["charts"]["signals"]
    compare = bool(cfg["charts"].get("compare_all_methods", False))
    speed_unit = cfg["charts"].get("speed_unit", "kmh")
    cycle = [tuple(c) for c in colors["method_cycle"]]
    dim = colors["text_dim"]

    def method_color(m):
        # Each method keeps its own identity colour in both modes.
        return cycle[m % len(cycle)]

    n = len(signals)
    gap = 10
    w = (band_rect.width - gap * (n - 1)) // n
    charts = []

    for k, sig in enumerate(signals):
        rect = pygame.Rect(band_rect.left + k * (w + gap), band_rect.top,
                           w, band_rect.height)
        method_range = range(len(data.methods)) if compare else [selected_idx]

        if sig == "position_error":
            series = [{"label": _abbr(data.methods[m]),
                       "color": method_color(m),
                       "y": data.pos_error[m]} for m in method_range]
            charts.append(StripChart(rect, "Position error [m]", data.t,
                                     series, colors, "{:.1f}"))
        elif sig == "speed":
            conv = 3.6 if speed_unit == "kmh" else 1.0
            unit = "km/h" if speed_unit == "kmh" else "m/s"
            series = [{"label": "GT", "color": dim, "y": data.true_v * conv}]
            series += [{"label": _abbr(data.methods[m]),
                        "color": method_color(m),
                        "y": data.est_speed[m] * conv} for m in method_range]
            charts.append(StripChart(rect, f"Speed [{unit}]", data.t,
                                     series, colors, "{:.0f}"))
        elif sig == "yaw":
            series = [{"label": "GT", "color": dim,
                       "y": np.rad2deg(data.true_psi)}]
            series += [{"label": _abbr(data.methods[m]),
                        "color": method_color(m),
                        "y": np.rad2deg(data.est_psi[m])} for m in method_range]
            charts.append(StripChart(rect, "Yaw [deg]", data.t,
                                     series, colors, "{:.0f}"))
    return charts


def _abbr(name: str) -> str:
    return {
        "Raw GNSS": "Raw",
        "Low-pass Filter": "LPF",
        "Complementary Filter": "Comp",
        "EKF": "EKF",
    }.get(name, name[:4])
