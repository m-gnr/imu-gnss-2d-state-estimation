"""Heads-up overlay: info panel, fault banner, layer legend, controls hint."""

import pygame


class Hud:
    def __init__(self, map_rect, data, faults_cfg, colors, cfg):
        self.map_rect = map_rect
        self.data = data
        self.c = colors
        self.cfg = cfg
        self.speed_unit = cfg["charts"].get("speed_unit", "kmh")
        self.fault_spans = []
        for key, label in (("dropout", "GNSS DROPOUT"),
                           ("position_jump", "GNSS POSITION JUMP"),
                           ("freeze", "GNSS FREEZE")):
            fc = faults_cfg.get(key, {})
            if fc.get("enabled"):
                self.fault_spans.append((float(fc["start"]), float(fc["end"]), label))

    def draw(self, surface, sim_t, veh, method_name, method_idx, font, font_big,
             font_small, blink, method_color=None):
        method_color = method_color or self.c["estimate"]
        self._panel(surface, sim_t, veh, method_name, method_idx, font, font_big,
                    method_color)
        self._banner(surface, sim_t, font, blink)
        self._legend(surface, method_name, font_small, method_color)
        self._controls(surface, font_small)

    def _panel(self, surface, sim_t, veh, method_name, method_idx, font, font_big,
               method_color):
        r = self.map_rect
        panel = pygame.Rect(r.left + 14, r.top + 14, 232, 132)
        s = pygame.Surface(panel.size, pygame.SRCALPHA)
        s.fill((*self.c["panel"], 235))
        surface.blit(s, panel.topleft)
        pygame.draw.rect(surface, self.c["grid"], panel, 1, border_radius=8)

        conv = 3.6 if self.speed_unit == "kmh" else 1.0
        unit = "km/h" if self.speed_unit == "kmh" else "m/s"
        err = self.data.pos_error_at(method_idx, sim_t)

        rows = [
            ("SPEED", f"{veh['v'] * conv:.0f} {unit}", self.c["text"]),
            ("TIME", f"{sim_t:.1f} / {self.data.t1:.0f} s", self.c["text"]),
            ("METHOD", method_name, method_color),
            ("POS ERROR", f"{err:.1f} m", method_color),
        ]
        y = panel.top + 12
        for k, (lab, val, col) in enumerate(rows):
            surface.blit(font.render(lab, True, self.c["text_dim"]),
                         (panel.left + 14, y))
            v = font_big.render(val, True, col)
            surface.blit(v, (panel.right - v.get_width() - 14, y - 3))
            y += 30

    def _banner(self, surface, sim_t, font, blink):
        active = next((lab for s, e, lab in self.fault_spans if s <= sim_t <= e), None)
        if not active or not blink:
            return
        r = self.map_rect
        w = 300
        banner = pygame.Rect(r.centerx - w // 2, r.top + 16, w, 38)
        s = pygame.Surface(banner.size, pygame.SRCALPHA)
        s.fill((*self.c["warning"], 60))
        surface.blit(s, banner.topleft)
        pygame.draw.rect(surface, self.c["warning"], banner, 2, border_radius=19)
        pygame.draw.circle(surface, self.c["warning"],
                           (banner.left + 24, banner.centery), 6)
        label = font.render(active, True, self.c["text"])
        surface.blit(label, (banner.left + 44, banner.centery - 9))

    def _legend(self, surface, method_name, font_small, method_color):
        r = self.map_rect
        items = [("Ground truth", self.c["ground_truth"]),
                 (f"Estimate ({method_name})", method_color),
                 ("GNSS valid", self.c["gnss_valid"]),
                 ("GNSS invalid", self.c["gnss_invalid"])]
        box = pygame.Rect(r.left + 14, r.bottom - 14 - len(items) * 18 - 8,
                          178, len(items) * 18 + 8)
        s = pygame.Surface(box.size, pygame.SRCALPHA)
        s.fill((*self.c["panel"], 220))
        surface.blit(s, box.topleft)
        y = box.top + 8
        for label, col in items:
            pygame.draw.circle(surface, col, (box.left + 14, y + 6), 5)
            surface.blit(font_small.render(label, True, self.c["text"]),
                         (box.left + 28, y))
            y += 18

    def _controls(self, surface, font_small):
        r = self.map_rect
        hint = "Space play/pause  ·  ← → ±5s  ·  ↑↓ speed  ·  M method  ·  O orient  ·  G gnss"
        label = font_small.render(hint, True, self.c["text_dim"])
        surface.blit(label, (r.right - label.get_width() - 14, r.bottom - 22))
