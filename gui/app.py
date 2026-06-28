"""Main Pygame application: wires data, camera, minimap, charts, HUD, timeline."""

import os
from pathlib import Path

import pygame
import yaml

from .camera import Camera
from .charts import build_strip_charts
from .data_loader import SimData
from .hud import Hud
from .minimap import Minimap
from .playback import Playback
from .timeline import Timeline


def _color_map(raw: dict) -> dict:
    out = {}
    for k, v in raw.items():
        if isinstance(v, list) and v and isinstance(v[0], (int, float)):
            out[k] = tuple(int(c) for c in v)
        else:
            out[k] = v
    return out


class App:
    def __init__(self, config_path: str | Path = "config.yaml"):
        self.config_path = Path(config_path)
        with self.config_path.open("r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        self.gui = self.config["gui"]
        self.colors = _color_map(self.gui["colors"])
        base = self.config_path.resolve().parent
        data_path = base / self.gui["data_source"]
        self.data = SimData(data_path)

        self.W = int(self.gui["window"]["width"])
        self.H = int(self.gui["window"]["height"])
        self.fps = int(self.gui["window"]["fps"])

        pygame.init()
        pygame.display.set_caption(self.gui["window"].get("title", "Minimap"))
        self.screen = pygame.display.set_mode((self.W, self.H))
        self.clock = pygame.time.Clock()

        self.font = pygame.font.SysFont("dejavusans,arial", 13)
        self.font_big = pygame.font.SysFont("dejavusans,arial", 19)
        self.font_small = pygame.font.SysFont("dejavusans,arial", 11)

        self._build()

    def _build(self):
        charts_h = int(self.gui["charts"]["height"])
        timeline_h = 46
        map_h = self.H - charts_h - timeline_h

        self.map_rect = pygame.Rect(0, 0, self.W, map_h)
        self.charts_band = pygame.Rect(10, map_h + 8, self.W - 20, charts_h - 16)
        self.timeline_rect = pygame.Rect(0, self.H - timeline_h, self.W, timeline_h)

        zoom_m = float(self.gui["map"]["zoom_m"])
        px_per_m = (min(self.map_rect.width, self.map_rect.height) * 0.5) / zoom_m
        self.orientation = self.gui["map"]["orientation"]
        self.camera = Camera(self.map_rect.centerx, self.map_rect.centery,
                             px_per_m, self.orientation)

        self.method_idx = self.data.method_index(self.gui["map"]["default_method"]) \
            if self.gui["map"]["default_method"] in self.data.methods else 0
        self.layers = dict(self.gui["layers"])

        self.compare_all = bool(self.gui["charts"].get("compare_all_methods", False))
        cycle = [tuple(int(x) for x in c) for c in self.colors["method_cycle"]]
        self.method_colors = [cycle[i % len(cycle)]
                              for i in range(len(self.data.methods))]
        self.minimap = Minimap(self.map_rect, self.camera, self.data,
                               self.colors, self.gui)
        self.charts = build_strip_charts(self.data, self.gui, self.colors,
                                         self.charts_band, self.method_idx)
        self.hud = Hud(self.map_rect, self.data, self.config["faults"],
                       self.colors, self.gui)
        self.timeline = Timeline(self.timeline_rect, self.data.t0, self.data.t1,
                                 self.config["faults"], self.colors)

        pb = self.gui["playback"]
        self.playback = Playback(self.data.t0, self.data.t1, pb["speed_levels"],
                                 pb["initial_speed"], pb["start_paused"])

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            return False
        if event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_ESCAPE, pygame.K_q):
                return False
            elif event.key == pygame.K_SPACE:
                self.playback.toggle()
            elif event.key == pygame.K_LEFT:
                self.playback.nudge(-5)
            elif event.key == pygame.K_RIGHT:
                self.playback.nudge(5)
            elif event.key == pygame.K_UP:
                self.playback.faster()
            elif event.key == pygame.K_DOWN:
                self.playback.slower()
            elif event.key == pygame.K_m:
                self.method_idx = (self.method_idx + 1) % len(self.data.methods)
                if not self.compare_all:
                    self.charts = build_strip_charts(
                        self.data, self.gui, self.colors, self.charts_band,
                        self.method_idx)
            elif event.key == pygame.K_o:
                self.orientation = ("north_up" if self.orientation == "heading_up"
                                    else "heading_up")
                self.camera.orientation = self.orientation
            elif event.key == pygame.K_g:
                self.layers["gnss_points"] = not self.layers["gnss_points"]
            elif event.key == pygame.K_t:
                self.layers["ground_truth"] = not self.layers["ground_truth"]
            elif event.key == pygame.K_e:
                self.layers["estimate"] = not self.layers["estimate"]
        self.timeline.handle_event(event, self.playback)
        return True

    def draw(self):
        self.screen.fill(self.colors["background"])
        veh = self.data.true_state(self.playback.sim_t)
        method_name = self.data.methods[self.method_idx]
        method_color = self.method_colors[self.method_idx]

        self.minimap.draw(self.screen, self.playback.sim_t, veh,
                          self.method_idx, self.layers, method_color)
        for chart in self.charts:
            chart.draw(self.screen, self.playback.sim_t, self.font, self.font_small)

        blink = (pygame.time.get_ticks() // 400) % 2 == 0
        self.hud.draw(self.screen, self.playback.sim_t, veh, method_name,
                      self.method_idx, self.font, self.font_big, self.font_small,
                      blink, method_color)
        self.timeline.draw(self.screen, self.playback, self.font)

    def run(self):
        running = True
        while running:
            dt_real = self.clock.tick(self.fps) / 1000.0
            for event in pygame.event.get():
                running = self.handle_event(event) and running
            self.playback.update(dt_real)
            self.draw()
            pygame.display.flip()
        pygame.quit()

    def save_frame(self, sim_t, path):
        """Render a single frame at the given sim time (for headless testing)."""
        self.playback.scrub_to(sim_t)
        self.draw()
        pygame.display.flip()
        pygame.image.save(self.screen, str(path))
