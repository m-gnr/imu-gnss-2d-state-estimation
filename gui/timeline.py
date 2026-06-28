"""Bottom scrub bar: progress track, fault markers, draggable playhead."""

import pygame


class Timeline:
    def __init__(self, rect, t0, t1, faults_cfg, colors):
        self.rect = rect
        self.t0 = t0
        self.t1 = t1
        self.c = colors
        self.dragging = False
        self.fault_spans = self._collect_faults(faults_cfg)

    def _collect_faults(self, faults_cfg):
        spans = []
        for key in ("dropout", "position_jump", "freeze"):
            fc = faults_cfg.get(key, {})
            if fc.get("enabled"):
                spans.append((float(fc["start"]), float(fc["end"])))
        return spans

    def _track(self):
        r = self.rect
        return pygame.Rect(r.left + 16, r.centery - 3, r.width - 150, 6)

    def _x_to_time(self, x):
        tr = self._track()
        f = (x - tr.left) / tr.width
        return self.t0 + min(max(f, 0.0), 1.0) * (self.t1 - self.t0)

    def _time_to_x(self, t):
        tr = self._track()
        return tr.left + (t - self.t0) / (self.t1 - self.t0) * tr.width

    def draw(self, surface, playback, font):
        r = self.rect
        pygame.draw.rect(surface, self.c["panel"], r)
        tr = self._track()

        # fault regions
        for s, e in self.fault_spans:
            x0 = self._time_to_x(s)
            x1 = self._time_to_x(e)
            pygame.draw.rect(surface, self.c["warning"],
                             pygame.Rect(x0, tr.top - 4, max(2, x1 - x0), tr.height + 8))

        pygame.draw.rect(surface, self.c["grid"], tr, border_radius=3)
        px = self._time_to_x(playback.sim_t)
        filled = pygame.Rect(tr.left, tr.top, max(0, px - tr.left), tr.height)
        pygame.draw.rect(surface, self.c["gnss_valid"], filled, border_radius=3)
        pygame.draw.circle(surface, self.c["text"], (int(px), tr.centery), 8)

        # play state + speed + time readout
        state = "PAUSE" if playback.playing else "PLAY"
        txt = f"{state}  {playback.speed:g}x   {playback.sim_t:6.1f} / {self.t1:.0f} s"
        label = font.render(txt, True, self.c["text"])
        surface.blit(label, (r.right - label.get_width() - 14, r.centery - 9))

    def handle_event(self, event, playback):
        tr = self._track()
        hot = self.rect.collidepoint(*pygame.mouse.get_pos())
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and \
                self.rect.collidepoint(event.pos) and event.pos[0] <= tr.right + 12:
            self.dragging = True
            playback.scrub_to(self._x_to_time(event.pos[0]))
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            playback.scrub_to(self._x_to_time(event.pos[0]))
        return hot
