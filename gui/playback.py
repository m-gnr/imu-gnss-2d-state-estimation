"""Playback clock: decouples simulation time from real frame time."""


class Playback:
    def __init__(self, t0, t1, speed_levels, initial_speed, start_paused):
        self.t0 = t0
        self.t1 = t1
        self.sim_t = t0
        self.speed_levels = list(speed_levels)
        try:
            self.speed_idx = self.speed_levels.index(initial_speed)
        except ValueError:
            self.speed_idx = 0
        self.playing = not start_paused

    @property
    def speed(self):
        return self.speed_levels[self.speed_idx]

    def update(self, dt_real):
        if self.playing:
            self.sim_t += dt_real * self.speed
            if self.sim_t >= self.t1:
                self.sim_t = self.t1
                self.playing = False

    def toggle(self):
        # Restart from the beginning if we are paused at the very end.
        if not self.playing and self.sim_t >= self.t1:
            self.sim_t = self.t0
        self.playing = not self.playing

    def faster(self):
        self.speed_idx = min(self.speed_idx + 1, len(self.speed_levels) - 1)

    def slower(self):
        self.speed_idx = max(self.speed_idx - 1, 0)

    def nudge(self, seconds):
        self.sim_t = min(max(self.sim_t + seconds, self.t0), self.t1)

    def scrub_to(self, sim_t):
        self.sim_t = min(max(sim_t, self.t0), self.t1)
