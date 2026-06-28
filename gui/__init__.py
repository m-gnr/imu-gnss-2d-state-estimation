"""Config-driven Pygame minimap GUI for the IMU-GNSS 2D state estimation project.

The GUI is fully decoupled from the simulation pipeline: it only reads the
exported time-series file (outputs/data/timeseries.npz). Run the pipeline first
(`python main.py`), then `python run_gui.py`.
"""
