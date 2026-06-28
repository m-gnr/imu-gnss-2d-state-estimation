"""Entry point for the IMU-GNSS minimap GUI.

Run the simulation pipeline first so the time-series file exists:
    python main.py
Then launch the GUI:
    python run_gui.py [path/to/config.yaml]
"""

import sys

from gui.app import App


def main() -> None:
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    try:
        App(config_path).run()
    except FileNotFoundError as exc:
        print(exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
