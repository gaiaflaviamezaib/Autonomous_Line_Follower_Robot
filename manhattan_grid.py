import subprocess
import sys
from pathlib import Path


script = Path(__file__).with_name("MAIN_PID_INTERSECTION_MANHATTAN.py")

raise SystemExit(
    subprocess.call([sys.executable, str(script), "--mode", "grid", "--preset", "balanced"])
)
