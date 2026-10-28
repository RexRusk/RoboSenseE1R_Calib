"""Same entry as zhang_calib.py with k3 fixed at 0. Kept so older commands still run."""

import runpy
import sys
from pathlib import Path

script = Path(__file__).with_name("zhang_calib.py")
sys.argv = [str(script), "--fix-k3", *sys.argv[1:]]
runpy.run_path(str(script), run_name="__main__")
