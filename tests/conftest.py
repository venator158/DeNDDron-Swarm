import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
for pkg in ("agent", "common", "ship"):
    sys.path.insert(0, str(SRC / pkg))
