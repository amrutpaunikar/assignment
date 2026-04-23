import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_csv(rows: List[Dict], csv_path: Path) -> None:
    if not rows:
        return
    try:
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    except PermissionError:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fallback_path = csv_path.with_name(f"{csv_path.stem}_{timestamp}{csv_path.suffix}")
        with open(fallback_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(
            f"Warning: could not write to '{csv_path}' (possibly open in another app). "
            f"Wrote to '{fallback_path}' instead."
        )
