import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_PKG_ROOT = REPO_ROOT / "ros2_ws" / "src" / "space_rover_autonomy"

if str(ROS2_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(ROS2_PKG_ROOT))

from space_rover_autonomy.visualization import bar_chart, heatmap, overlay_path, series


class PublicationFigureExportTest(unittest.TestCase):
    def test_publication_plots_export_png_and_pdf(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            heatmap_path = temp_path / "heatmap.png"
            path_path = temp_path / "path.png"
            series_path = temp_path / "series.png"
            bar_path = temp_path / "bar.png"

            field = np.arange(25, dtype=float).reshape(5, 5)
            labels = ["nominal", "rugged", "delay"]
            values = [0.82, 0.64, 0.55]

            heatmap(field, "Risk Heatmap", heatmap_path)
            overlay_path(field, [(0, 0), (1, 1), (2, 2), (4, 4)], "Path Overlay", path_path)
            series([0.2, 0.4, 0.35, 0.6], "Training Return", "Return", "Episode", series_path)
            bar_chart(labels, values, "Success Rate", "Rate", bar_path)

            for png_path in [heatmap_path, path_path, series_path, bar_path]:
                self.assertTrue(png_path.exists())
                self.assertTrue(png_path.with_suffix(".pdf").exists())


if __name__ == "__main__":
    unittest.main()
