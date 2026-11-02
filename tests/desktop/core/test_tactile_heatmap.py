import io

import numpy as np
from PIL import Image

from desktop.core.tactile_heatmap import colormap, grid_coords, render_png


def _image(kind, values):
    return np.asarray(Image.open(io.BytesIO(render_png(kind, values))))


def test_grid_vertices_follow_each_sensor_outline():
    from desktop.core.tactile_heatmap import _points

    pad, tip = grid_coords('pad'), grid_coords('tip')
    assert len(pad) == 11 and all(len(row) == 11 for row in pad)
    assert len(tip) == 24 and all(len(row) == 16 for row in tip)
    for kind, grid in (('pad', pad), ('tip', tip)):
        points = _points(kind)
        valid = [point for row in grid for point in row if point is not None]
        assert len(valid) >= 0.75 * len(grid) * len(grid[0])
        # 贴片要盖过测点包围盒，否则边缘传感器露在外面。
        assert min(p[0] for p in valid) < points[:, 0].min()
        assert max(p[0] for p in valid) > points[:, 0].max()
        assert min(p[1] for p in valid) < points[:, 1].min()
        assert max(p[1] for p in valid) > points[:, 1].max()
def test_rest_state_is_a_transparent_plate_without_hot_colours():
    image = _image('tip', np.zeros(52))
    assert image.shape[2] == 4
    assert image[..., 3].min() == 0 and image[..., 3].max() > 0
    lit = image[image[..., 3] > 0]
    assert lit[:, 0].max() < 90  # 深灰垫片，不是热力色


def test_pressed_taxel_renders_a_hot_spot():
    values = np.zeros(52)
    values[26] = 220.0
    image = _image('tip', values)
    hot = image[image[..., 3] > 200]
    assert len(hot) > 0
    assert hot[:, 0].max() > 90 and hot[:, 2].min() < hot[:, 0].max()
    assert colormap(np.array([0.0, 220.0]))[1][0] > colormap(np.array([0.0, 220.0]))[0][0]
