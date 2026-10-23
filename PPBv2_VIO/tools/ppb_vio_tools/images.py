"""Image loading for dataset conversion (PIL + numpy)."""

import numpy as np
from PIL import Image as PILImage

from ppb_vio_tools.euroc import DatasetError


def load_image_mono8(path, calib):
    """Return (pixels uint8 flat array, ambiguous_16bit flag)."""
    with PILImage.open(path) as img:
        if img.size != (calib.width, calib.height):
            raise DatasetError(f'{path}: size {img.size} != calibration '
                               f'{(calib.width, calib.height)}')
        if img.mode == 'L':
            return np.asarray(img, dtype=np.uint8).reshape(-1), False
        if img.mode in ('I;16', 'I;16B', 'I;16L'):
            raw = np.asarray(img).astype(np.uint16)
            return (raw >> 8).astype(np.uint8).reshape(-1), int(raw.max()) < 256
        raise DatasetError(f'{path}: unsupported image mode {img.mode}')
