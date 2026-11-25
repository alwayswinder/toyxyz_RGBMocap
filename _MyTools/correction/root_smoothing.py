"""Opt-in smoothing of the pelvis compensation before contact solving."""
import os


def requested():
    return os.environ.get('RGB_MOCAP_SMOOTH_ROOT_XZ') == '1'


def smooth_horizontal_shift(shift, fps):
    from scipy.ndimage import gaussian_filter1d

    shift[:, [0, 2]] = .65 * gaussian_filter1d(
        shift[:, [0, 2]], .12 * fps, axis=0, mode='nearest'
    )
