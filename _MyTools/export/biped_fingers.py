"""Encode SMPL-H fingers for 3ds Max Biped's BVH reader, not a generic reader.

Measured in Max 2026.3 (28.3.0.30732), with a default five-finger, three-link
Biped. For a ZXY channel matrix E, the reader makes the local Biped rotation
D @ E.T on the left and D @ K @ E @ K on the right. D is the Biped reference
rotation and K is a half turn about Z. Thumbs have a separate reference frame.
Single-axis probes and full 693-frame imports established these rules.
"""
# SPDX-License-Identifier: Apache-2.0
# Adapted from FlowHMR-Windows; see _MyTools/licenses/flowhmr_biped/NOTICE.
import numpy as np


# Native Biped reference frames, captured after a zero-rotation BVH import.
# Palm columns are forward/across/normal, expressed in the Hand node's frame.
_PALM = np.array([
    [[.9950371919, -.0995037019, -.0000002680],
     [-.0000789195, -.0007918891, .9999996833],
     [-.0995036706, -.9950368768, -.0007958119]],
    [[.9950371522, -.0995040990, .0000000002],
     [-.0000791834, -.0007918324, -.9999996834],
     [.0995040675, .9950368371, -.0007957818]],
])
_FINGER_REST = np.array([
    [[1., .0000004170, .0000000009],
     [-.0000004170, .9999996828, .0007964969],
     [-.0000000006, -.0007964969, .9999996828]],
    [[1., .0000000888, .0000005670],
     [-.0000000884, .9999996840, -.0007950368],
     [-.0000005671, .0007950368, .9999996840]],
])
_THUMB_REST = np.array([
    [[.9553364059, -.0000002623, .2955204757],
     [.2389428645, .5884291694, -.7724360298],
     [-.1738926654, .8085487694, .5621478705]],
    [[.9553364850, .0000001984, -.2955202199],
     [.2389424438, .5884289597, .7724363197],
     [.1738928088, -.8085489221, .5621476067]],
])
_RIGHT_AXES = np.diag((-1., -1., 1.))
# World Hand orientation from a zero-rotation BVH, in the BVH's Y-up frame.
# The reader retains this extra reference rotation even when wrist channels
# are zero. Leaving it uncompensated turns a straight wrist nearly 90 degrees.
_WRIST_ZERO = np.array([
    [[.000000268, -.999999683, .000795812],
     [-1., -.000000268, .000000090],
     [-.000000090, -.000795812, -.999999683]],
    [[0., .999999683, .000795782],
     [-1., 0., -.000000476],
     [-.000000476, -.000795782, .999999683]],
])


def geometry(lines):
    """Read the already validated BVH rest hierarchy, including terminal tips."""
    names, parents, rest, ends = [], [], [], []
    stack = []
    pending = None
    for line in lines:
        tokens = line.split()
        if tokens[:1] in ([b'ROOT'], [b'JOINT']):
            pending = len(names)
            names.append(tokens[1].decode('ascii'))
            parents.append(stack[-1] if stack else -1)
            rest.append(None)
            ends.append(np.zeros(3))
        elif tokens == [b'End', b'Site']:
            pending = None
        elif tokens == [b'{']:
            stack.append(pending)
        elif tokens == [b'}']:
            stack.pop()
        elif tokens[:1] == [b'OFFSET']:
            offset = np.asarray(tokens[1:], dtype=float)
            if stack[-1] is None:
                ends[stack[-2]] = offset
            else:
                joint = stack[-1]
                parent = parents[joint]
                rest[joint] = offset if parent < 0 else rest[parent] + offset
    lookup = {name: index for index, name in enumerate(names)}
    for side in ('Left', 'Right'):
        for digit in ('Thumb', 'Index', 'Middle', 'Ring', 'Pinky'):
            for segment in range(1, 4):
                name = f'{side}Hand{digit}{segment}'
                expected = side + 'Hand' if segment == 1 else f'{side}Hand{digit}{segment-1}'
                if names[parents[lookup[name]]] != expected:
                    raise ValueError('Invalid Biped hand hierarchy: ' + name)
    return names, np.asarray(rest), np.asarray(ends)


def _unit(vector):
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-7:
        raise ValueError('Degenerate hand reference for Biped BVH export')
    return vector / length


def _swing_frame(reference, direction):
    """Align the bone's X axis with the segment without inventing axial twist."""
    x = _unit(direction)
    original = reference[:, 0]
    cosine = float(np.clip(np.dot(original, x), -1., 1.))
    if cosine < -1. + 1e-7:
        y = reference[:, 1]
        turn = 2 * np.outer(y, y) - np.eye(3)
    else:
        a, b, c = np.cross(original, x)
        cross = np.array(((0., -c, b), (c, 0., -a), (-b, a, 0.)))
        turn = np.eye(3) + cross + cross @ cross / (1. + cosine)
    return turn @ reference


def encode_fingers(rotations, joints, ends, names):
    """Return Biped hand channels, including the wrists' reference correction.

Resolve the three-link fingers by anatomical name, independently of BVH order.
Only rest geometry is used for calibration; the motion's first pose is never
treated as its rest pose. Apply this after adding any leading reference frame.
"""
    encoded = rotations.copy()
    lookup = {name: joint for joint, name in enumerate(names)}
    for side, label in enumerate(("Left", "Right")):
        wrist = lookup[label + "Hand"]
        forward = _unit(joints[lookup[label + "HandMiddle1"]] - joints[wrist])
        across = joints[lookup[label + "HandIndex1"]] - joints[lookup[label + "HandPinky1"]]
        across = _unit(across - forward * np.dot(forward, across))
        source_palm = np.stack((forward, across, np.cross(forward, across)), axis=1)
        hand_frame = _PALM[side] @ source_palm.T
        encoded[:, wrist] = rotations[:, wrist] @ source_palm @ _PALM[side].T @ _WRIST_ZERO[side].T
        for finger, digit in enumerate(("Index", "Middle", "Pinky", "Ring", "Thumb")):
            parent_frame = hand_frame.T
            rest_base = _THUMB_REST[side] if finger == 4 else _FINGER_REST[side]
            # Carry Biped's reference roll into the source hand. An arbitrary
            # world-up guide can add a 180-degree twist while passing every
            # segment-direction check, and then twists a skinned finger.
            reference = source_palm @ _PALM[side].T @ rest_base
            for segment in range(3):
                joint = lookup[f"{label}Hand{digit}{segment + 1}"]
                direction = joints[lookup[f"{label}Hand{digit}{segment + 2}"]] - joints[joint] if segment < 2 else ends[joint]
                bone_frame = _swing_frame(reference, direction)
                desired = parent_frame.T @ rotations[:, joint] @ bone_frame
                rest = rest_base if segment == 0 else np.eye(3)
                if side == 0:
                    encoded[:, joint] = desired.swapaxes(-1, -2) @ rest
                else:
                    encoded[:, joint] = _RIGHT_AXES @ rest.T @ desired @ _RIGHT_AXES
                parent_frame = bone_frame
    return encoded
