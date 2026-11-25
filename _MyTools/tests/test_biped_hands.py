"""Measured native Max importer regressions, adapted from FlowHMR-Windows."""
# SPDX-License-Identifier: Apache-2.0
import io,json,unittest
from pathlib import Path
from tempfile import TemporaryDirectory
import numpy as np
from scipy.spatial.transform import Rotation
from _MyTools.export.biped_fingers import encode_fingers
from mocap.biped_bvh import convert
from mocap.export import JOINT_NAMES,PARENTS
CANONICAL=JOINT_NAMES+[side+'Hand'+digit+str(segment) for side in ('Left','Right') for digit in ('Index','Middle','Pinky','Ring','Thumb') for segment in (1,2,3)]

def read_bvh(text):
    tokens = iter(text.split())
    names, parents, offsets = [], [], []
    assert next(tokens) == 'HIERARCHY'

    def joint(parent):
        names.append(next(tokens))
        current = len(names) - 1
        parents.append(parent)
        assert next(tokens) == '{'
        assert next(tokens) == 'OFFSET'
        offsets.append([float(next(tokens)) for _ in range(3)])
        assert next(tokens) == 'CHANNELS'
        count = int(next(tokens))
        channels = [next(tokens) for _ in range(count)]
        assert channels == (['Xposition', 'Yposition', 'Zposition'] if parent == -1 else []) + ['Zrotation', 'Xrotation', 'Yrotation']
        while True:
            token = next(tokens)
            if token == '}':
                return
            if token == 'JOINT':
                joint(current)
            else:
                assert token == 'End' and next(tokens) == 'Site' and next(tokens) == '{'
                assert next(tokens) == 'OFFSET'
                assert all(np.isfinite(float(next(tokens))) for _ in range(3))
                assert next(tokens) == '}'

    assert next(tokens) == 'ROOT'
    joint(-1)
    assert next(tokens) == 'MOTION' and next(tokens) == 'Frames:'
    frames = int(next(tokens))
    assert next(tokens) == 'Frame' and next(tokens) == 'Time:'
    step = float(next(tokens))
    samples = np.asarray([float(value) for value in tokens]).reshape(frames, 3 + 3 * len(names))
    local = Rotation.from_euler('ZXY', samples[:, 3:].reshape(-1, 3), degrees=True).as_matrix().reshape(frames, len(names), 3, 3)
    world = local.copy()
    positions = np.zeros((frames, len(names), 3))
    for j, parent in enumerate(parents):
        if parent < 0:
            positions[:, j] = samples[:, :3] + offsets[j]
        else:
            positions[:, j] = positions[:, parent] + world[:, parent] @ offsets[j]
            world[:, j] = world[:, parent] @ local[:, j]
    return names, samples, step, local, positions


def synthetic_body():
    parents = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19]
    for wrist in (20, 21):
        for _ in range(5):
            first = len(parents)
            parents.extend((wrist, first, first + 1))
    parents = np.asarray(parents)
    rng = np.random.default_rng(53)
    offsets = rng.normal(size=(52, 3)) * .1
    offsets[0] = (0, .9, 0)
    joints = offsets.copy()
    for j in range(1, 52):
        joints[j] += joints[parents[j]]
    vertices = np.vstack((joints, (0, 5, 0)))
    weights = np.eye(53)[:, :52]
    weights[-1, 15] = 1.
    return dict(v_template=vertices, shapedirs=np.zeros((53, 3, 10)),
                J_regressor=np.eye(53)[:52], weights=weights,
                kintree_table=np.stack((parents, np.arange(52)))), joints, parents



def export_bvh(motion,path,joints,parents):
    rotations=np.asarray(motion['poses']).reshape(-1,52,3)
    order=[]; lines=['HIERARCHY']
    def visit(j):
        order.append(j)
        lines.extend([('ROOT ' if j==0 else 'JOINT ')+CANONICAL[j],'{'])
        offset=np.zeros(3) if j==0 else joints[j]-joints[parents[j]]
        lines.append('OFFSET '+' '.join(map(str,offset)))
        lines.append('CHANNELS '+('6 Xposition Yposition Zposition ' if j==0 else '3 ')+'Zrotation Xrotation Yrotation')
        children=np.flatnonzero(parents==j)
        for child in children:visit(child)
        if not len(children):
            tip=joints[j]-joints[parents[j]]
            lines.extend(['End Site','{','OFFSET '+' '.join(map(str,tip)),'}'])
        lines.append('}')
    visit(0)
    angles=Rotation.from_rotvec(rotations.reshape(-1,3)).as_euler('ZXY',degrees=True).reshape(-1,52,3)
    trans=np.asarray(motion['trans'])+joints[0]
    lines.extend(['MOTION',f'Frames: {len(trans)}','Frame Time: 0.033333333333'])
    for xyz,frame in zip(trans,angles):lines.append(' '.join(map(str,np.r_[xyz,frame[order].reshape(-1)])))
    output=io.BytesIO();convert(io.BytesIO(('\n'.join(lines)+'\n').encode('ascii')),output)
    Path(path).write_bytes(output.getvalue())

class BipedHandsTest(unittest.TestCase):
    def setUp(self):
        self.model,self.joints,self.parents=synthetic_body()
        temp=TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.target=Path(temp.name)/'motion.bvh'

    def test_body_and_root_are_preserved_across_streaming_blocks(self):
        from mocap.biped_bvh import bone_names
        rng=np.random.default_rng(163)
        poses=rng.normal(size=(270,52,3))
        trans=rng.normal(size=(270,3))
        export_bvh(dict(poses=poses,trans=trans),self.target,self.joints,self.parents)
        names,samples,_,local,_=read_bvh(self.target.read_text())
        source=Rotation.from_rotvec(poses.reshape(-1,3)).as_matrix().reshape(-1,52,3,3)
        mapping=bone_names()
        for j,name in enumerate(CANONICAL[:20]):
            np.testing.assert_allclose(local[:,names.index(mapping[name])],source[:,j],atol=3e-11)
        np.testing.assert_allclose(samples[:,:3],trans+self.joints[0],atol=6e-10)
    def test_all_finger_directions_against_measured_native_import_responses(self):
        # Forward simulation uses captured Max axis probes, independently of the
        # production inverse encoder. Includes both thumbs and terminal links.
        fixture = json.loads((Path(__file__).parent / 'fixtures/biped_2026_finger_probes.json').read_text())
        rng = np.random.default_rng(84)
        poses = rng.normal(size=(140, 52, 3)) * 1.5
        export_bvh(dict(poses=poses, trans=np.zeros((len(poses), 3))), self.target,self.joints,self.parents)
        names, samples, _, decoded, _ = read_bvh(self.target.read_text())
        lookup = {name: i for i, name in enumerate(names)}
        local = Rotation.from_rotvec(poses.reshape(-1, 3)).as_matrix().reshape(-1, 52, 3, 3)
        source_to_max = np.array(((1, 0, 0), (0, 0, -1), (0, 1, 0)))
        source_world_all = local.copy()
        for joint in range(1, 52):
            source_world_all[:, joint] = source_world_all[:, self.parents[joint]] @ local[:, joint]
        ends = {}
        # Terminal directions follow the same established source rig geometry.
        for joint in (24, 27, 30, 33, 36, 39, 42, 45, 48, 51):
            ends[joint] = self.joints[joint] - self.joints[self.parents[joint]]
        for side, first, wrist in (('Left', 22, 20), ('Right', 37, 21)):
            forward = self.joints[first + 3] - self.joints[wrist]
            forward /= np.linalg.norm(forward)
            across = self.joints[first] - self.joints[first + 6]
            across -= forward * np.dot(forward, across)
            across /= np.linalg.norm(across)
            source_palm = np.stack((forward, across, np.cross(forward, across)), axis=1)
            hand_reference = np.asarray(fixture['hands'][side]['rotation'])
            palm_in_hand = hand_reference.T @ np.asarray(fixture['hands'][side]['palm'])
            wrist_offset = source_to_max.T @ hand_reference
            target_hand = source_to_max @ source_world_all[:, self.parents[wrist]] @ decoded[:, lookup[side + 'Wrist']] @ wrist_offset
            expected_palm = source_to_max @ source_world_all[:, wrist] @ source_palm
            np.testing.assert_allclose(target_hand @ palm_in_hand, expected_palm, atol=8e-5, err_msg=side + ' wrist/palm')
            for finger, digit in enumerate((1, 2, 4, 3, 0)):
                source_world = source_world_all[:, wrist].copy()
                target_world = target_hand.copy()
                for segment in range(3):
                    name = side + 'Finger' + str(digit) + (str(segment) if segment else '')
                    probe = fixture['fingers'][name]
                    values = np.radians(samples[:, 3 + 3 * lookup[name]:6 + 3 * lookup[name]])
                    by_axis = dict(zip('ZXY', values.T))
                    native = np.broadcast_to(np.eye(3), source_world.shape).copy()
                    for axis in probe['order']:
                        direction = np.asarray(probe['axes'][axis])
                        direction /= np.linalg.norm(direction)
                        native = native @ Rotation.from_rotvec(by_axis[axis][:, None] * direction).as_matrix()
                    reference = np.asarray(probe['parent_rotation'])
                    target_world = target_world @ reference.T @ source_to_max @ native @ source_to_max.T @ reference @ probe['rest']
                    joint = first + 3 * finger + segment
                    source_world = source_world @ local[:, joint]
                    offset = self.joints[joint + 1] - self.joints[joint] if segment < 2 else ends[joint]
                    expected = (source_to_max @ source_world @ offset[..., None])[..., 0]
                    expected /= np.linalg.norm(expected, axis=-1, keepdims=True)
                    actual = target_world[..., 0]
                    np.testing.assert_allclose(actual, expected, atol=8e-5, err_msg=name)
                    # Reference roll is measured independently from the encoder.
                    rest_base=np.asarray(fixture['fingers'][side+'Finger'+str(digit)]['rest'])
                    reference_frame=source_palm @ palm_in_hand.T @ rest_base
                    turn=Rotation.align_vectors([offset/np.linalg.norm(offset)],[reference_frame[:,0]])[0].as_matrix()
                    bone_frame=turn @ reference_frame
                    desired_frame=source_to_max @ source_world @ bone_frame
                    np.testing.assert_allclose(target_world,desired_frame,atol=8e-5,err_msg=name+' full rotation')

    def test_native_neutral_hand_does_not_acquire_wrist_or_finger_twist(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/biped_2026_finger_probes.json').read_text())
        to_source = np.array(((1, 0, 0), (0, 0, 1), (0, -1, 0)))
        joints = np.zeros((52, 3))
        ends = {}
        for side, first, wrist in (('Left', 22, 20), ('Right', 37, 21)):
            positions = fixture['hands'][side]['positions']
            joints[wrist] = to_source @ positions[side + 'Wrist']
            for finger, digit in enumerate((1, 2, 4, 3, 0)):
                reference = np.asarray(fixture['hands'][side]['rotation'])
                for segment in range(3):
                    name = side + 'Finger' + str(digit) + (str(segment) if segment else '')
                    joint = first + 3 * finger + segment
                    joints[joint] = to_source @ positions[name]
                    reference = reference @ fixture['fingers'][name]['rest']
                    if segment == 2:
                        ends[joint] = to_source @ reference[:, 0] * .02
        identity = np.broadcast_to(np.eye(3), (1, 52, 3, 3)).copy()
        encoded = encode_fingers(identity, joints, ends, CANONICAL)
        np.testing.assert_allclose(encoded[:, 20:], identity[:, 20:], atol=1e-5)


if __name__=='__main__':unittest.main()
