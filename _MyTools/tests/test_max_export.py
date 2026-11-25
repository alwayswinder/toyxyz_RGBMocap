"""BVH/Max export units, publication and failure regression checks."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from _MyTools.export import max_export
from mocap import export_copy
from mocap.biped_bvh import bone_names


def sample():
    names=list(bone_names())
    parents={'Hips':None,'Spine':'Hips','Spine1':'Spine','Spine2':'Spine1','Neck':'Spine2','Head':'Neck'}
    offsets={}
    for side,sign in (('Left',1),('Right',-1)):
        for child,parent in (('UpLeg',''),('Leg','UpLeg'),('Foot','Leg'),('ToeBase','Foot'),
                             ('Shoulder','Spine2'),('Arm','Shoulder'),('ForeArm','Arm'),('Hand','ForeArm')):
            parents[side+child]=side+parent if parent and parent!='Spine2' else parent or 'Hips'
        for digit,(x,y,z) in zip(('Thumb','Index','Middle','Ring','Pinky'),
                                 ((.02,-.02,-.04),(.05,0,-.02),(.06,0,0),(.05,0,.013),(.04,0,.025))):
            for segment in range(1,4):
                name=f'{side}Hand{digit}{segment}'
                parents[name]=side+'Hand' if segment==1 else f'{side}Hand{digit}{segment-1}'
                offsets[name]=(sign*x,y,z) if segment==1 else (sign*.025,0,0)
    lines=['HIERARCHY']
    def visit(name):
        lines.extend([('ROOT ' if name=='Hips' else 'JOINT ')+name,'{'])
        offset=offsets.get(name,(0,.9,0) if name=='Hips' else (.1,.2,-.3))
        lines.append(' OFFSET '+' '.join(map(str,offset)))
        lines.append(' CHANNELS '+('6 Xposition Yposition Zposition ' if name=='Hips' else '3 ')+'Zrotation Xrotation Yrotation')
        children=[child for child in names if parents[child]==name]
        for child in children:visit(child)
        if not children:lines.extend(['End Site','{','OFFSET 0.02 0 0','}'])
        lines.append('}')
    visit('Hips')
    text='\n'.join(lines)+'\nMOTION\nFrames: 2\nFrame Time: 0.033333333\n'
    for xyz in ('1 2 3', '1.5 2.1 2.5'):
        text += xyz + ' ' + ' '.join(['12.25 -3.5 7.75'] * 52) + '\n'
    return text.encode('ascii')


class MaxExportTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.folder = Path(temporary.name)
        self.source = self.folder / 'source.bvh'
        self.source.write_bytes(sample())
        self.target = self.folder / '动作.max'
        self.target.write_bytes(b'previous scene')
        self.request = self.folder / 'request.json'

    def rows(self, **options):
        self.request.write_text(json.dumps([dict(project=str(self.folder), kind='initial',
            source=str(self.source), target=str(self.target), **options)]), encoding='utf-8')

    def export(self, **options):
        self.rows(**options)
        with patch.object(export_copy, 'selected_export', return_value=self.source), \
             patch.object(export_copy, 'is_project', return_value=False), \
             patch.object(max_export, 'executable', return_value=Path('3dsmaxbatch.exe')):
            return export_copy.run(self.request, max_scene=True)

    def test_centimetres_change_only_offsets_and_root_translation(self):
        output = self.folder / 'scaled.bvh'
        self.assertEqual(max_export.prepare_bvh(self.source, output), (2, 30))
        before, after = sample().split(b'MOTION'), output.read_bytes().split(b'MOTION')
        original_offsets = [line.split()[1:] for line in before[0].splitlines() if line.strip().startswith(b'OFFSET')]
        scaled_offsets = [line.split()[1:] for line in after[0].splitlines() if line.strip().startswith(b'OFFSET')]
        self.assertEqual(len(original_offsets), len(scaled_offsets))
        for left, right in zip(original_offsets, scaled_offsets):
            for a, b in zip(left, right):self.assertAlmostEqual(float(b), float(a) * 100)
        original, scaled = before[1].splitlines(), after[1].splitlines()
        self.assertEqual(original[:3], scaled[:3])
        for a, b in zip(original[3:], scaled[3:]):
            self.assertEqual(a.split()[3:], b.split()[3:])
            for x, y in zip(a.split()[:3], b.split()[:3]):self.assertAlmostEqual(float(y), float(x) * 100)
        self.assertEqual(self.source.read_bytes(), sample())

    def test_reject_fractional_frame_rate(self):
        self.source.write_bytes(sample().replace(b'0.033333333', b'0.041708375'))
        with self.assertRaisesRegex(ValueError, 'integer frame rate'):
            max_export.prepare_bvh(self.source, self.folder / 'bad.bvh')

    def test_max_conversion_publishes_and_keeps_source(self):
        def convert(staged, target, request, emit):
            self.assertIn(b'JOINT Chest', staged.read_bytes())
            self.assertEqual(target, self.target)
            staged.write_bytes(b'new scene')
        with patch.object(max_export, 'convert', side_effect=convert):
            self.assertEqual(self.export(), 1)
        self.assertEqual(self.target.read_bytes(), b'new scene')
        self.assertEqual(self.source.read_bytes(), sample())
        self.assertFalse(self.request.exists())
        self.assertFalse(list(self.folder.glob('*.partial')))

    def test_failure_and_cancel_retain_existing_destination(self):
        for error in (RuntimeError('Max failed'), KeyboardInterrupt()):
            with self.subTest(error=type(error)), patch.object(max_export, 'convert', side_effect=error):
                with self.assertRaises(type(error)):
                    self.export()
            self.assertEqual(self.target.read_bytes(), b'previous scene')
            self.assertEqual(self.source.read_bytes(), sample())
            self.assertFalse(self.request.exists())
            self.assertFalse(list(self.folder.glob('*.partial')))

    def test_generation_change_during_max_blocks_publication(self):
        def change(staged, *args):
            staged.write_bytes(b'new scene')
            self.source.write_bytes(sample() + b'changed')
        with patch.object(max_export, 'convert', side_effect=change):
            with self.assertRaisesRegex(ValueError, 'changed during export'):
                self.export()
        self.assertEqual(self.target.read_bytes(), b'previous scene')

    def test_missing_max_retains_existing_destination(self):
        self.rows()
        with patch.object(max_export, 'executable', side_effect=ValueError('Max unavailable')):
            with self.assertRaisesRegex(ValueError, 'Max unavailable'):
                export_copy.run(self.request, max_scene=True)
        self.assertEqual(self.target.read_bytes(), b'previous scene')
        self.assertFalse(self.request.exists())

    def test_later_take_failure_does_not_publish_earlier_take(self):
        other = self.folder / 'second.max'
        other.write_bytes(b'second previous scene')
        self.rows()
        rows = json.loads(self.request.read_text(encoding='utf-8'))
        rows.append(dict(rows[0], target=str(other)))
        self.request.write_text(json.dumps(rows), encoding='utf-8')
        def convert(staged, target, *args):
            if target == other:raise RuntimeError('Second take failed')
            staged.write_bytes(b'first converted scene')
        with patch.object(export_copy, 'selected_export', return_value=self.source), \
             patch.object(export_copy, 'is_project', return_value=False), \
             patch.object(max_export, 'executable', return_value=Path('3dsmaxbatch.exe')), \
             patch.object(max_export, 'convert', side_effect=convert):
            with self.assertRaisesRegex(RuntimeError, 'Second take failed'):
                export_copy.run(self.request, max_scene=True)
        self.assertEqual(self.target.read_bytes(), b'previous scene')
        self.assertEqual(other.read_bytes(), b'second previous scene')
        self.assertFalse(list(self.folder.glob('*.partial')))

    def test_batch_failure_cleans_only_owned_staging(self):
        work = max_export.staging(self.target, self.request)
        def fail(*args, **kwargs):
            raise RuntimeError('Batch startup failed')
        with patch.object(max_export, 'executable', return_value=Path('3dsmaxbatch.exe')), \
             patch.object(max_export.subprocess, 'Popen', side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, 'Batch startup failed'):
                max_export.convert(self.source, self.target, self.request)
        self.assertFalse(work.exists())
        self.assertEqual(self.target.read_bytes(), b'previous scene')
        self.assertEqual(self.source.read_bytes(), sample())

    def test_unowned_existing_staging_is_preserved(self):
        work = max_export.staging(self.target, self.request)
        work.mkdir()
        unrelated = work / 'user-file.txt'
        unrelated.write_text('keep', encoding='utf-8')
        with patch.object(max_export, 'executable', return_value=Path('3dsmaxbatch.exe')):
            with self.assertRaises(FileExistsError):
                max_export.convert(self.source, self.target, self.request)
        self.assertEqual(unrelated.read_text(encoding='utf-8'), 'keep')

    def test_generic_bvh_copy_stays_byte_identical(self):
        self.target = self.folder / 'copy.bvh'
        self.rows()
        with patch.object(export_copy, 'selected_export', return_value=self.source), \
             patch.object(export_copy, 'is_project', return_value=False):
            export_copy.run(self.request)
        self.assertEqual(self.target.read_bytes(), sample())

    def test_nonstandard_install_path_must_be_valid(self):
        with patch.dict('os.environ', RGB_MOCAP_3DSMAX_BATCH=str(self.folder / 'missing.exe')):
            with self.assertRaisesRegex(ValueError, 'existing 3dsmaxbatch.exe'):
                max_export.executable()


if __name__ == '__main__':
    unittest.main()
