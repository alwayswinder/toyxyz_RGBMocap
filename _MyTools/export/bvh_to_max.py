"""Convert an RGB MoCap 52-joint BVH to an animated 3ds Max Biped scene."""
import argparse
import os
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='RGB MoCap BVH (metres, Y up)')
    parser.add_argument('output', type=Path, help='Destination .max file')
    parser.add_argument('--profile', choices=('generic', 'biped'), default='generic',
                        help='Profile used when the input BVH was exported')
    args = parser.parse_args()
    from mocap.biped_bvh import convert as biped
    from .max_export import convert, executable
    from mocap.result_artifacts import file_stat
    source, target = args.input.resolve(), args.output.resolve()
    if source.suffix.lower() != '.bvh' or target.suffix.lower() != '.max' or source == target:
        parser.error('Choose an input .bvh and an output .max file.')
    executable()
    stamp = file_stat(source)
    with TemporaryDirectory(prefix='.rgbmocap-bvh-max-', dir=target.parent) as directory:
        work = Path(directory)
        staged = work / 'source.bvh'
        with source.open('rb') as inp, staged.open('xb') as out:
            if args.profile == 'generic':
                biped(inp, out)
            else:
                shutil.copyfileobj(inp, out, 1024 * 1024)
        def progress(kind, **event):
            print(event.get('message', kind), flush=True)
        convert(staged, target, work / 'request.json', progress)
        if file_stat(source) != stamp:
            raise ValueError('Input BVH changed during conversion. Output was not replaced.')
        os.replace(staged, target)
    print('Exported: ' + str(target))
