"""Isolated Character Studio BVH-to-Max conversion; no running scene is used."""
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

from mocap.blend_support import cleanup, partial


def executable():
    configured = os.environ.get('RGB_MOCAP_3DSMAX_BATCH')
    if configured:
        path = Path(configured)
        if path.name.lower() != '3dsmaxbatch.exe' or not path.is_file():
            raise ValueError('RGB_MOCAP_3DSMAX_BATCH must point to an existing 3dsmaxbatch.exe.')
        return path.resolve()
    root = Path(os.environ.get('ProgramFiles', 'C:/Program Files')) / 'Autodesk'
    for folder in sorted(root.glob('3ds Max 20[0-9][0-9]'), reverse=True):
        path = folder / '3dsmaxbatch.exe'
        if path.is_file():
            return path
    raise ValueError('Install and activate 3ds Max, or set RGB_MOCAP_3DSMAX_BATCH to its 3dsmaxbatch.exe before starting RGB MoCap.')


def staging(target, request):
    target = Path(target)
    return partial(target.with_name(target.name + '-max-work'), request)


def prepare_bvh(source, target, progress=lambda: None):
    """Scale the existing Biped BVH's metre offsets/translations to centimetres."""
    with Path(source).open('rb') as inp, Path(target).open('xb') as out:
        for _ in range(4096):
            line = inp.readline(65537)
            if not line or len(line) > 65536:
                raise ValueError('Invalid Biped BVH hierarchy')
            fields = line.split()
            if fields[:1] == [b'OFFSET']:
                values = [float(value) * 100 for value in fields[1:]]
                if len(values) != 3 or not all(math.isfinite(value) for value in values):
                    raise ValueError('Invalid BVH offset')
                line = line[:len(line)-len(line.lstrip())] + ('OFFSET ' + ' '.join(f'{v:.9f}' for v in values) + '\n').encode('ascii')
            out.write(line)
            if line.strip() == b'MOTION':
                break
        else:
            raise ValueError('BVH hierarchy too large')
        count_line, time_line = inp.readline(256), inp.readline(256)
        if not count_line.startswith(b'Frames:') or not time_line.startswith(b'Frame Time:'):
            raise ValueError('Invalid BVH timing')
        frames = int(count_line.split(b':')[1])
        step = float(time_line.split(b':')[1])
        if frames < 1 or not math.isfinite(step) or step <= 0:
            raise ValueError('Invalid BVH timing')
        fps = round(1 / step)
        if not 1 <= fps <= 4800 or abs(fps * step - 1) > 1e-5:
            raise ValueError('3ds Max scene export requires an integer frame rate.')
        out.write(count_line)
        out.write(time_line)
        for index in range(frames):
            line = inp.readline(65537)
            fields = line.split()
            if len(line) > 65536 or len(fields) != 159:
                raise ValueError('Invalid Biped BVH frame')
            values = [float(value) for value in fields]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('Non-finite BVH frame')
            scaled = [value * 100 for value in values[:3]]
            if not all(math.isfinite(value) for value in scaled):
                raise ValueError('BVH translation exceeds the supported range')
            fields[:3] = [f'{value:.9f}'.encode('ascii') for value in scaled]
            out.write(b' '.join(fields) + b'\n')
            if index % 128 == 0:
                progress()
        if inp.read(1):
            raise ValueError('Unexpected BVH trailing data')
    return frames, fps


def convert(source, target, request, emit=lambda *a, **k: None):
    """Replace only an owned staged BVH with a successfully saved Max scene."""
    batch = executable()
    work = staging(target, request)
    work.mkdir()
    try:
        (work / '.rgbmocap-owner').write_text(Path(request).stem, encoding='utf-8')
    except BaseException:
        work.rmdir()
        raise
    process = None
    try:
        frames, fps = prepare_bvh(source, work / 'motion.bvh')
        script = work / 'build.ms'
        shutil.copyfile(Path(__file__).with_name('max_export_scene.ms'), script)
        args = [str(batch), str(script), '-log', str(work / 'max.log'),
                '-mxsString', f'frames:{frames}', '-mxsString', f'fps:{fps}']
        started = time.monotonic()
        with (work / 'console.log').open('wb') as log:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                       creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            while True:
                emit('progress', stage='save_max', current=0, total=0,
                     message=f'Converting BVH in 3ds Max ({int(time.monotonic()-started)}s)')
                try:
                    process.wait(timeout=1)
                    break
                except subprocess.TimeoutExpired:
                    if time.monotonic() - started > 600:
                        raise RuntimeError('3ds Max conversion timed out after 10 minutes. Check its activation and batch startup.')
                    if any(path.is_file() and path.stat().st_size > 32 * 1024**2
                           for path in (work / 'console.log', work / 'max.log')):
                        raise RuntimeError('3ds Max conversion stopped because its diagnostic log exceeded 32 MiB.')
        report = work / 'result.txt'
        summary = ''
        if report.is_file():
            with report.open('r', encoding='utf-8-sig', errors='replace') as stream:
                summary = stream.read(4096)
        scene = work / 'motion.max'
        if process.returncode or summary.strip() != 'RGBMOCAP_OK' or not scene.is_file() or not scene.stat().st_size:
            raise RuntimeError('3ds Max BVH conversion failed: ' + (summary.strip() or f'no completion report (exit {process.returncode}); check 3ds Max activation.'))
        os.replace(scene, source)
    finally:
        if process is not None and process.poll() is None:
            if os.name == 'nt':
                subprocess.run(['taskkill.exe', '/PID', str(process.pid), '/T', '/F'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=15, creationflags=subprocess.CREATE_NO_WINDOW)
            else:
                process.kill()
            process.wait(timeout=15)
        cleanup(work, request)
