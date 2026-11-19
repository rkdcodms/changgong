"""Download the exact previously used model; never unpickle or execute it here."""
from __future__ import annotations
import argparse
import hashlib
import os
from pathlib import Path
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
URL = 'https://raw.githubusercontent.com/rkdcodms/changgong/9f9d66dfd971276d2fbb0ab79bd2d233a6898df9/best.pt'
SHA256 = 'e67df5f04399513f8578a6948843902ea31b346c558375af88e919834283bd84'
SIZE = 5507098

def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()

def download(destination):
    destination = Path(destination).expanduser().resolve()
    if destination.exists():
        if not destination.is_file():
            raise RuntimeError('Destination is not a regular model file; nothing was changed.')
        if sha256(destination) == SHA256:
            print('Verified existing model:', destination)
            return
        raise RuntimeError('Destination contains a different model. Choose a new path; nothing was overwritten.')
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=destination.parent, suffix='.download', delete=False) as stream:
            temporary = Path(stream.name)
            request = urllib.request.Request(URL, headers={'User-Agent': 'AutoRobot-model-downloader/1'})
            size = 0
            with urllib.request.urlopen(request, timeout=30) as response:
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    size += len(chunk)
                    if size > SIZE:
                        raise RuntimeError('Model exceeds the expected size; refusing this download.')
                    stream.write(chunk)
        if size != SIZE or sha256(temporary) != SHA256:
            raise RuntimeError('Model size or SHA-256 mismatch; nothing was installed.')
        # Both paths are on the same filesystem. Publish a fully verified file
        # atomically; fail rather than replace a concurrently created destination.
        os.link(temporary, destination)
        print('Downloaded and SHA-256 verified:', destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'models/best.pt')
    args = parser.parse_args()
    try:
        download(args.output)
    except (OSError, RuntimeError) as exc:
        parser.exit(1, f'Model download failed: {exc}\n')

if __name__ == '__main__':
    main()
