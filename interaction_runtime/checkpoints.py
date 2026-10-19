"""Atomic checkpoint allocation: repeated acquisition never overwrites pixels."""
from pathlib import Path

def allocate(root,label):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    for ix in range(10000):
        path=root/(label if ix==0 else '%s_%03d'%(label,ix))
        try:path.mkdir();return path
        except FileExistsError:continue
    raise RuntimeError('Checkpoint namespace exhausted')
