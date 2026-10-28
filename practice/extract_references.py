"""Render the three source pages without changing their content.

python3 practice/extract_references.py '/path/to/FYSM 1. Практика.pdf'
Requires PyMuPDF and Pillow only when regenerating these reference illustrations.
The ordinary HTML build reads the prepared WebP files and needs only Python 3.
"""
import hashlib
import io
import json
from pathlib import Path
import sys

import fitz
from PIL import Image

root = Path(__file__).resolve().parent
manifest = json.loads((root / 'algorithms.json').read_text())
source = Path(sys.argv[1])
if hashlib.sha256(source.read_bytes()).hexdigest() != manifest['source']['sha256']:
    raise SystemExit('The supplied PDF does not match practice/algorithms.json')
document = fitz.open(source)
for algorithm in manifest['algorithms']:
    pixmap = document[algorithm['page'] - 1].get_pixmap(matrix=fitz.Matrix(2, 2), alpha=False)
    illustration = Image.open(io.BytesIO(pixmap.tobytes('png')))
    target = root / ('reference-' + algorithm['id'] + '.webp')
    illustration.save(target, 'WEBP', quality=85, method=6)
    print(target.name, target.stat().st_size, 'bytes')
