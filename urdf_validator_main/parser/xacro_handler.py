from __future__ import annotations

import os
import tempfile


def preprocess(path: str) -> str:
    """Process a .xacro file and write the result to a temporary URDF file.

    The temp file is created alongside the source file so that package://
    mesh paths resolve correctly from the same directory. The caller is
    responsible for deleting the returned path when done.

    Raises ImportError if the 'xacro' package is not installed.
    Raises RuntimeError if xacro processing fails.
    """
    try:
        import xacro
    except ImportError:
        raise ImportError(
            "The 'xacro' package is required to process .xacro files. "
            "Install it with: pip install 'urdf-validator[xacro]'"
        )

    try:
        doc = xacro.process_file(path)
    except Exception as exc:
        raise RuntimeError(
            f"xacro preprocessing failed for '{os.path.basename(path)}': {exc}"
        ) from exc

    xacro_dir = os.path.dirname(os.path.abspath(path))
    stem = os.path.splitext(os.path.basename(path))[0]

    tmp = tempfile.NamedTemporaryFile(
        prefix=f"{stem}_processed_",
        suffix=".urdf",
        dir=xacro_dir,
        delete=False,
        mode="w",
        encoding="utf-8",
    )
    try:
        tmp.write(doc.toprettyxml(indent="  "))
    finally:
        tmp.close()

    return tmp.name
