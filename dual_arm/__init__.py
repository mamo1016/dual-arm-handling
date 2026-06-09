"""
Make Python dependencies in a repo-local ``.venv`` importable to ROS 2 nodes.

ROS 2 launches nodes under the system interpreter, which often cannot see the
roboticstoolbox / spatialmath packages this demo needs. If a ``.venv`` exists at
the repo root, prepend its site-packages so the nodes can import them. This is a
no-op when there is no venv — in that case the dependencies must be importable
some other way (a global ``pip install`` or ``PYTHONPATH``). See the README.
"""
import glob
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
for _sp in glob.glob(
        os.path.join(_here, os.pardir, '.venv', 'lib', 'python*', 'site-packages')):
    if os.path.isdir(_sp) and _sp not in sys.path:
        sys.path.insert(0, _sp)
