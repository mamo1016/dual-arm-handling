"""
dual_urdf.py — build a two-arm URDF from the single-arm OpenArm description.

The single-arm URDF hardcodes ``openarm_*`` for every link/joint and pins the
robot to ``openarm_link0``. To put two arms in one scene we prefix every link/
joint NAME reference (``arm_a_`` and ``arm_b_``), then splice both robots under a
shared ``world`` link with a fixed joint placing each arm's base.

Critical: only NAME references are rewritten — never mesh paths. A blind
``openarm_`` → prefix replace would corrupt ``filename="package://openarm_description/…"``
and break mesh loading. We only touch the ``name=``, ``link=`` and ``joint=``
attributes (the latter covers ``<mimic>`` and transmission joint references).
"""
import re


# Matches name/link/joint attributes that reference an openarm_* element, e.g.
#   name="openarm_link0"   parent link="openarm_hand"   <mimic joint="openarm_finger_joint1"/>
# but NOT filename="package://openarm_description/...".
_NAME_REF = re.compile(r'(name|link|joint)="openarm_')


def _prefix_body(body: str, prefix: str) -> str:
    """Prefix every openarm_* name reference in a robot body fragment."""
    return _NAME_REF.sub(rf'\1="{prefix}openarm_', body)


def _extract_body(single_urdf_xml: str) -> str:
    """Return the inner content of the single-arm <robot>…</robot> element."""
    open_match = re.search(r'<robot\b[^>]*>', single_urdf_xml)
    if not open_match:
        raise ValueError('No <robot> element found in single-arm URDF')
    close_idx = single_urdf_xml.rfind('</robot>')
    if close_idx == -1:
        raise ValueError('No closing </robot> tag found in single-arm URDF')
    return single_urdf_xml[open_match.end():close_idx]


def _base_joint(prefix: str, xyz, rpy) -> str:
    """Fixed joint attaching an arm's link0 to the shared world frame."""
    return (
        f'  <joint name="world_to_{prefix}base" type="fixed">\n'
        f'    <parent link="world"/>\n'
        f'    <child link="{prefix}openarm_link0"/>\n'
        f'    <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" '
        f'rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>\n'
        f'  </joint>\n'
    )


def build_dual_urdf(single_urdf_path,
                    arm_a_xyz=(0.0, 0.0, 0.0), arm_a_rpy=(0.0, 0.0, 0.0),
                    arm_b_xyz=(-0.8, 0.0, 0.0), arm_b_rpy=(0.0, 0.0, 3.14159)):
    """Read the single-arm URDF and return a combined two-arm URDF string.

    arm_a is placed at the world origin; arm_b faces it across the shared
    workspace (rotated 180° about Z by default). Both bases are tunable via the
    xyz/rpy arguments so the reach envelopes can be aligned with the scene.
    """
    with open(single_urdf_path, 'r') as f:
        single = f.read()

    body = _extract_body(single)
    body_a = _prefix_body(body, 'arm_a_')
    body_b = _prefix_body(body, 'arm_b_')

    return (
        '<?xml version="1.0" ?>\n'
        '<robot name="dual_openarm">\n'
        '  <link name="world"/>\n'
        + _base_joint('arm_a_', arm_a_xyz, arm_a_rpy)
        + body_a
        + _base_joint('arm_b_', arm_b_xyz, arm_b_rpy)
        + body_b
        + '</robot>\n'
    )
