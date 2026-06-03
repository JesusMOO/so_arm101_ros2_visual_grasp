from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_BASE_LINK = "base_link"
DEFAULT_END_EFFECTOR_LINK = "gripper_frame_link"
ACTIVE_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
DEFAULT_URDF_PATH = None


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: tuple
    origin_rpy: tuple
    axis: tuple
    lower: float | None = None
    upper: float | None = None

    @property
    def is_active(self):
        return self.joint_type in {"revolute", "continuous"}


@dataclass(frozen=True)
class SO101Model:
    joints: list
    base_link: str = DEFAULT_BASE_LINK
    end_effector_link: str = DEFAULT_END_EFFECTOR_LINK

    @property
    def active_joint_names(self):
        return [
            joint.name
            for joint in self.joints
            if joint.name in ACTIVE_JOINT_NAMES
        ]

    def get_joint(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise KeyError(name)

    def joint_limits(self):
        return {
            joint.name: (joint.lower, joint.upper)
            for joint in self.joints
            if joint.name in ACTIVE_JOINT_NAMES
        }


def load_so101_model(path=None):
    path = Path(path) if path is not None else default_urdf_path()
    root = ET.parse(path).getroot()
    all_joints = [_parse_joint(element) for element in root.findall("joint")]
    ordered_joints = _order_joints_from_base(
        all_joints,
        DEFAULT_BASE_LINK,
    )
    return SO101Model(joints=ordered_joints)


def default_urdf_path():
    source_path = ROOT_DIR / "urdf" / "so101" / "so101_new_calib.urdf"
    if source_path.exists():
        return source_path

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_path

    return (
        Path(get_package_share_directory("lerobot"))
        / "urdf"
        / "so101"
        / "so101_new_calib.urdf"
    )


def _order_joints_from_base(joints, base_link):
    children_by_parent = {}
    for joint in joints:
        children_by_parent.setdefault(joint.parent, []).append(joint)

    ordered = []
    queue = [base_link]
    while queue:
        parent = queue.pop(0)
        for joint in children_by_parent.get(parent, []):
            ordered.append(joint)
            queue.append(joint.child)
    return ordered


def _parse_joint(element):
    origin = element.find("origin")
    axis = element.find("axis")
    limit = element.find("limit")

    lower = None
    upper = None
    if limit is not None:
        lower = float(limit.attrib["lower"]) if "lower" in limit.attrib else None
        upper = float(limit.attrib["upper"]) if "upper" in limit.attrib else None

    return JointSpec(
        name=element.attrib["name"],
        joint_type=element.attrib["type"],
        parent=element.find("parent").attrib["link"],
        child=element.find("child").attrib["link"],
        origin_xyz=_parse_vector(origin, "xyz", default=(0.0, 0.0, 0.0)),
        origin_rpy=_parse_vector(origin, "rpy", default=(0.0, 0.0, 0.0)),
        axis=_parse_vector(axis, "xyz", default=(0.0, 0.0, 0.0)),
        lower=lower,
        upper=upper,
    )


def _parse_vector(element, attr, default):
    if element is None or attr not in element.attrib:
        return default
    return tuple(float(value) for value in element.attrib[attr].split())
