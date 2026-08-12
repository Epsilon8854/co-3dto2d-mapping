from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent.parent if PACKAGE.parent.name == "src" else PACKAGE
TEXT_SUFFIXES = {".py", ".cpp", ".hpp", ".xml", ".txt", ".yaml", ".sh"}
FORBIDDEN = (
    "/home/user/Swarm-SLAM/src",
    "/home/user/Swarm-SLAM/install",
    "cslam_common_interfaces",
    "from cslam",
    "import cslam",
)
GENERATED_DIRS = {
    ".cache",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".ros",
    "__pycache__",
    "build",
    "cache",
    "install",
    "log",
}


def runtime_files():
    runtime_roots = (
        PACKAGE / "co_3dto2d_mapping",
        PACKAGE / "launch",
        PACKAGE / "src",
        PACKAGE / "include",
        ROOT / "scripts" / "mapping",
    )
    for path in runtime_roots:
        if not path.exists():
            continue
        for candidate in path.rglob("*"):
            if candidate.is_file() and candidate.suffix in TEXT_SUFFIXES:
                yield candidate


def test_package_is_integrated_in_root_workspace():
    assert (PACKAGE / "package.xml").is_file()
    assert (PACKAGE / "CMakeLists.txt").is_file()


def test_manifest_maintainer_email_is_valid():
    manifest = ET.parse(PACKAGE / "package.xml").getroot()
    email = manifest.find("maintainer").attrib["email"]
    assert re.fullmatch(r"[^@\s]+@[^@\s]+\.[^@\s]+", email)


def test_runtime_has_no_swarm_slam_or_cslam_dependencies():
    violations = []
    for path in runtime_files():
        text = path.read_text(errors="ignore")
        for token in FORBIDDEN:
            if token in text:
                violations.append(f"{path.relative_to(ROOT)}: {token}")
    assert violations == []


def test_runtime_contains_no_symlinks():
    links = []
    for runtime_root in (PACKAGE, ROOT / "scripts" / "mapping"):
        if runtime_root.exists():
            links.extend(
                path.relative_to(ROOT)
                for path in runtime_root.rglob("*")
                if path.is_symlink()
                and not any(
                    part in GENERATED_DIRS
                    for part in path.relative_to(ROOT).parts
                )
            )
    assert links == []
