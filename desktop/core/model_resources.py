"""原始 URDF 资源定位。

职责：按 URDF 所在位置解析相对路径和 package:// 资源。
线程：调用者线程执行，不创建线程。
输入：URDF 路径与资源引用。
输出：已验证存在的资源文件路径。
状态所有权：无共享可写状态。
禁止依赖：禁止依赖 Qt、串口或设备控制。
"""
from pathlib import Path
import xml.etree.ElementTree as ET


def resolve_mesh(urdf: Path | str, reference: str) -> Path:
    urdf = Path(urdf).resolve()
    if reference.startswith("package://"):
        package, separator, relative = reference[len("package://"):].partition("/")
        if not separator or not package or not relative:
            raise ValueError(f"invalid mesh reference: {reference}")
        package_root = next((parent for parent in urdf.parents if parent.name == package), None)
        if package_root is None:
            for parent in urdf.parents:
                declaration = parent / "package.xml"
                if declaration.is_file() and ET.parse(declaration).getroot().findtext("name") == package:
                    package_root = parent
                    break
                candidate = parent / package
                if (candidate / "package.xml").is_file():
                    package_root = candidate
                    break
        if package_root is None:
            raise ValueError(f"cannot locate URDF package {package}: {urdf}")
        path = (package_root / relative).resolve()
        if not path.is_relative_to(package_root.resolve()):
            raise ValueError(f"mesh escapes package: {reference}")
    else:
        path = Path(reference)
        if path.is_absolute() or "://" in reference:
            raise ValueError(f"mesh must use relative or package:// path: {reference}")
        path = (urdf.parent / path).resolve()
    if not path.is_file():
        raise ValueError(f"missing mesh {reference}: {path}")
    return path


def mesh_paths(urdf: Path | str) -> dict[str, Path]:
    return {
        mesh.attrib["filename"]: resolve_mesh(urdf, mesh.attrib["filename"])
        for mesh in ET.parse(urdf).getroot().iter("mesh")
    }
