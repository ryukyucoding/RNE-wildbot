from setuptools import find_packages, setup
import os

package_name = "yolo_example_pkg"

_models_dir = "models"
_model_install_files = []
if os.path.isdir(_models_dir):
    _model_install_files = [
        os.path.join(_models_dir, f) for f in sorted(os.listdir(_models_dir))
    ]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "models"), _model_install_files),
        (
            os.path.join("share", package_name),
            ["requirements-yolo-node.txt"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="alianlbj23@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_node = yolo_example_pkg.object_detect:main",
            "seg_node = yolo_example_pkg.object_segment:main",
        ],
    },
)
