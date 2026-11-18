from glob import glob

from setuptools import setup


setup(
    name="dog_controller",
    version="0.1.0",
    packages=["dog_controller"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dog_controller"]),
        ("share/dog_controller", ["package.xml"]),
        ("share/dog_controller/model/dog", glob("model/dog/*.urdf")),
        ("share/dog_controller/model/dog/meshes", glob("model/dog/meshes/*.stl")),
    ],
    install_requires=["setuptools"],
    maintainer="dog-ros",
    maintainer_email="maintainers@example.invalid",
    description="Quadruped control middleware and URDF model",
    license="Unspecified",
)
