from glob import glob

from setuptools import setup


setup(
    name="dog_sim",
    version="0.1.0",
    packages=["dog_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dog_sim"]),
        ("share/dog_sim", ["package.xml"]),
        ("share/dog_sim/model", glob("model/*.xml")),
    ],
    install_requires=["setuptools"],
    maintainer="dog-ros",
    maintainer_email="maintainers@example.invalid",
    description="Quadruped MuJoCo simulation backend",
    license="Unspecified",
    entry_points={"console_scripts": ['simulate = dog_sim.node:main']},
)
