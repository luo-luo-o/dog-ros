from setuptools import setup


setup(
    name="dog_hardware",
    version="0.1.0",
    packages=["dog_hardware"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dog_hardware"]),
        ("share/dog_hardware", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="dog-ros",
    maintainer_email="maintainers@example.invalid",
    description="Quadruped hardware IK and servo interfaces",
    license="Unspecified",
    entry_points={"console_scripts": ['hardware = dog_hardware.node:main']},
)
