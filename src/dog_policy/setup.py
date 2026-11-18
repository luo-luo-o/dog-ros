from setuptools import setup


setup(
    name="dog_policy",
    version="0.1.0",
    packages=["dog_policy"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dog_policy"]),
        ("share/dog_policy", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="dog-ros",
    maintainer_email="maintainers@example.invalid",
    description="Quadruped policy inference and version management",
    license="Unspecified",
    entry_points={"console_scripts": ['policy = dog_policy.cli:main']},
)
