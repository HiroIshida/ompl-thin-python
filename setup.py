from skbuild import setup

install_requires = [
    "numpy",
]

setup(
    name="ompl-thin",
    version="0.0.16",
    description="thin ompl python wrapper",
    author="Hirokazu Ishida",
    license="MIT",
    packages=["ompl"],
    package_dir={"": "python"},
    cmake_install_dir="python/ompl/",
    install_requires=install_requires,
    package_data={"ompl": ["py.typed"]},
)
