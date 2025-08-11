## ompl-thin [![pypi-version](https://badge.fury.io/py/ompl-thin.svg)](https://pypi.org/project/ompl-thin/)
If you are looking for a thicker wrapper with more extensive features, please refer to OMPL's [official python bindings](https://ompl.kavrakilab.org/python.html).

The main advantages of this package are fast build times and simplicity.
This is achieved by focusing solely on thin-wrapping the OMPL library and leveraging pybind11.

If your primary use case for OMPL involves articulated robots, we recommend trying [plainmp](https://github.com/HiroIshida/plainmp).


## installation
The package is available on [pypi](https://pypi.org/project/ompl-thin/). So, to install you just need to
```bash
pip3 install ompl-thin
```
NOTE: binary distribution is available only for x86_64. For other architecture, only source distributions are available and takes few minutes to install. Also, before installation, you need to run the following to get shared dependency.
```
sudo apt install libboost-all-dev -y
```

## Usage
please try
```bash
python example/many_algorithms.py
```
