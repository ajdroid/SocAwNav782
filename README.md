# SocAwNav782
16782 Planning project using Stanford Drone Dataset

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Setup

This project was tested on Ubuntu 16.04. Follow the setup guide below to setup the project

### C++ Libraries

Install [XTL](https://github.com/QuantStack/xtl)

```
git clone https://github.com/QuantStack/xtl.git
cd xtl
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install
cd
```

Install [xtensor](https://github.com/QuantStack/xtensor)

```
git clone https://github.com/QuantStack/xtensor.git
cd xtensor
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install
cd
```

Install [pybind11](https://github.com/pybind/pybind11/)

```
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install
cd
```

Install [xtensor-python](https://github.com/QuantStack/xtensor-python)

```
git clone https://github.com/QuantStack/xtensor-python.git
cd xtensor-python
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install
cd
```


```
pip install numpy
pip install matplotlib
pip install 
```