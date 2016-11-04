# hpp-quadcopter

### Install instructions

Install the external dependencies.
```bash
sudo apt-get install libnlopt-dev libgoogle-glog-dev
```

Follow [these instructions](https://humanoid-path-planner.github.io/hpp-doc/download.html?branch=master) to install HPP.

Then,
```bash
cd ${DEVEL_DIR}/src
git clone --recursive git@github.com:jmirabel/hpp-quadcopter
mkdir hpp-quadcopter/build-rel
cd hpp-quadcopter/build-rel
# Change the option as it suits you.
cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_DIR}/install -DCMAKE_BUILD_TYPE=Release ..
make install
```

### How to use

Launch command `hpp-quadcopter-server` and `gepetto-viewer-server`.
You can then execute the example Python script `script/quadcopter.py`.
