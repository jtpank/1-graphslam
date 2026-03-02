Readme goes here.

1. `mkdir build && cd build`
2. `cmake ..`
3. `make -j4`

Eigen lib ubuntu
`sudo apt-get install libeigen3-dev`

MacOs
`brew install eigen`

Pangolin:
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
Notes:
If you get constant Werrors:
 grep -R "Werror" -n CMakeLists.txt components/*/CMakeLists.txt 2>/dev/null
then remove the -Werror

If it complains about the python version on the cmake, force a specific version (may have multiple)
 cmake -B build -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_CXX_FLAGS="-Wno-error -Wno-error=deprecated-copy"

Multithread the build
 `cmake --build build -j 10`


Note: For linux ubuntu 20.04
- check with `lsb_release -a`

install g++-13:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install g++-13
```

Otherwise at the top of `CMakeLists.txt` use:
```
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
```