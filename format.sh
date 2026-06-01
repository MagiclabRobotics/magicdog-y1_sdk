#!/bin/bash

# clang-format 安装方法：sudo apt install clang-format
# clang-format, version v14.0.0-1ubuntu1.1 is required
find ./sdk -regex '.*\.cc\|.*\.cpp\|.*\.h\|.*\.hpp\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=file
find ./example -regex '.*\.cc\|.*\.cpp\|.*\.h\|.*\.hpp\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=file
find ./python -regex '.*\.cc\|.*\.cpp\|.*\.h\|.*\.hpp\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=file
echo "clang-format done"

# cmake-format 安装方法：sudo apt install cmake-format
# cmake-format, version 0.6.13 is required
{ find . -maxdepth 1 -name "CMakeLists.txt"; find ./sdk -name "CMakeLists.txt"; } | xargs cmake-format -c ./.cmake-format.py -i
{ find ./cmake -name "*.cmake"; find ./sdk -name "*.cmake"; } | xargs cmake-format -c ./.cmake-format.py -i
echo "cmake-format done"

# black 安装方法：pip install black
# black, version 25.1.0 is required
black example/python/
black example/pybind11_test/
echo "black python format done"