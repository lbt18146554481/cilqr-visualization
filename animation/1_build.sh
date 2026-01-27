# 清理上次编译结果（可选，如果遇到编译问题可以取消注释）
# rm -rf build

mkdir -p build
cd build
# debug的编译模式
# 如果使用conda环境，确保使用conda的Python
if [ -n "$CONDA_PREFIX" ]; then
    echo "Using conda Python from: $CONDA_PREFIX"
    cmake -DCMAKE_BUILD_TYPE=Debug -DPython3_ROOT_DIR=$CONDA_PREFIX ..
else
cmake -DCMAKE_BUILD_TYPE=Debug ..  
fi
# cmake  ..
make -j16
cd ..
./build/motion_planning -c ./config/scenario_two_straight.yaml

# ./Voronoi_exe
# python3 plot_python.py
