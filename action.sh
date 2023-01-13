frag="debug"
if [ $frag = "debug" ]; then
    if [ ! -d "build_debug" ]; then
        mkdir "build_debug"
    fi
    cd build_debug
elif [ $frag = "release" ]; then
    if [ ! -d "build_release" ]; then
        mkdir "build_release"
    fi
    cd build_release
fi
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=$frag ..
make -j4
