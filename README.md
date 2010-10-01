llvs
====

llvs stands for low-level vision server. It is a vision server which
processes images from camera and apply algorithms on it
non-interactively. It has been developed for robotics purpose and is
able to communicate with the dynamic-graph framework.

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The package depends on several packages which have to be available on
your machine.

 - Libraries:
   - TO BE DONE
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
