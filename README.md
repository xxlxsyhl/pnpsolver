# PnP Solver

Simplified [COLMAP](https://github.com/colmap/colmap) PnP Solver with weight PnP Solver Support.

## About
____
We have simplified COLMAP project and keeping only the PnP Solver parts. Then enhence the project with weighted PnP Solver. 

## Dependencies
____
- C++11
- [CMake](https://cmake.org/) is a cross platform build system.
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) is used extensively for doing nearly all the matrix and linear algebra operations.
- [Ceres Solver](http://ceres-solver.org/) is a library for solving non-linear least squares problems.
- [Glog](https://code.google.com/archive/p/google-glog/) is used for error checking and logging.
- [Boost](https://www.boost.org/) is used for filesystem operation and unit testing.

## Install
____
```
mkdir build && cd build
cmake .. && make -j4
sudo make install
```

## Acknowledgements
____
The Backbone of PnP Solver is based on [COLMAP](https://github.com/colmap/colmap)

    @inproceedings{schoenberger2016sfm,
        author={Sch\"{o}nberger, Johannes Lutz and Frahm, Jan-Michael},
        title={Structure-from-Motion Revisited},
        booktitle={Conference on Computer Vision and Pattern Recognition (CVPR)},
        year={2016},
    }
## Licence
____
    Copyright (c) 2021, ZJU and SenseTime 
    All rights reserved. 

    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met: 

    * Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer. 

    * Redistributions in binary form must reproduce the above copyright 
    notice, this list of conditions and the following disclaimer in the 
    documentation and/or other materials provided with the distribution. 
    
    * Neither the name of ZJU and SenseTime nor the names of its contributors 
    may be used to endorse or promote products derived from this software 
    without specific prior written permission. 

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    POSSIBILITY OF SUCH DAMAGE.