#!/bin/bash
export OMNIORB_CONFIG=/home/demorobotatcwe/devel/omniorb/omniORB.cfg
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH:/lib/tls
./LLVS_server --fgsize="640x480" --check_entry

