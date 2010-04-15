#!/bin/bash
/usr/bin/scp ../build/libllvs-0.so ~/devel/openrobots/lib
/usr/bin/scp ../build/LLVS_server  ~/devel/openrobots/bin
/usr/bin/scp ../server/VisionSystemProfiles/* ~/devel/openrobots/bin
/usr/bin/scp ../scripts/run.sh  ~/devel/openrobots/bin
