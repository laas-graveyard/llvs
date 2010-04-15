#!/bin/bash
/usr/bin/scp ../build/libllvs-0.so dune@hrp2010v:~/devel/openrobots/lib
/usr/bin/scp ../build/LLVS_server  dune@hrp2010v:~/devel/openrobots/bin
/usr/bin/scp ../server/VisionSystemProfiles/* dune@hrp2010v:~/devel/openrobots/bin
