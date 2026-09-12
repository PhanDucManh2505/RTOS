#!/bin/sh
# deploy_via_vm1.sh - runs on vm1 after bin/* was copied to /tmp/manh/new.
# Keeps the binaries it replaces in bak_<tag> on every node, then copies
# the new ones into place over Qnet and prints the checksums.
TAG=${1:-before_fix}
N=/tmp/manh/new
V1=/tmp/manh
V2=/net/vm2/data/var/tmp/manh
V3=/net/vm3/data/var/tmp/manh

for d in $V1 $V2 $V3; do mkdir -p $d/bak_$TAG; done
[ -f $V1/bak_$TAG/central ] || cp -p $V1/central $V1/intersection_i* $V1/railway $V1/bak_$TAG/
[ -f $V2/bak_$TAG/intersection_i1 ] || cp -p $V2/intersection_i* $V2/bak_$TAG/
[ -f $V3/bak_$TAG/railway ] || cp -p $V3/railway $V3/bak_$TAG/

cp $N/central $N/intersection_i* $N/railway $V1/
cp $N/intersection_i* $V2/
cp $N/railway $V3/
chmod +x $V1/central $V1/intersection_i* $V1/railway $V2/intersection_i* $V3/railway
echo "--- installed:"
cksum $V1/central $V2/intersection_i* $V3/railway
echo "--- backup kept in bak_$TAG on each node:"
cksum $V1/bak_$TAG/central $V2/bak_$TAG/intersection_i1 $V3/bak_$TAG/railway
