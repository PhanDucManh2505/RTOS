#!/bin/sh
# Collect the current run's log segment from all three nodes.
# vm2 and vm3 files are read straight over Qnet. Note /net/<node>/tmp is a
# trap: /tmp is an absolute symlink to /data/var/tmp and resolves back to
# the local node, so the real remote path has to be spelled out.
V1=/tmp/manh
V2=/net/vm2/data/var/tmp/manh
V3=/net/vm3/data/var/tmp/manh

seg() {                       # label  file  maxlines
    echo "#### $1"
    n=$(grep -n started "$2" 2>/dev/null | tail -1 | cut -d: -f1)
    [ -z "$n" ] && n=1
    tail -n +"$n" "$2" | head -"$3"
}

for i in 1 2 3 4 5 6; do
    seg "I$i" "$V2/intersection_I$i.log" 110
done
seg RAILWAY "$V3/railway.log" 60
seg CENTRAL "$V1/central.log" 40
