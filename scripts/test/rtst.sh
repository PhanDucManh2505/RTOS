#!/bin/sh
# rtst.sh - test harness for the RTS traffic system. Lives on vm1.
#
# Starts all eight processes the same way run_vm*.sh does (railway on vm3,
# six intersections on vm2, central on vm1) but with:
#   - keyboard input for central and railway coming from a FIFO, so a test
#     can "press" keys:            rtst.sh key c f      rtst.sh key r a
#   - screens captured as plain text (RTS_COLOR=0) into /dev/shmem/rtst
#   - logs written to /tmp/manh/t on each node, away from the demo logs
#
#   rtst.sh start [speed]    stop everything, then start all eight
#   rtst.sh stop             stop everything
#   rtst.sh key c|r TEXT     send keys to central (c) or railway (r)
#   rtst.sh frame c|r        print the tail of that screen capture
#   rtst.sh vm2 N            print the tail of intersection N's screen
#   rtst.sh logs             print all eight test logs, current run only
#   rtst.sh shm              dump the VM2 corridor region (green wave)
#   rtst.sh ps               where each process is running
#   rtst.sh kill NAME [node] slay one process (node vm1|vm2|vm3)
#   rtst.sh restart NAME     start one process again (central, railway, iN)
T=/tmp/manh/t
O=/dev/shmem/rtst_
B=/tmp/manh
PROCS="central railway intersection_i1 intersection_i2 intersection_i3 intersection_i4 intersection_i5 intersection_i6"
SP_FILE=$T/speed

stopall() {
    for p in $PROCS; do
        slay -f $p 2>/dev/null; on -f vm2 slay -f $p 2>/dev/null; on -f vm3 slay -f $p 2>/dev/null
    done
    if [ -f $T/holders ]; then kill $(cat $T/holders) 2>/dev/null; rm -f $T/holders; fi
}

start_rail() {
    on -f vm3 -e RTS_COLOR=0 -e RTS_SPEED=$SP sh -c "cd $T && exec $B/railway" < $T/r.in >> ${O}r.out 2>&1 &
}
start_inter() {
    on -f vm2 -e RTS_COLOR=0 -e RTS_SPEED=$SP sh -c "cd $T && exec $B/intersection_i$1" < /dev/null >> ${O}i$1.out 2>&1 &
}
start_central() {
    sh -c "cd $T && exec $B/central" < $T/c.in >> ${O}c.out 2>&1 &
}
holders() {
    sleep 999999 > $T/c.in 2>/dev/null < /dev/null & echo $! >> $T/holders
    sleep 999999 > $T/r.in 2>/dev/null < /dev/null & echo $! >> $T/holders
}

export RTS_COLOR=0 RTS_NODE_CENTRAL=vm1 RTS_NODE_INTER=vm2 RTS_NODE_RAIL=vm3
[ -f $SP_FILE ] && SP=$(cat $SP_FILE)
export RTS_SPEED=${SP:-5}

case "$1" in
start)
    SP=${2:-5}; export RTS_SPEED=$SP
    stopall; sleep 2
    mkdir -p $T; on -f vm2 mkdir -p $T; on -f vm3 mkdir -p $T
    rm -f $T/*.log; on -f vm2 sh -c "rm -f $T/*.log"; on -f vm3 sh -c "rm -f $T/*.log"
    rm -f $T/c.in $T/r.in ${O}*; mkfifo $T/c.in $T/r.in
    echo $SP > $SP_FILE
    holders
    start_rail;                       sleep 2
    [ "$3" = noauto ] && printf t > $T/r.in
    for n in 1 2 3 4 5 6; do start_inter $n; sleep 1; done
    sleep 1
    start_central;                    sleep 2
    echo "started, speed $SP"
    ;;
stop)
    stopall; echo stopped ;;
key)
    w=$2; shift 2; printf '%s' "$*" > $T/$w.in ;;
frame)
    tail -c 5000 ${O}$2.out ;;
vm2)
    tail -n ${3:-40} ${O}i$2.out ;;
trunc)
    for f in ${O}c.out ${O}r.out; do : > $f; done ;;
snap)
    tail -c 5000 ${O}c.out; echo "#### VM2"
    for n in 1 2 3 4 5 6; do tail -n 4 ${O}i$n.out; done ;;
shm)
    # the corridor region the six controllers share on vm2, as bytes:
    # each slot carries the planned start of that controller's cycle on
    # vm2's own clock, which is the only common time base they have
    od -A n -t u1 /net/vm2/dev/shmem/rts_corridor ;;
logs)
    for f in $T/central.log /net/vm2/data/var/tmp/manh/t/intersection_I1.log \
             /net/vm2/data/var/tmp/manh/t/intersection_I2.log /net/vm2/data/var/tmp/manh/t/intersection_I3.log \
             /net/vm2/data/var/tmp/manh/t/intersection_I4.log /net/vm2/data/var/tmp/manh/t/intersection_I5.log \
             /net/vm2/data/var/tmp/manh/t/intersection_I6.log /net/vm3/data/var/tmp/manh/t/railway.log; do
        echo "#### $f"; cat $f 2>/dev/null
    done ;;
ps)
    for n in vm1 vm2 vm3; do
        if [ $n = vm1 ]; then r=""; else r="on -f $n"; fi
        printf "%s: " $n
        $r pidin -f an 2>/dev/null | awk '/central|railway|intersection_i/ {print $2}' | sed 's,.*/,,' | sort | tr '\n' ' '
        echo
    done ;;
kill)
    case "${3:-vm1}" in vm1) slay ${2};; *) on -f $3 slay $2;; esac ;;
restart)
    case "$2" in
    central) start_central ;;
    railway) start_rail ;;
    i[1-6])  start_inter ${2#i} ;;
    esac
    sleep 1; echo "restarted $2" ;;
*)
    sed -n '2,20p' $0 ;;
esac
