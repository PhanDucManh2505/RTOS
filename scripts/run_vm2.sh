#!/bin/sh
# VM2 - the six local controllers, each its own process.
#
# Without arguments all six are started in the background and their
# output is merged into this terminal, which is what you want for a
# demonstration over a single SSH session.
#
#   ./run_vm2.sh            start all six
#   ./run_vm2.sh 1          start only I1, in the foreground
#   ./run_vm2.sh stop       stop all six, and the panel if it runs
cd "$(dirname "$0")" || exit 1
export RTS_NODE_CENTRAL=${RTS_NODE_CENTRAL:-vm1}
export RTS_NODE_INTER=${RTS_NODE_INTER:-vm2}
export RTS_NODE_RAIL=${RTS_NODE_RAIL:-vm3}
export TERM=${TERM:-xterm-256color}
mkdir -p /fs/rts

if [ "$1" = "stop" ]; then
    for n in 1 2 3 4 5 6; do slay -f "intersection_i$n" 2>/dev/null; done
    slay -f inter_panel 2>/dev/null
    echo "all six local controllers stopped"
    exit 0
fi

if [ -n "$1" ] && [ "$1" != "all" ]; then
    N=$1
    shift
    exec "./intersection_i$N" "$@"
fi

for n in 1 2 3 4 5 6; do
    "./intersection_i$n" &
    sleep 1          # stagger the starts so the banners stay readable
done

echo ""
echo "six local controllers running. Ctrl-C stops this shell;"
echo "use ./run_vm2.sh stop to stop the processes themselves."
wait
