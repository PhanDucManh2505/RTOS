#!/bin/sh
# run_vm2_lines.sh - start the six local controllers like run_vm2.sh, but
# hand each one's output to the shared terminal a whole line at a time.
#
# The programs set stdout unbuffered (rts_util.c), so a single printf can
# leave as several small writes. With six processes on one terminal those
# writes interleave mid-line and the merged view turns into letter soup.
# Here each controller writes into its own pipe instead; a reader joins the
# pieces back into full lines and prints each line in one go.
#
# The controllers themselves are untouched: same binaries, same process
# names (so "slay intersection_i3" still works), same timing, same colours.
#
#   ./run_vm2_lines.sh      start all six
#   ./run_vm2.sh stop       stop all six
cd "$(dirname "$0")" || exit 1
export RTS_NODE_CENTRAL=${RTS_NODE_CENTRAL:-vm1}
export RTS_NODE_INTER=${RTS_NODE_INTER:-vm2}
export RTS_NODE_RAIL=${RTS_NODE_RAIL:-vm3}
export TERM=${TERM:-xterm-256color}

whole_lines() {
    while IFS= read -r line; do
        print -r -- "$line"
    done
}

for n in 1 2 3 4 5 6; do
    "./intersection_i$n" 2>&1 | whole_lines &
    sleep 1          # stagger the starts so the banners stay readable
done

echo ""
echo "six local controllers running, output joined line by line."
echo "stop them with: ./run_vm2.sh stop"
wait
