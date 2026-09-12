#!/bin/sh
# VM1 - the control room. One process, one terminal.
cd "$(dirname "$0")" || exit 1
export RTS_NODE_CENTRAL=${RTS_NODE_CENTRAL:-vm1}
export RTS_NODE_INTER=${RTS_NODE_INTER:-vm2}
export RTS_NODE_RAIL=${RTS_NODE_RAIL:-vm3}
export TERM=${TERM:-xterm-256color}
mkdir -p /fs/rts
exec ./central "$@"
