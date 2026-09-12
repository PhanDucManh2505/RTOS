#!/bin/sh
# start_qnet.sh - bring Qnet up on a mkqnximage target.
#
# Plain "mount -T io-pkt lsm-qnet.so" is refused on the default boot
# instance, so the stack is restarted with the qnet module built in.
# Restarting io-pkt drops the SSH session once; it comes straight back.
#
# Usage:  ./start_qnet.sh <ip-address> <hostname>
#   e.g.  ./start_qnet.sh 192.168.56.110 vm1
#
# Run it detached so it survives the SSH drop:
#   on -d -s /tmp/manh/start_qnet.sh 192.168.56.110 vm1

IP=${1:-192.168.56.110}
HOST=${2:-vm1}
IFACE=${3:-wm0}

exec </dev/null >/tmp/start_qnet.log 2>&1

echo "setting hostname to $HOST"
hostname "$HOST"

sleep 3
echo "restarting io-pkt with the qnet module"
slay io-pkt-v6-hc
sleep 2
io-pkt-v6-hc -d e1000 -p qnet -U 33:33
sleep 3

echo "restoring the interface address"
ifconfig "$IFACE" "$IP" netmask 255.255.255.0 up
sleep 1

echo "restarting qconn (io-pkt took it down with it)"
qconn &

sleep 2
echo "nodes visible under /net:"
ls /net
