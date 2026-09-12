#!/bin/sh
# start_qnet_ssh.sh - like start_qnet.sh, but also restarts sshd so the
# SSH session comes back after io-pkt is cycled. Run detached:
#   on -d -s /tmp/manh/start_qnet_ssh.sh 192.168.56.110 vm1
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

echo "restarting qconn"
qconn &
sleep 1

echo "restarting sshd (io-pkt took it down with it)"
slay -f sshd 2>/dev/null
sleep 1
/system/xbin/sshd -f /system/etc/ssh/sshd_config &

sleep 2
echo "nodes visible under /net:"
ls /net
echo "done"
