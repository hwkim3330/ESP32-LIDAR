#!/bin/sh
# Offer the switch something worth queueing.
#
#   tools/load.sh <interface> [mbit] [pcp]
#   tools/load.sh enxc84d44263ba6 80 3
#
# Why this exists, and why it is not the ESP's job. Gates are on egress, so contention has to
# happen on the port the board receives through -- which means the competing traffic has to be
# addressed to the board. The board cannot supply it: a switch never sends a frame back out the
# port it arrived on, broadcast included, so nothing the board transmits can ever compete with
# what it receives. Something else on the bench has to send it.
#
# And there is nowhere to plug that something in. A LAN9662 has two data ports and both are
# taken: on A the sensor and the link to B, on B that link and this board. Making room means
# either giving up the tap -- the only reference measurement of what the sensor actually emits --
# or moving to the LAN9692, which has twelve and is the RECON target anyway.
#
# So: put the PC's USB adapter on a switch port instead of on the tap, give it an address on the
# bench segment, and run this. 100BASE-TX fills at about 95 Mbit/s of UDP; the sensor's own
# stream is 3.3, so anything above ~90 makes the port genuinely contended.
#
# The PCP argument is the point of the exercise. Send the load on a different priority from the
# sensor (which is PCP 0), gate that priority, and the sensor's stream should come through
# untouched while the load is shaped -- which is the thing TAS is actually for, and which cannot
# be demonstrated at all while the only traffic on the wire is the traffic you care about.
set -eu

IFACE=${1:-}
MBIT=${2:-80}
PCP=${3:-3}
TARGET=192.168.1.20      # the board
PORT=9999                # not 7502: the board must not mistake load for sensor data

if [ -z "$IFACE" ]; then
  echo "usage: $0 <interface> [mbit] [pcp]" >&2
  echo "interfaces on this machine:" >&2
  ip -br link | awk '{print "  " $1}' >&2
  exit 2
fi

if ! ip -br addr show "$IFACE" 2>/dev/null | grep -q 192.168.1.; then
  echo "$IFACE has no address on 192.168.1.0/24 -- it is not on the bench segment." >&2
  echo "  sudo ip addr add 192.168.1.30/24 dev $IFACE" >&2
  exit 1
fi

# A VLAN interface is how the priority gets onto the wire: PCP lives in the 802.1Q tag, and an
# untagged frame has no priority for the switch to sort on.
VLAN="$IFACE.1"
if ! ip link show "$VLAN" >/dev/null 2>&1; then
  echo "creating $VLAN with egress priority 0 -> $PCP"
  sudo ip link add link "$IFACE" name "$VLAN" type vlan id 1 egress-qos-map "0:$PCP"
  sudo ip addr add 192.168.1.31/24 dev "$VLAN"
  sudo ip link set "$VLAN" up
fi

echo "sending ${MBIT} Mbit/s of UDP at $TARGET:$PORT on PCP $PCP -- ctrl-C to stop"
exec iperf3 -c "$TARGET" -u -b "${MBIT}M" -t 3600 -p "$PORT" -B 192.168.1.31
