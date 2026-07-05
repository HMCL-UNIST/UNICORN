#!/usr/bin/env bash
#
# dds_netcheck.sh — measure live network state and recommend CycloneDDS socket
# buffer sizes (the SocketReceiveBufferSize in cyclonedds.xml + net.core.r/wmem_max).
#
# It samples the real traffic on your DDS network interface, checks whether the
# kernel is dropping UDP datagrams *right now* (the only sure sign the buffer is
# too small), reads memory + ping latency, then sizes the buffer as
#     throughput[B/s] x stall-window[s] x safety,   floored by the biggest message.
#
# Usage:
#   ./dds_netcheck.sh [iface] [seconds] [ping_target]
#   IFACE=wlp9s0 ./dds_netcheck.sh           # override interface
#   ./dds_netcheck.sh wlp9s0 5 192.168.60.1  # 5 s sample, ping the car
#
# No root needed. Linux only (reads /proc, /sys). Run it WHILE the stack is up.

set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
XML="$REPO/cyclonedds.xml"

DURATION="${2:-${DURATION:-5}}"
PING_TARGET="${3:-${PING_TARGET:-}}"

# --- pick the DDS interface: arg/env > cyclonedds.xml named iface > default route
pick_iface() {
  [ -n "${1:-}" ] && { echo "$1"; return; }
  [ -n "${IFACE:-}" ] && { echo "$IFACE"; return; }
  if [ -f "$XML" ]; then
    local n; n=$(grep -oE 'NetworkInterface name="[^"]+"' "$XML" | sed -E 's/.*"([^"]+)"/\1/' | head -1)
    [ -n "$n" ] && [ -d "/sys/class/net/$n" ] && { echo "$n"; return; }
  fi
  ip route show default 2>/dev/null | awk '{print $5; exit}'
}
IFACE="$(pick_iface "${1:-}")"
[ -d "/sys/class/net/$IFACE" ] || { echo "ERROR: interface '$IFACE' not found"; exit 1; }
case "$IFACE" in wl*) WIRELESS=1;; *) WIRELESS=0;; esac

# --- snmp UDP counter by name (robust to column order) -----------------------
udp_field() {  # $1 = field name (e.g. RcvbufErrors)
  awk -v f="$1" '
    /^Udp:/ && $1=="Udp:" && !hdr { for(i=2;i<=NF;i++) col[$i]=i; hdr=1; next }
    /^Udp:/ && hdr { print $(col[f]); exit }' /proc/net/snmp
}

rx0=$(cat /sys/class/net/$IFACE/statistics/rx_bytes)
tx0=$(cat /sys/class/net/$IFACE/statistics/tx_bytes)
rcverr0=$(udp_field RcvbufErrors); snderr0=$(udp_field SndbufErrors); inerr0=$(udp_field InErrors)

echo "Sampling $IFACE for ${DURATION}s (run the stack now if it isn't already)..."
sleep "$DURATION"

rx1=$(cat /sys/class/net/$IFACE/statistics/rx_bytes)
tx1=$(cat /sys/class/net/$IFACE/statistics/tx_bytes)
rcverr1=$(udp_field RcvbufErrors); snderr1=$(udp_field SndbufErrors); inerr1=$(udp_field InErrors)

RX_BPS=$(( (rx1 - rx0) / DURATION ))
TX_BPS=$(( (tx1 - tx0) / DURATION ))
D_RCVERR=$(( rcverr1 - rcverr0 ))
D_SNDERR=$(( snderr1 - snderr0 ))
D_INERR=$(( inerr1 - inerr0 ))

# --- latency / jitter (optional) ---------------------------------------------
RTT_AVG=""; RTT_MDEV=""
if [ -n "$PING_TARGET" ]; then
  if line=$(ping -c 5 -i 0.2 -w 3 "$PING_TARGET" 2>/dev/null | grep -E 'rtt|round-trip'); then
    RTT_AVG=$(echo "$line"  | awk -F'/' '{print $5}')
    RTT_MDEV=$(echo "$line" | awk -F'/' '{print $7}' | tr -d ' ms')
  fi
fi

# --- current settings --------------------------------------------------------
RMEM=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
WMEM=$(sysctl -n net.core.wmem_max 2>/dev/null || echo 0)
XML_RECV=$(grep -oE 'SocketReceiveBufferSize[^/]*' "$XML" 2>/dev/null | grep -oE 'min="[^"]+"' | sed -E 's/min="([^"]+)"/\1/' | head -1)
MEM_AVAIL_MB=$(free -m | awk '/^Mem:/{print $7}')

# --- recommendation (floats via awk) -----------------------------------------
# stall window D: WiFi jitter is the dominant unknown. Use (avg + 4*mdev) if we
# pinged, else a wireless/wired default. Clamp to [0.1, 0.5] s.
read -r REC_RECV_MB REC_SEND_MB REQ_RMEM REQ_WMEM D_USED <<EOF
$(awk -v rx="$RX_BPS" -v tx="$TX_BPS" -v wl="$WIRELESS" \
       -v avg="${RTT_AVG:-}" -v mdev="${RTT_MDEV:-}" -v rmem="$RMEM" -v wmem="$WMEM" '
  BEGIN{
    if (avg!="" && mdev!="") d=(avg+4*mdev)/1000.0; else d=(wl? 0.30 : 0.15);
    if (d<0.1) d=0.1; if (d>0.5) d=0.5;
    safety=3; floor=8*1048576;                    # >=8MB to absorb a latched map burst
    recv=rx*d*safety; if(recv<floor) recv=floor;
    send=tx*d*safety; if(send<4*1048576) send=4*1048576;
    rmb=int(recv/1048576)+1; smb=int(send/1048576)+1;
    reqr=rmb*1048576; if(reqr<rmem) reqr=rmem;
    reqw=smb*1048576; if(reqw<wmem) reqw=wmem;
    printf "%d %d %d %d %.2f", rmb, smb, reqr, reqw, d;
  }')
EOF

mb(){ awk -v b="$1" 'BEGIN{printf "%.2f", b/1048576}'; }

echo
echo "============================================================"
echo " DDS network check — iface=$IFACE  ($([ $WIRELESS = 1 ] && echo wireless || echo wired))"
echo "============================================================"
printf " live throughput     : RX %s MB/s   TX %s MB/s\n" "$(mb $RX_BPS)" "$(mb $TX_BPS)"
printf " UDP drops (in %ss)   : RcvbufErrors +%s   SndbufErrors +%s   InErrors +%s\n" "$DURATION" "$D_RCVERR" "$D_SNDERR" "$D_INERR"
printf " current kernel cap  : rmem_max %s MB   wmem_max %s MB\n" "$(mb $RMEM)" "$(mb $WMEM)"
printf " cyclonedds.xml recv : %s\n" "${XML_RECV:-<not set>}"
printf " free memory         : %s MB available\n" "$MEM_AVAIL_MB"
[ -n "$RTT_AVG" ] && printf " ping %-15s: avg %s ms   jitter(mdev) %s ms\n" "$PING_TARGET" "$RTT_AVG" "$RTT_MDEV"
echo "------------------------------------------------------------"
echo " VERDICT"
if [ "$D_RCVERR" -gt 0 ]; then
  echo "  ✗ kernel is DROPPING UDP now (RcvbufErrors climbing) — buffer TOO SMALL."
else
  echo "  ✓ no UDP receive-buffer drops during the sample."
fi
xmlb=$(awk -v s="${XML_RECV:-0}" 'BEGIN{n=s+0; if(s ~ /MB/)n*=1048576; else if(s ~ /[kK]B/)n*=1024; print int(n)}')
if [ "${xmlb:-0}" -gt 0 ] && [ "$RMEM" -lt "$xmlb" ]; then
  echo "  ✗ rmem_max ($(mb $RMEM) MB) < cyclonedds.xml request ($(mb $xmlb) MB) — XML is CAPPED. Raise rmem_max."
fi
[ "${MEM_AVAIL_MB:-9999}" -lt 1024 ] && echo "  ! low free RAM (${MEM_AVAIL_MB} MB) — don't oversize the buffer on this host."
[ "$WIRELESS" = 1 ] && echo "  i wireless link: jitter inflates the stall window, so a larger buffer is justified."
echo "------------------------------------------------------------"
echo " RECOMMENDATION  (stall window D=${D_USED}s, safety x3)"
printf "   cyclonedds.xml : <SocketReceiveBufferSize min=\"%sMB\"/>   (send: %sMB)\n" "$REC_RECV_MB" "$REC_SEND_MB"
printf "   sysctl         : net.core.rmem_max=%s  net.core.wmem_max=%s\n" "$REQ_RMEM" "$REQ_WMEM"
echo "   (rmem_max/wmem_max must be >= the XML values, or the XML is capped.)"
echo "============================================================"
