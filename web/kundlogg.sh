#!/usr/bin/env bash
# kundlogg.sh — följ en kunds trafik live på servern: webbserverns frågor från kundens
# datorer och brandväggens räknare (KUNDISOLERING) var 10:e sekund.
#
#   sudo bash kundlogg.sh sleipner          följ live (Ctrl+C avslutar)
#   sudo bash kundlogg.sh sleipner --nu     visa läget en gång
set -uo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KUND="${1:?Ange kund, t.ex. sleipner}"
DATORER=$(python3 -c "import json,sys;print(' '.join(json.load(open('$DIR/kunder.json'))['kunder']['$KUND']['datorer']))") || { echo "Okänd kund: $KUND"; exit 1; }
MONSTER=$(echo "$DATORER" | sed 's/ /|/g; s/\./\\./g')

visa_brandvagg() {
  echo "--- $(date +%T) brandväggen ($KUND): paket/byte per regel"
  iptables -L KUNDISOLERING -v -n -x | grep -E "$KUND|ESTABLISHED" | awk '{printf "  %8s paket %10s byte  %-6s %s -> %s\n",$1,$2,$3,$8,$9}'
  wg show wg0 latest-handshakes | while read -r nyckel t; do
    ip=$(wg show wg0 allowed-ips | awk -v k="$nyckel" '$1==k{print $2}')
    case " $DATORER $(python3 -c "import json;print(' '.join(json.load(open('$DIR/kunder.json'))['kunder']['$KUND']['robotar']))") " in
      *" ${ip%/32} "*) echo "  WireGuard ${ip%/32}: senast hörd för $(( $(date +%s) - t )) s sedan" ;;
    esac
  done
}

if [ "${2:-}" = "--nu" ]; then
  journalctl -u rise_sdvp.service --since "-24h" --no-pager | grep -E "$MONSTER" | tail -20
  visa_brandvagg
  echo "--- stoppade paket (senaste 24 h):"
  journalctl -k --since "-24h" --no-pager | grep "KUNDISOL-$KUND" | tail -10 | sed -E 's/.*(SRC=[^ ]+ DST=[^ ]+).*(PROTO=[^ ]+)( SPT=[^ ]+ DPT=[^ ]+)?.*/  \1 \2\3/'
  exit 0
fi

echo "Följer $KUND ($DATORER). Ctrl+C avslutar."
journalctl -u rise_sdvp.service -f -n 0 --no-pager | grep --line-buffered -E "$MONSTER" | sed -u 's/^/  webb: /' &
trap 'kill %1 2>/dev/null' EXIT
while true; do visa_brandvagg; sleep 10; done
