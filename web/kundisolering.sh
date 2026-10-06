#!/usr/bin/env bash
# kundisolering.sh — brandväggsregler på WireGuard-servern så att varje kund bara når
# sina egna robotar i tunneln (wg0). Läser kunder.json (samma fil som server.py).
#
#   sudo bash kundisolering.sh            lägg på reglerna (ersätter tidigare)
#   bash kundisolering.sh --visa          visa reglerna utan att ändra något
#   sudo bash kundisolering.sh --ta-bort  ta bort reglerna (allt öppet som förut)
#   sudo bash kundisolering.sh --installera   dessutom: lägg på vid varje uppstart
#
# Regler (kedjan KUNDISOLERING, anropas för trafik wg0 -> wg0):
#   - svar på befintliga anslutningar släpps alltid (vi kan nå kundernas robotar)
#   - kunddator -> egna robotar: ok; kunddator -> allt annat i tunneln: stopp
#   - kundrobot -> egna kunddatorer: ok; kundrobot -> allt annat i tunneln: stopp
#   - stoppade paket loggas (högst 10/min per regel) i kärnloggen: journalctl -k | grep KUNDISOL
#   - allt annat (våra datorer och robotar) påverkas inte
# Servern själv (192.168.200.1, webbservern) påverkas inte: den trafiken går inte via
# FORWARD.
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KUNDFIL="$DIR/kunder.json"
KEDJA=KUNDISOLERING
IF=wg0
LAGE="${1:-}"

regler() {
  python3 - "$KUNDFIL" <<'EOF'
import json, sys
kunder = json.load(open(sys.argv[1])).get('kunder', {})
print("-m conntrack --ctstate ESTABLISHED,RELATED -j RETURN")
for namn, k in kunder.items():
    datorer, robotar = k.get('datorer', []), k.get('robotar', [])
    for d in datorer:
        for r in robotar:
            print(f"-s {d} -d {r} -j RETURN -m comment --comment {namn}")
        print(f"-s {d} -m limit --limit 10/min -j LOG --log-prefix KUNDISOL-{namn}: -m comment --comment {namn}")
        print(f"-s {d} -j DROP -m comment --comment {namn}")
    for r in robotar:
        for d in datorer:
            print(f"-s {r} -d {d} -j RETURN -m comment --comment {namn}")
        print(f"-s {r} -m limit --limit 10/min -j LOG --log-prefix KUNDISOL-{namn}: -m comment --comment {namn}")
        print(f"-s {r} -j DROP -m comment --comment {namn}")
print("-j RETURN")
EOF
}

if [ "$LAGE" = "--visa" ]; then
  echo "Regler i kedjan $KEDJA (anropas från FORWARD -i $IF -o $IF):"
  regler | sed "s/^/  iptables -A $KEDJA /"
  exit 0
fi

[ "$(id -u)" -eq 0 ] || { echo "Kör med sudo."; exit 1; }

ta_bort() {
  while iptables -C FORWARD -i "$IF" -o "$IF" -j "$KEDJA" 2>/dev/null; do
    iptables -D FORWARD -i "$IF" -o "$IF" -j "$KEDJA"
  done
  iptables -F "$KEDJA" 2>/dev/null || true
  iptables -X "$KEDJA" 2>/dev/null || true
}

if [ "$LAGE" = "--ta-bort" ]; then
  ta_bort
  systemctl disable --now kundisolering.service 2>/dev/null || true
  echo "Kundisoleringen är borttagen."
  exit 0
fi

# Bygg en ny kedja och koppla in den (ersätter en tidigare).
iptables -N "$KEDJA" 2>/dev/null || iptables -F "$KEDJA"
while read -r rad; do
  # shellcheck disable=SC2086
  iptables -A "$KEDJA" $rad
done < <(regler)
iptables -C FORWARD -i "$IF" -o "$IF" -j "$KEDJA" 2>/dev/null || iptables -I FORWARD 1 -i "$IF" -o "$IF" -j "$KEDJA"
echo "Kundisoleringen är på:"
iptables -S "$KEDJA" | sed 's/^/  /'

if [ "$LAGE" = "--installera" ]; then
  cat > /etc/systemd/system/kundisolering.service <<EOF
[Unit]
Description=Kundisolering i WireGuard (kunder når bara sina egna robotar)
After=wg-quick@$IF.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash $DIR/kundisolering.sh

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable kundisolering.service
  echo "Läggs på vid varje uppstart (kundisolering.service)."
fi
