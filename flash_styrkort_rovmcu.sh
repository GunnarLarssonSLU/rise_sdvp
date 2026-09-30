#!/bin/bash
# ==============================================================================
# ⚡ flash_styrkort_rovmcu.sh: bygg och flasha STM32:an på ROV_MCU (Upwis MP101_323)
# ==============================================================================
# Körs PÅ kortets egen Raspberry Pi CM5:
#
#   cd ~/rise_sdvp && ./flash_styrkort_rovmcu.sh            bygg, fråga FLASHA, flasha
#   ./flash_styrkort_rovmcu.sh --ja                          utan FLASHA-frågan
#   ./flash_styrkort_rovmcu.sh --bara-bygg                   bygg bara, rör ingen hårdvara
#   ./flash_styrkort_rovmcu.sh --st-link                     tvinga ST-Link
#   ./flash_styrkort_rovmcu.sh --cm5                         tvinga CM5:ans SWD-ben
#
# Firmware: make rovmcu (F9-kortets layout + BMI270, CAN ur tystläge, DI1-4 ut,
# GPS 2 ur reset; se Embedded/RC_Controller/ROVMCU.md). ELF-filen flashas, så
# styrkortets inställningar (EEPROM) behålls.
#
# Två sätt att nå STM32:an:
#   - ST-Link V2 på USB (som på MacBot), om en sådan sitter i.
#   - CM5:ans egna ben, inbyggt på kortet: GPIO11 = SWCLK, GPIO8 = SWDIO (via R39),
#     GPIO17 = NRST, GPIO16 = BOOT0. openocd med linuxgpiod (fungerar på Pi 5/CM5).
# ==============================================================================
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
FW_DIR="$DIR/Embedded/RC_Controller"
ELF="$FW_DIR/build/fw_rovmcu.elf"

RED='\e[31m'; GREEN='\e[32m'; YELLOW='\e[33m'; BOLD='\e[1m'; NC='\e[0m'

SATT=""; JA=0; BARA_BYGG=0
for a in "$@"; do
  case "$a" in
    --st-link) SATT="stlink" ;;
    --cm5) SATT="cm5" ;;
    --ja) JA=1 ;;
    --bara-bygg) BARA_BYGG=1 ;;
    -h|--hjalp|--help) sed -n '3,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo -e "${RED}Okänt argument: $a${NC}"; exit 1 ;;
  esac
done

# CM5-benen (BCM-numrering)
SWCLK=11; SWDIO=8; NRST=17; BOOT0=16

# --- 1. Bygg (via flash_styrkort.sh, som flashar ELF och kan ST-Link) --------------
if [ -z "$SATT" ] && lsusb 2>/dev/null | grep -qi "st-link"; then
  SATT="stlink"
fi
if [ "$SATT" = "stlink" ]; then
  echo -e "${BOLD}ST-Link hittad: flashar med flash_styrkort.sh.${NC}"
  ARGS=(--maskin rovmcu --fw-dir "$FW_DIR")
  [ "$JA" -eq 1 ] && ARGS+=(--ja)
  [ "$BARA_BYGG" -eq 1 ] && ARGS+=(--bara-bygg)
  exec "$DIR/flash_styrkort.sh" "${ARGS[@]}"
fi

echo -e "${BOLD}Bygger firmware för ROV_MCU...${NC}"
if ! "$DIR/flash_styrkort.sh" --maskin rovmcu --fw-dir "$FW_DIR" --bara-bygg >/tmp/rovmcu_bygg.log 2>&1; then
  tail -20 /tmp/rovmcu_bygg.log
  echo -e "${RED}❌ Bygget misslyckades (hela loggen: /tmp/rovmcu_bygg.log).${NC}"
  exit 1
fi
echo -e "${GREEN}✅ Byggd: $ELF${NC}"
[ "$BARA_BYGG" -eq 1 ] && { echo "Klart (--bara-bygg): ingen hårdvara har rörts."; exit 0; }

# --- 2. Flasha via CM5:ans SWD-ben ---------------------------------------------------
command -v openocd >/dev/null || { echo -e "${RED}openocd saknas: sudo apt install openocd${NC}"; exit 1; }

# GPIO-kretsen för 40-stiftsbenen heter "pinctrl-rp1" på Pi 5/CM5 (gpiochip0 eller
# gpiochip4 beroende på kärnversion).
CHIP=""
for n in 0 1 2 3 4 5 6 7 8 9 10 11 12 13; do
  lbl=$(cat /sys/bus/gpio/devices/gpiochip$n/label 2>/dev/null || true)
  [ -z "$lbl" ] && lbl=$(gpioinfo gpiochip$n 2>/dev/null | head -1)
  if echo "$lbl" | grep -q "pinctrl-rp1"; then CHIP=$n; break; fi
done
if [ -z "$CHIP" ]; then
  echo -e "${RED}Hittade inte Pi 5/CM5:ans GPIO-krets (pinctrl-rp1). Är det här en CM5?${NC}"
  echo "Sätt i en ST-Link och kör med --st-link i stället."
  exit 1
fi

if pgrep -x Car_Client >/dev/null 2>&1; then
  echo -e "${RED}${BOLD}⚠️  Car_Client körs.${NC} Flashningen startar om styrkortet: maskinen ska stå still"
  echo "   (motorn av på hydrauliska maskiner) och RControlStation/robotstyrning vara frånkopplade."
fi
if [ "$JA" -ne 1 ]; then
  read -p "Skriv ordet FLASHA för att programmera STM32:an via CM5 (allt annat avbryter): " SVAR
  [ "$SVAR" = "FLASHA" ] || { echo "Avbrutet — ingenting flashades."; exit 0; }
fi

CFG=$(mktemp /tmp/rovmcu_openocd.XXXXXX.cfg)
cat > "$CFG" <<EOF
adapter driver linuxgpiod
adapter gpio swclk $SWCLK -chip $CHIP
adapter gpio swdio $SWDIO -chip $CHIP
adapter gpio srst $NRST -chip $CHIP
transport select swd
adapter speed 1000
reset_config srst_only srst_push_pull
source [find target/stm32f4x.cfg]
EOF

# BOOT0 låg (vanlig start från flash) medan vi flashar.
command -v gpioset >/dev/null && (gpioset gpiochip$CHIP $BOOT0=0 2>/dev/null || gpioset -c gpiochip$CHIP $BOOT0=0 2>/dev/null) &

echo -e "\n${BOLD}Flashar via CM5 (gpiochip$CHIP: SWCLK=$SWCLK SWDIO=$SWDIO NRST=$NRST)...${NC}"
if sudo openocd -f "$CFG" -c "program $ELF verify reset exit"; then
  echo -e "${GREEN}✅ Flashningen slutförd och verifierad.${NC}"
  RES=0
else
  echo -e "${RED}❌ Flashningen misslyckades.${NC} Kontrollera att R39 (SWDIO) är monterad och att"
  echo "   STM32:an har ström. Pröva annars med ST-Link (--st-link)."
  RES=1
fi
rm -f "$CFG"

# STM32:ans reset (GPIO17) och BOOT0 (GPIO16) ska ha fasta nivåer när ingen flashar,
# annars kan Pi:ns standardneddragning på GPIO17 hålla STM32:an i reset.
if ! grep -qsE "^gpio=17=op,dh" /boot/firmware/config.txt; then
  echo -e "\n${YELLOW}Tips:${NC} lägg till i /boot/firmware/config.txt (och starta om CM5 en gång):"
  echo "   gpio=17=op,dh   # STM32 NRST hög (kör)"
  echo "   gpio=16=op,dl   # STM32 BOOT0 låg (starta från flash)"
fi

# Kontroll: läs kortets firmwareversion via Car_Client om den kör.
if [ "$RES" -eq 0 ] && pgrep -x Car_Client >/dev/null 2>&1 && [ -f "$DIR/Linux/tools/fw_version.py" ]; then
  sleep 8
  echo "Firmware på kortet: $(cd /tmp && python3 "$DIR/Linux/tools/fw_version.py" 2>/dev/null || echo 'svarar inte (RControlStation ansluten?)')"
fi
exit $RES
