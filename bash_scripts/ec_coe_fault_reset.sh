#!/usr/bin/env bash
set -u

MASTER=1
DRIVE_SLAVES=(0 1 2)

SLEEP_SHORT=0.2
SLEEP_LONG=0.5

ec() {
  ethercat -m "$MASTER" "$@"
}

read_u16() {
  local slave="$1"
  local index="$2"
  local subindex="$3"

  ec upload -p "$slave" --type uint16 "$index" "$subindex" 2>/dev/null \
    | awk '{print $1}'
}

write_u16() {
  local slave="$1"
  local index="$2"
  local subindex="$3"
  local value="$4"

  ec download -p "$slave" --type uint16 "$index" "$subindex" "$value" >/dev/null 2>&1
}

echo "Using EtherCAT master $MASTER"
echo "Resetting DS402 drives only: ${DRIVE_SLAVES[*]}"
echo

for p in "${DRIVE_SLAVES[@]}"; do
  echo "=============================="
  echo "Drive slave $p"
  echo "=============================="

  cw_before=$(read_u16 "$p" 0x6040 0 || true)
  sw_before=$(read_u16 "$p" 0x6041 0 || true)
  err_before=$(read_u16 "$p" 0x603F 0 || true)

  echo "Before:"
  echo "  Controlword 0x6040: ${cw_before:-N/A}"
  echo "  Statusword  0x6041: ${sw_before:-N/A}"
  echo "  Error code  0x603F: ${err_before:-N/A}"

  if [[ -z "${sw_before:-}" ]]; then
    echo "  Cannot read DS402 statusword. Skipping."
    echo
    continue
  fi

  echo "Clearing fault reset bit..."
  write_u16 "$p" 0x6040 0 0x0000 || {
    echo "  Failed to write 0x6040=0x0000"
    echo
    continue
  }
  sleep "$SLEEP_SHORT"

  echo "Pulsing fault reset bit..."
  write_u16 "$p" 0x6040 0 0x0080 || {
    echo "  Failed to write 0x6040=0x0080"
    echo
    continue
  }
  sleep "$SLEEP_SHORT"

  write_u16 "$p" 0x6040 0 0x0000 || {
    echo "  Failed to clear 0x6040 after reset"
    echo
    continue
  }
  sleep "$SLEEP_LONG"

  sw_after=$(read_u16 "$p" 0x6041 0 || true)
  err_after=$(read_u16 "$p" 0x603F 0 || true)

  echo "After reset pulse:"
  echo "  Statusword 0x6041: ${sw_after:-N/A}"
  echo "  Error code 0x603F: ${err_after:-N/A}"

  echo
done

echo "Final slave states:"
ec slaves