#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# EtherCAT mailbox-readiness gate.
# Waits until every drive on master 1 answers a CoE upload before the ROS2
# control stack is launched. This avoids the PREOP->SAFEOP startup race where
# the master begins the transition before a slave's CoE mailbox is ready,
# producing "CoE download ... timeout / No response" and AL 0x001D
# "Invalid output configuration" (typically on slave 1-0).
#
# Runs in the IgH master "idle" phase (no app attached) -- same window the
# reset_coe_faults binary uses.

set -u

MASTER=1
# Slave positions that must be mailbox-ready before launch.
DRIVE_SLAVES=(0 1 2)

# Object read to probe the mailbox: 0x1018:01 = CoE Identity / Vendor ID.
# Present on every CoE slave, cheap, and read-only.
PROBE_INDEX=0x1018
PROBE_SUBINDEX=1
PROBE_TYPE=uint32

OVERALL_TIMEOUT=${EC_WAIT_TIMEOUT:-60}   # max seconds to wait in total
POLL_INTERVAL=${EC_WAIT_POLL:-0.5}       # seconds between full sweeps
STABLE_PASSES=${EC_WAIT_STABLE:-2}       # consecutive all-ready sweeps required

ec() {
  ethercat -m "$MASTER" "$@"
}

probe_slave() {
  # Returns 0 if the slave answers the CoE upload, non-zero otherwise.
  local slave="$1"
  local val
  val=$(ec upload -p "$slave" --type "$PROBE_TYPE" \
        "$PROBE_INDEX" "$PROBE_SUBINDEX" 2>/dev/null | awk '{print $1}')
  [[ -n "$val" ]]
}

echo "*** Waiting for EtherCAT master $MASTER mailbox readiness ***"
echo "    slaves: ${DRIVE_SLAVES[*]}  timeout: ${OVERALL_TIMEOUT}s"

start=$(date +%s)
passes=0

while :; do
  all_ready=1
  for p in "${DRIVE_SLAVES[@]}"; do
    if ! probe_slave "$p"; then
      all_ready=0
      echo "  slave $p: mailbox not ready yet"
    fi
  done

  if [[ $all_ready -eq 1 ]]; then
    passes=$((passes + 1))
    echo "  all slaves ready (stable pass $passes/$STABLE_PASSES)"
    [[ $passes -ge $STABLE_PASSES ]] && break
  else
    passes=0
  fi

  now=$(date +%s)
  if (( now - start >= OVERALL_TIMEOUT )); then
    echo "WARNING: mailbox-readiness gate timed out after ${OVERALL_TIMEOUT}s." >&2
    echo "         Proceeding anyway; startup may still fail." >&2
    ec slaves 2>/dev/null || true
    exit 124
  fi

  sleep "$POLL_INTERVAL"
done

echo "*** EtherCAT master $MASTER ready: all drives answering CoE ***"
exit 0
