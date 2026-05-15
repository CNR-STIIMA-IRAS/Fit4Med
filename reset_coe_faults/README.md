# IgH EtherCAT DS402 Fault Reset Tool

Standalone C++ utility for an IgH EtherCAT network with:

- Master: `1`
- Slaves `0`, `1`, `2`: CoE / CiA-402 drives
- Slave `3`: ATI Axia F/T sensor

The tool is intended to reset faults at multiple levels:

1. EtherCAT / AL level
2. CoE emergency-history level, where supported
3. CiA-402 / DS402 drive fault level via controlword `0x6040`

It also keeps cyclic PDO exchange active so that output watchdogs do not immediately trip.

---

## Network layout

Expected network:

```text
Master 1
  ├── Slave 0: DS402 drive, vendor 0x000001dd, product 0x10305070
  ├── Slave 1: DS402 drive, vendor 0x000001dd, product 0x10305070
  ├── Slave 2: DS402 drive, vendor 0x000001dd, product 0x10305070
  └── Slave 3: ATI Axia F/T Sensor, vendor 0x00000732, product 0x26483053

---

## Important safety notes

Stop the normal robot/control application before running this tool.

Only one userspace application can own the IgH EtherCAT master at a time. If another application is already controlling master `1`, this tool will fail to request the master.

By default, the tool only resets faults and does **not** enable the drives.

The `--enable` option attempts to bring the DS402 drives through:

```text
Shutdown → Switch on → Enable operation
0x0006   → 0x0007    → 0x000F
```

Use `--enable` only when the robot/mechanism is safe to energize.

---

Regardless of the options used, the tool always performs the following steps before entering the cyclic loop:

1. Requests EtherCAT master `1` and creates a process-data domain.
2. Downloads `0` to object `0x1003:00` (pre-defined error field) on each DS402 drive to clear the CoE emergency history.
3. Activates the master.
4. Calls `ecrt_master_reset()` in the activated master context to ask the master to recover slaves from EtherCAT AL errors.
5. Enters the cyclic PDO loop.
6. When a drive reaches CiA-402 `FAULT`, the controlword follows the sequence `0x0000 -> 0x0080 -> 0x0000` to issue the DS402 fault-reset edge on bit 7.
7. If `--enable` is passed, the CiA-402 state machine is driven after the reset pulse is complete until the program is interrupted with `Ctrl-C` or `SIGTERM`.




## Usage

The binary must be run with sufficient privileges to access the EtherCAT master (typically `sudo` or with a real-time capable user).

```bash
sudo reset_coe_faults [options]
```

### Options

| Option | Argument | Default | Description |
|---|---|---|---|
| `--cycle-ns` | `N` | `1000000` | Cyclic PDO loop period in nanoseconds (1 ms default). |
| `--dc` | — | disabled | Enable Distributed Clocks (DC) configuration on drives 0–2. |
| `--dc-assign` | `HEX` | `0x0300` | DC `assignActivate` value written to each drive when `--dc` is set. |
| `--enable` | — | disabled | After the fault-reset sequence, run the CiA-402 state machine (Shutdown → Switch On → Enable Operation) to bring drives to operational state. |
| `--mode` | `N` | `8` | DS402 mode of operation written to all drives when `--enable` is active. Default `8` = Cyclic Synchronous |
| | | | 8  = CSP, Cyclic Synchronous Position |
| | | | 9  = CSV, Cyclic Synchronous Velocity |
| `--help` | — | — | Print a short help message and exit. |


Reset faults only:

```bash
sudo ./reset_igh_ds402
```

Reset faults with 1 ms cycle time:

```bash
sudo ./reset_igh_ds402 --cycle-ns 1000000
```

Reset faults and configure Distributed Clocks:

```bash
sudo ./reset_igh_ds402 --cycle-ns 1000000 --dc --dc-assign 0x0300
```

Reset faults, configure DC, and try to enable DS402 drives in CSP mode:

```bash
sudo ./reset_igh_ds402 --cycle-ns 1000000 --dc --dc-assign 0x0300 --enable --mode 8
```

Show help:

```bash
./reset_igh_ds402 --help
```

Use the mode supported and configured by your drives.

---

## What the tool does internally

### 1. Requests IgH master 1

```cpp
ecrt_request_master(1)
```

### 2. Configures PDOs

The PDO layout is based on the output of:

```bash
ethercat -m 1 cstruct
```

Drives `0`, `1`, `2`:

```text
RxPDO 0x1601:
  0x6040:00 Controlword
  0x607A:00 Target position
  0x60FF:00 Target velocity
  0x6060:00 Mode of operation
  0x60B1:00 Velocity offset

TxPDO 0x1A01:
  0x6041:00 Statusword
  0x6064:00 Actual position
  0x606C:00 Actual velocity
  0x6077:00 Actual torque
  0x6061:00 Mode display
```

ATI sensor slave `3`:

```text
RxPDO 0x1601:
  0x7010:01 Control 1
  0x7010:02 Control 2

TxPDO 0x1A00:
  0x6000:01 Fx
  0x6000:02 Fy
  0x6000:03 Fz
  0x6000:04 Tx
  0x6000:05 Ty
  0x6000:06 Tz
  0x6010:00 Status code
  0x6020:00 Sample counter
```

### 3. Resets EtherCAT AL state

The tool calls:

```cpp
ecrt_master_reset(master)
```

This happens after master activation, because IgH documents `ecrt_master_reset()` as an activated-master operation. It asks the master to recover slaves from EtherCAT AL errors.

### 4. Clears CoE emergency history

For the three DS402 drives, it tries to write `0` to:

```text
0x1003:00
```

This clears the predefined error field / emergency history if the object is supported by the drive.

Failure to clear `0x1003:00` is printed but not treated as fatal, because not all devices allow or support this write.

### 5. Pulses DS402 fault reset

For drives `0`, `1`, `2`, the tool writes a rising edge on controlword bit 7 when the statusword decodes to CiA-402 `FAULT`:

```text
0x0000 -> 0x0080 -> 0x0000
```

The tool waits through `FAULT_REACTION_ACTIVE` and only starts the pulse once the drive reaches `FAULT`. This resets DS402 faults if the underlying cause has disappeared.

### 6. Keeps PDO communication alive

The cyclic loop continuously calls:

```cpp
ecrt_master_receive(master);
ecrt_domain_process(domain);

// When --dc is enabled:
ecrt_master_application_time(master, app_time);
ecrt_master_sync_reference_clock(master);
ecrt_master_sync_slave_clocks(master);

ecrt_domain_queue(domain);
ecrt_master_send(master);
```

It also writes neutral values to drive outputs and ATI control PDOs.

---

## Expected output

During normal operation, the tool periodically prints something like:

```text
Master: slaves=4, al_states=0x08, link=1 | Domain WC=12, WC state=2
  Drive 0: CW=0x0000 SW=0x0640
  Drive 1: CW=0x0000 SW=0x0640
  Drive 2: CW=0x0000 SW=0x0640
  ATI: status=0x00000000 counter=12345
```

Good signs:

```text
slaves=4
Domain WC=12
al_states contains OP
no "No response"
no "Failed to fetch SII"
no "Failed to get DC times"
```

---

## Troubleshooting

### Program cannot request master

Possible cause: another application already owns master `1`.

Stop the normal EtherCAT application, then run again.

Check:

```bash
sudo systemctl status ethercat
ps aux | grep ethercat
```

---

### Slaves return to fault after reset

If faults clear briefly and then come back, the underlying fault is still present.

Check:

```bash
ethercat -m 1 slaves
ethercat -m 1 domain
dmesg -w
```

Look for:

```text
DC Sync Timeout Error
Sync manager watchdog
No response
Failed to fetch SII contents
Failed to get DC times
Working counter changed to less than 12/12
```

If these appear, the problem is likely EtherCAT communication, DC timing, slave power, or cabling.

---

### Working counter is below 12/12

Expected working counter for this configured domain is:

```text
12/12
```

If the working counter drops, process data is not being acknowledged by all expected slaves.

Common causes:

```text
bad EtherCAT cable
bad connector
slave power loss
wrong slave order
wrong PDO mapping
wrong DC cycle time
cyclic task jitter
another application interfering with master
```

---

### Slave count changes between 2 and 4

This is a bus-level problem.

Focus on the section:

```text
master → slave 0 → slave 1 → slave 2 → slave 3
```

If the master often reports only two slaves, check especially:

```text
EtherCAT OUT of slave 1
cable between slave 1 and slave 2
EtherCAT IN of slave 2
logic power of slave 2
slave 2 hardware
```

---

### `0x6040`, `0x6041`, `0x603F` do not exist on slave 3

This is expected.

Slave 3 is the ATI Axia F/T sensor, not a DS402 drive. It does not expose DS402 objects.

The tool only applies DS402 reset logic to slaves `0`, `1`, and `2`.

---

### DC Sync Timeout Error

Example:

```text
AL status message 0x0034: "DC Sync Timeout Error"
```

Try running without DC first:

```bash
sudo ./reset_igh_ds402 --cycle-ns 1000000
```

Then try with DC:

```bash
sudo ./reset_igh_ds402 --cycle-ns 1000000 --dc --dc-assign 0x0300
```

If DC still fails, verify:

```text
correct cycle time
correct assignActivate value
real-time scheduling
stable cyclic loop
drive DC support/configuration
slave 0 suitable as reference clock
```

---

### Sync Manager watchdog

Example:

```text
AL status message 0x001B: "Sync manager watchdog"
```

This means a slave expected cyclic process data and did not receive it in time.

Possible causes:

```text
cyclic application stopped
cycle period too slow
jitter too high
slave output watchdog too strict
EtherCAT frames lost
bus communication unstable
```

---

## Recommended diagnostic commands

Check slaves:

```bash
ethercat -m 1 slaves
ethercat -m 1 slaves -v
```

Check domain:

```bash
ethercat -m 1 domain
```

Check DS402 status of drives:

```bash
for p in 0 1 2; do
  echo "---- slave $p ----"
  ethercat -m 1 upload -p $p --type uint16 0x6040 0
  ethercat -m 1 upload -p $p --type uint16 0x6041 0
  ethercat -m 1 upload -p $p --type uint16 0x603F 0
done
```

Watch live kernel messages:

```bash
sudo dmesg -w
```

Watch slave/domain state:

```bash
watch -n 0.5 'ethercat -m 1 slaves; echo; ethercat -m 1 domain'
```

---

## Notes

This tool is a recovery/debug utility, not a replacement for the normal robot controller.

If the network has a real physical or timing issue, the tool may reset the faults temporarily, but the slaves will fault again.

The stable target condition is:

```text
4 slaves responding
all required slaves in OP
Domain WC = 12/12
no DC timeout
no Sync Manager watchdog
no SII/DC "No response"
```

```
```
