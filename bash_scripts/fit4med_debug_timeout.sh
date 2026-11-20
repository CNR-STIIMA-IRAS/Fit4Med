#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    echo "Uso: bash $0 [--seconds SECONDI]  (predefinito: 600)"
    echo "Sul PC Fit4Med, con GUI collegata, robot fermo e motori disattivati."
    echo "Crea un rapporto .txt sulla scrivania. Ctrl+C termina e salva il rapporto."
    exit 0
fi

if [[ ! -r /opt/ros/jazzy/setup.bash ]] || ! source /opt/ros/jazzy/setup.bash; then
    echo "ROS 2 Jazzy non disponibile. Avvia questo script nel terminale del PC Fit4Med." >&2
    exit 1
fi

# Match fit4med-bringup@.service unless the shell already specifies these.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

umask 077
exec nice -n 10 /usr/bin/python3 -u - "$@" <<'PYTHON'
import argparse
import asyncio
from collections import Counter
from contextlib import contextmanager
from datetime import datetime
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time


SERVICE = "/controller_manager/list_controllers"
WS_URL = "ws://127.0.0.1:9090"
PERIOD = 1.0
ROS_TIMEOUT = 5.0
WS_TIMEOUT = 7.0
CHANNELS = ("ROS2", "ROSBRIDGE")
ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
CONSOLE_EVENTS = re.compile(
    r"list_controllers|rosbridge_websocket.*Client (?:connected|disconnected)"
    r"|Working counter|slave\(s\) responding|overrun|datagram.*timed out",
    re.IGNORECASE,
)


def timestamp():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def run_read_command(*args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"{args[0]}: {exc}"
    return "\n".join(text.strip() for text in (result.stdout, result.stderr) if text.strip())


def desktop():
    home = Path.home()
    path = Path(run_read_command("xdg-user-dir", "DESKTOP"))
    if path.is_absolute() and path.is_dir() and path != home:
        return path
    return next((path for path in (home / "Scrivania", home / "Desktop") if path.is_dir()), home)


@contextmanager
def native_probe():
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    rclpy.init(args=["--ros-args", "--disable-external-lib-logs"], context=context)
    try:
        node = rclpy.create_node(
            f"fit4med_debug_timeout_{os.getpid()}", context=context,
            enable_rosout=False, start_parameter_services=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        try:
            client = node.create_client(ListControllers, SERVICE)

            def probe(_request_id):
                if not client.service_is_ready():
                    visible = any(name == SERVICE for name, _ in node.get_service_names_and_types())
                    return "SERVICE_NOT_READY", f"visibile nel grafo ROS={visible}"
                future = client.call_async(ListControllers.Request())
                executor.spin_until_future_complete(future, ROS_TIMEOUT)
                if not future.done():
                    client.remove_pending_request(future)
                    return "TIMEOUT", ""
                response = future.result()
                return "OK", " ".join(f"{c.name}={c.state}" for c in response.controller)

            yield probe
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        context.try_shutdown()


@contextmanager
def rosbridge_probe():
    from tornado.ioloop import IOLoop
    from tornado.websocket import websocket_connect

    loop = IOLoop()
    connection = None

    def close():
        nonlocal connection
        if connection is not None:
            connection.close()
            connection = None

    async def exchange(request_id):
        nonlocal connection
        if connection is None:
            connection = await websocket_connect(WS_URL, connect_timeout=2, max_message_size=2**20)
        await connection.write_message(json.dumps(
            {"op": "call_service", "service": SERVICE, "args": {}, "id": request_id}
        ))
        while True:
            response = await connection.read_message()
            if response is None:
                raise ConnectionError("WebSocket chiuso")
            message = json.loads(response)
            if message.get("op") == "service_response" and message.get("id") == request_id:
                if not message.get("result"):
                    details = message.get("values")
                    error = str(details).lower()
                    status = "ROSBRIDGE_TIMEOUT" if "timeout" in error or "timed out" in error else "SERVICE_ERROR"
                    return status, details
                controllers = message.get("values", {}).get("controller", [])
                return "OK", " ".join(f"{c['name']}={c['state']}" for c in controllers)

    async def probe_once(request_id):
        try:
            async with asyncio.timeout(WS_TIMEOUT):
                return await exchange(request_id)
        except TimeoutError:
            close()
            return "WEBSOCKET_TIMEOUT", ""
        except Exception:
            close()
            raise

    def probe(request_id):
        return loop.run_sync(lambda: probe_once(request_id))

    try:
        yield probe
    finally:
        close()
        loop.close(all_fds=True)


def active_log_files():
    log_root = Path(os.environ.get("FIT4MED_LOG_ROOT") or Path.home() / ".ros/fit4med_log")
    candidates = [path for path in log_root.glob("run_*/starts") if path.is_dir()]
    if not candidates:
        return []
    starts = max(candidates, key=lambda path: path.parent.name)
    logs = []
    for label in ("rosbridge", "platform"):
        folders = sorted(starts.glob(f"*_{label}_*"))
        if folders:
            logs.append(folders[-1] / "console.log")
            if label == "platform":
                logs.extend(sorted(folders[-1].glob("ec_link_monitor_*.log"))[-1:])
    return [path for path in logs if path.is_file()]


def append_events(report, offsets, started, finished):
    report.write("\n===== Eventi EtherCAT dal journal del kernel =====\n")
    report.write(run_read_command(
        "journalctl", "-k", "--no-pager", "-o", "short-iso-precise",
        "--since", f"@{int(started)}", "--until", f"@{int(finished) + 1}",
        "--grep", "ethercat", "--case-sensitive=no", "--lines", "500",
    ) + "\n")
    report.write("\n===== Eventi aggiunti ai log Fit4Med durante il test =====\n")
    log_paths = sorted(set(offsets) | set(active_log_files()))
    if not log_paths:
        report.write("Nessun file di log attivo trovato in FIT4MED_LOG_ROOT.\n")
    for path in log_paths:
        report.write(f"--- {path} ---\n")
        try:
            with path.open("rb") as source:
                end = source.seek(0, os.SEEK_END)
                start = offsets.get(path, 0)
                if start > end:
                    start = 0
                if end - start > 256 * 1024:
                    report.write("Estratto limitato agli ultimi 256 KiB.\n")
                    start = end - 256 * 1024
                source.seek(start)
                rows = ANSI.sub("", source.read(end - start).decode(errors="replace")).splitlines()
        except OSError as exc:
            report.write(f"{exc}\n")
            continue
        selected = [row for row in rows if path.name != "console.log" or CONSOLE_EVENTS.search(row)]
        report.write("\n".join(selected) + "\n" if selected else "(nessun evento)\n")


def main():
    parser = argparse.ArgumentParser(description="Diagnostica Fit4Med del servizio list_controllers")
    parser.add_argument("--seconds", type=int, default=600, help="durata in secondi")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds deve essere maggiore di zero")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    started = time.time()
    offsets = {}
    for path in active_log_files():
        try:
            offsets[path] = path.stat().st_size
        except OSError:
            continue

    counts = {channel: Counter() for channel in CHANNELS}
    lock = threading.Lock()
    name = f"fit4med_debug_timeout_{datetime.now():%Y%m%d-%H%M%S}_{os.getpid()}.txt"
    path = desktop() / name

    with path.open("x", encoding="utf-8", buffering=1) as report:
        report.write(f"Fit4Med | {timestamp()} | {socket.gethostname()} | {os.uname().release}\n")
        for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"):
            report.write(f"{key}={os.environ[key]}\n")
        report.write(f"Servizio {SERVICE} | {WS_URL} | durata {args.seconds}s | periodo {PERIOD}s\n")
        report.write(run_read_command(
            "dpkg-query", "-W", "ros-jazzy-controller-manager",
            "ros-jazzy-rosbridge-server", "ros-jazzy-rosbridge-library", "ros-jazzy-rmw-fastrtps-cpp",
        ) + "\n\n")
        report.write("inizio\tcanale\tid\tesito\tms\tdettagli\n")

        def record(channel, request_id, when, begin, status, details):
            elapsed_ms = (time.monotonic() - begin) * 1_000
            with lock:
                counts[channel][status] += 1
                report.write(f"{when}\t{channel}\t{request_id}\t{status}\t{elapsed_ms:.2f}\t{details!r}\n")

        def sample(channel, create_probe):
            try:
                with create_probe() as probe:
                    seq = 0
                    while not stop.is_set() and time.monotonic() < deadline:
                        seq += 1
                        request_id = f"fit4med-debug-{channel.lower()}-{os.getpid()}-{seq}"
                        begin = time.monotonic()
                        when = timestamp()
                        try:
                            status, details = probe(request_id)
                        except Exception as exc:
                            status, details = "ERROR", f"{type(exc).__name__}: {exc}"
                        record(channel, request_id, when, begin, status, details)
                        stop.wait(max(0, min(begin + PERIOD, deadline) - time.monotonic()))
            except Exception as exc:
                record(channel, "-", timestamp(), time.monotonic(), "SETUP_ERROR",
                       f"{type(exc).__name__}: {exc}")

        print("Test Fit4Med in corso. Lascia la GUI collegata, il robot fermo e i motori disattivati.")
        print(f"Durata {args.seconds} secondi. Per finire prima e salvare, premi Ctrl+C.")
        print(f"Rapporto: {path}")
        deadline = time.monotonic() + args.seconds
        workers = [
            threading.Thread(target=sample, args=("ROS2", native_probe)),
            threading.Thread(target=sample, args=("ROSBRIDGE", rosbridge_probe)),
        ]
        for worker in workers:
            worker.start()
        while not stop.wait(min(60, max(0, deadline - time.monotonic()))):
            remaining = max(0, round(deadline - time.monotonic()))
            if remaining == 0:
                break
            print(f"Test in corso. Mancano circa {remaining} secondi.")
        for worker in workers:
            worker.join()

        report.write(f"\n===== Riepilogo | {timestamp()} =====\n")
        for channel in CHANNELS:
            summary = ", ".join(f"{status}={n}" for status, n in sorted(counts[channel].items()))
            report.write(f"{channel}: {summary or '(nessuna misura)'}\n")
        append_events(report, offsets, started, time.time())

    print(f"Test terminato. Invia questo file: {path}")
    if any(counts[channel]["OK"] == 0 for channel in CHANNELS):
        print("Almeno un canale non ha risposto. Invia comunque il rapporto.")


if __name__ == "__main__":
    main()
PYTHON
