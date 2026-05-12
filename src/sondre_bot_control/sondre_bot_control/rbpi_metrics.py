#!/usr/bin/env python3

import os
import re
import json
import time
import shutil
import subprocess
from typing import Dict, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RaspberryPiMetrics(Node):
    def __init__(self):
        super().__init__("rbpi_metrics")

        self.pub = self.create_publisher(String, "/rbpi/metrics", 10)

        self.prev_cpu = None
        self.prev_net = None
        self.prev_net_time = None

        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info("Raspberry Pi metrics publisher started on /rbpi/metrics")

    # --------------------------------------------------
    # Small helpers
    # --------------------------------------------------

    def run_cmd(self, cmd, timeout=0.5) -> str:
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=timeout,
                check=False,
            )
            return result.stdout.strip()
        except Exception:
            return ""

    def run_vcgencmd(self, args) -> str:
        out = self.run_cmd(["vcgencmd"] + args)

        if not out:
            out = self.run_cmd(["sudo", "-n", "vcgencmd"] + args)

        return out

    def read_file(self, path: str) -> str:
        try:
            with open(path, "r") as f:
                return f.read().strip()
        except Exception:
            return ""

    # --------------------------------------------------
    # Temperature / power / throttle
    # --------------------------------------------------

    def get_input_voltage_v(self) -> Optional[float]:
        out = self.run_vcgencmd(["pmic_read_adc", "EXT5V_V"])
        match = re.search(r"EXT5V_V\s+volt\(\d+\)=\s*([0-9.]+)V", out)
        if match:
            return float(match.group(1))
        return None

    def get_cpu_temp_c(self) -> Optional[float]:
        out = self.run_vcgencmd(["measure_temp"])
        match = re.search(r"temp=([0-9.]+)", out)
        if match:
            return float(match.group(1))

        raw = self.read_file("/sys/class/thermal/thermal_zone0/temp")
        if raw.isdigit():
            return float(raw) / 1000.0

        return None

    def get_cpu_clock_mhz(self) -> Optional[float]:
        out = self.run_vcgencmd(["measure_clock", "arm"])
        match = re.search(r"frequency\(.*?\)=([0-9]+)", out)
        if match:
            return int(match.group(1)) / 1_000_000.0

        raw = self.read_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        if raw.isdigit():
            return int(raw) / 1000.0

        return None

    def get_core_volts(self) -> Optional[float]:
        out = self.run_vcgencmd(["measure_volts", "core"])
        match = re.search(r"volt=([0-9.]+)", out)
        if match:
            return float(match.group(1))
        return None

    def get_throttled(self) -> Dict[str, Any]:
        out = self.run_vcgencmd(["get_throttled"])

        result = {
            "raw": None,
            "hex": "unknown",
            "undervoltage_now": False,
            "freq_capped_now": False,
            "throttled_now": False,
            "soft_temp_limit_now": False,
            "undervoltage_seen": False,
            "freq_capped_seen": False,
            "throttled_seen": False,
            "soft_temp_limit_seen": False,
            "ok": False,
            "warnings": [],
        }

        match = re.search(r"throttled=0x([0-9a-fA-F]+)", out)
        if not match:
            result["warnings"].append("vcgencmd get_throttled unavailable")
            return result

        value = int(match.group(1), 16)

        result["raw"] = value
        result["hex"] = f"0x{value:x}"

        result["undervoltage_now"] = bool(value & 0x1)
        result["freq_capped_now"] = bool(value & 0x2)
        result["throttled_now"] = bool(value & 0x4)
        result["soft_temp_limit_now"] = bool(value & 0x8)

        result["undervoltage_seen"] = bool(value & 0x10000)
        result["freq_capped_seen"] = bool(value & 0x20000)
        result["throttled_seen"] = bool(value & 0x40000)
        result["soft_temp_limit_seen"] = bool(value & 0x80000)

        if result["undervoltage_now"]:
            result["warnings"].append("UNDERVOLTAGE NOW")
        if result["throttled_now"]:
            result["warnings"].append("THROTTLING NOW")
        if result["freq_capped_now"]:
            result["warnings"].append("FREQUENCY CAPPED NOW")
        if result["soft_temp_limit_now"]:
            result["warnings"].append("SOFT TEMP LIMIT NOW")

        if result["undervoltage_seen"]:
            result["warnings"].append("undervoltage seen since boot")
        if result["throttled_seen"]:
            result["warnings"].append("throttling seen since boot")
        if result["freq_capped_seen"]:
            result["warnings"].append("frequency cap seen since boot")
        if result["soft_temp_limit_seen"]:
            result["warnings"].append("soft temp limit seen since boot")

        result["ok"] = value == 0
        return result

    # --------------------------------------------------
    # CPU
    # --------------------------------------------------

    def parse_cpu_line(self, line: str) -> Optional[Tuple[str, int, int]]:
        parts = line.split()
        if not parts or not parts[0].startswith("cpu"):
            return None

        name = parts[0]
        values = [int(x) for x in parts[1:]]
        idle = values[3] + values[4]
        total = sum(values)

        return name, idle, total

    def get_cpu_usage(self) -> Dict[str, Any]:
        lines = self.read_file("/proc/stat").splitlines()

        current = {}
        for line in lines:
            parsed = self.parse_cpu_line(line)
            if parsed:
                name, idle, total = parsed
                current[name] = (idle, total)

        usage = {
            "total_percent": None,
            "per_core_percent": {},
        }

        if self.prev_cpu is not None:
            for name, (idle, total) in current.items():
                if name in self.prev_cpu:
                    prev_idle, prev_total = self.prev_cpu[name]

                    delta_idle = idle - prev_idle
                    delta_total = total - prev_total

                    if delta_total > 0:
                        percent = 100.0 * (1.0 - (delta_idle / delta_total))

                        if name == "cpu":
                            usage["total_percent"] = percent
                        else:
                            usage["per_core_percent"][name] = percent

        self.prev_cpu = current
        return usage

    def get_load_average(self) -> Dict[str, float]:
        try:
            one, five, fifteen = os.getloadavg()
            return {
                "load_1m": one,
                "load_5m": five,
                "load_15m": fifteen,
            }
        except Exception:
            return {
                "load_1m": 0.0,
                "load_5m": 0.0,
                "load_15m": 0.0,
            }

    # --------------------------------------------------
    # Memory / disk
    # --------------------------------------------------

    def get_memory(self) -> Dict[str, Any]:
        meminfo = {}

        for line in self.read_file("/proc/meminfo").splitlines():
            if ":" not in line:
                continue

            key, value = line.split(":", 1)
            number = value.strip().split()[0]

            if number.isdigit():
                meminfo[key] = int(number) * 1024

        mem_total = meminfo.get("MemTotal", 0)
        mem_available = meminfo.get("MemAvailable", 0)
        swap_total = meminfo.get("SwapTotal", 0)
        swap_free = meminfo.get("SwapFree", 0)

        mem_used = max(0, mem_total - mem_available)
        swap_used = max(0, swap_total - swap_free)

        return {
            "ram_total_bytes": mem_total,
            "ram_used_bytes": mem_used,
            "ram_available_bytes": mem_available,
            "ram_used_percent": (100.0 * mem_used / mem_total) if mem_total else None,
            "swap_total_bytes": swap_total,
            "swap_used_bytes": swap_used,
            "swap_used_percent": (100.0 * swap_used / swap_total) if swap_total else 0.0,
        }

    def get_disk(self) -> Dict[str, Any]:
        disk = shutil.disk_usage("/")

        return {
            "root_total_bytes": disk.total,
            "root_used_bytes": disk.used,
            "root_free_bytes": disk.free,
            "root_used_percent": 100.0 * disk.used / disk.total,
        }

    # --------------------------------------------------
    # Network
    # --------------------------------------------------

    def get_network(self) -> Dict[str, Any]:
        now = time.time()
        interfaces = {}

        for line in self.read_file("/proc/net/dev").splitlines():
            if ":" not in line:
                continue

            name, data = line.split(":", 1)
            name = name.strip()

            if name == "lo":
                continue

            parts = data.split()
            if len(parts) < 16:
                continue

            rx_bytes = int(parts[0])
            rx_packets = int(parts[1])
            rx_errors = int(parts[2])
            rx_dropped = int(parts[3])

            tx_bytes = int(parts[8])
            tx_packets = int(parts[9])
            tx_errors = int(parts[10])
            tx_dropped = int(parts[11])

            interfaces[name] = {
                "rx_bytes": rx_bytes,
                "tx_bytes": tx_bytes,
                "rx_packets": rx_packets,
                "tx_packets": tx_packets,
                "rx_errors": rx_errors,
                "tx_errors": tx_errors,
                "rx_dropped": rx_dropped,
                "tx_dropped": tx_dropped,
                "rx_rate_bytes_s": 0.0,
                "tx_rate_bytes_s": 0.0,
            }

        if self.prev_net is not None and self.prev_net_time is not None:
            dt = max(0.001, now - self.prev_net_time)

            for name, values in interfaces.items():
                if name in self.prev_net:
                    prev = self.prev_net[name]
                    values["rx_rate_bytes_s"] = max(
                        0.0,
                        (values["rx_bytes"] - prev["rx_bytes"]) / dt,
                    )
                    values["tx_rate_bytes_s"] = max(
                        0.0,
                        (values["tx_bytes"] - prev["tx_bytes"]) / dt,
                    )

        self.prev_net = interfaces
        self.prev_net_time = now

        return interfaces

    # --------------------------------------------------
    # Other status
    # --------------------------------------------------

    def get_uptime(self) -> Dict[str, Any]:
        raw = self.read_file("/proc/uptime")
        try:
            seconds = float(raw.split()[0])
        except Exception:
            seconds = 0.0

        return {
            "uptime_seconds": seconds,
        }

    def get_top_processes(self) -> list:
        out = self.run_cmd(
            ["ps", "-eo", "pid,comm,%cpu,%mem", "--sort=-%cpu"],
            timeout=0.5,
        )

        processes = []

        for line in out.splitlines()[1:8]:
            parts = line.split(None, 3)
            if len(parts) != 4:
                continue

            pid, command, cpu, mem = parts

            try:
                processes.append(
                    {
                        "pid": int(pid),
                        "command": command,
                        "cpu_percent": float(cpu),
                        "mem_percent": float(mem),
                    }
                )
            except Exception:
                pass

        return processes

    def get_kernel_warning_count(self) -> Dict[str, Any]:
        out = self.run_cmd(
            ["dmesg", "--color=never"],
            timeout=0.8,
        )

        if not out:
            return {
                "available": False,
                "storage_error_count": None,
                "oom_error_count": None,
                "power_error_count": None,
            }

        storage_pattern = re.compile(
            r"mmc|i/o error|ext4-fs error|buffer i/o error|resetting high-speed",
            re.IGNORECASE,
        )
        oom_pattern = re.compile(
            r"out of memory|oom|killed process",
            re.IGNORECASE,
        )
        power_pattern = re.compile(
            r"under-voltage|undervoltage|voltage|thrott",
            re.IGNORECASE,
        )

        return {
            "available": True,
            "storage_error_count": len(storage_pattern.findall(out)),
            "oom_error_count": len(oom_pattern.findall(out)),
            "power_error_count": len(power_pattern.findall(out)),
        }

    # --------------------------------------------------
    # Publish
    # --------------------------------------------------

    def timer_cb(self):
        cpu_temp_c = self.get_cpu_temp_c()
        cpu_clock_mhz = self.get_cpu_clock_mhz()
        core_volts = self.get_core_volts()
        input_voltage_v = self.get_input_voltage_v()
        throttle = self.get_throttled()

        cpu = self.get_cpu_usage()
        load = self.get_load_average()
        memory = self.get_memory()
        disk = self.get_disk()
        network = self.get_network()
        uptime = self.get_uptime()
        top_processes = self.get_top_processes()
        kernel = self.get_kernel_warning_count()

        warnings = []

        if throttle.get("warnings"):
            warnings.extend(throttle["warnings"])

        if input_voltage_v is not None:
            if input_voltage_v < 4.8:
                warnings.append("5V input low")
            elif input_voltage_v < 4.9:
                warnings.append("5V input marginal")

        if cpu_temp_c is not None:
            if cpu_temp_c >= 85.0:
                warnings.append("CPU HOT")
            elif cpu_temp_c >= 75.0:
                warnings.append("CPU warm")

        if memory["ram_used_percent"] is not None and memory["ram_used_percent"] >= 90.0:
            warnings.append("RAM high")

        if memory["swap_used_percent"] is not None and memory["swap_used_percent"] >= 50.0:
            warnings.append("Swap high")

        if disk["root_used_percent"] >= 90.0:
            warnings.append("Disk nearly full")

        if kernel.get("storage_error_count"):
            warnings.append("Storage/kernel errors seen")

        status = "WARN" if warnings else "OK"

        payload = {
            "stamp": time.time(),
            "status": status,
            "warnings": warnings,
            "cpu_temp_c": cpu_temp_c,
            "cpu_clock_mhz": cpu_clock_mhz,
            "core_volts": core_volts,
            "input_voltage_v": input_voltage_v,
            "throttle": throttle,
            "cpu": cpu,
            "load": load,
            "memory": memory,
            "disk": disk,
            "network": network,
            "uptime": uptime,
            "top_processes": top_processes,
            "kernel": kernel,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RaspberryPiMetrics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()