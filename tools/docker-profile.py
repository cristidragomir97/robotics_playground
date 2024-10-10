#!/usr/bin/env python3

import docker
import time
import sys
import shutil
from datetime import datetime

def clear_terminal():
    """Clears the terminal screen."""
    print("\033[H\033[J", end="")

def bytes_to_mib(bytes_value):
    """Converts bytes to mebibytes (MiB)."""
    return bytes_value / (1024 * 1024)

def get_terminal_size():
    """Returns the terminal size."""
    size = shutil.get_terminal_size(fallback=(80, 24))
    return size.columns, size.lines

def format_value(value, unit=''):
    """Formats the value to two decimal places with optional unit."""
    return f"{value:.2f}{unit}"

def main(interval=2):
    client = docker.from_env()
    while True:
        clear_terminal()
        columns, _ = get_terminal_size()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"Docker Container Resource Usage - {timestamp}\n")

        # Print header
        header = f"{'CONTAINER ID':<12} {'NAME':<20} {'CPU (%)':<8} {'MEM USAGE (MiB)':<17} {'MEM LIMIT (MiB)':<17} {'MEM (%)':<8} {'DISK USAGE':<12} {'NET I/O (RX/TX)':<20} {'BLOCK I/O (R/W)':<20}"
        print(header)
        print('-' * min(columns, len(header)))

        # Get list of running containers
        containers = client.containers.list()

        for container in containers:
            try:
                # Get container stats
                stats = container.stats(stream=False)
                cpu_percent = calculate_cpu_percent(stats)
                mem_usage = stats['memory_stats']['usage']
                mem_limit = stats['memory_stats']['limit']
                mem_percent = (mem_usage / mem_limit) * 100 if mem_limit > 0 else 0

                mem_usage_mib = bytes_to_mib(mem_usage)
                mem_limit_mib = bytes_to_mib(mem_limit)

                # Get network I/O
                net_io = stats['networks']
                total_rx = 0
                total_tx = 0
                for iface in net_io.values():
                    total_rx += iface['rx_bytes']
                    total_tx += iface['tx_bytes']
                net_io_formatted = f"{format_value(bytes_to_mib(total_rx), 'MiB')} / {format_value(bytes_to_mib(total_tx), 'MiB')}"

                # Get block I/O
                blkio_stats = stats['blkio_stats']['io_service_bytes_recursive']
                blk_read = 0
                blk_write = 0
                for blk in blkio_stats:
                    if blk['op'] == 'Read':
                        blk_read += blk['value']
                    elif blk['op'] == 'Write':
                        blk_write += blk['value']
                blk_io_formatted = f"{format_value(bytes_to_mib(blk_read), 'MiB')} / {format_value(bytes_to_mib(blk_write), 'MiB')}"

                # Get disk usage (size of writable layer)
                container_info = container.attrs
                disk_usage = container_info['SizeRw']
                disk_usage_formatted = f"{format_value(bytes_to_mib(disk_usage), 'MiB')}"

                # Print container stats
                print(f"{container.short_id:<12} {container.name:<20} {format_value(cpu_percent):<8} {format_value(mem_usage_mib):<17} {format_value(mem_limit_mib):<17} {format_value(mem_percent):<8} {disk_usage_formatted:<12} {net_io_formatted:<20} {blk_io_formatted:<20}")
            except Exception as e:
                print(f"Error fetching stats for container {container.name}: {e}")

        # Wait for the specified interval before refreshing
        time.sleep(interval)

def calculate_cpu_percent(stats):
    """Calculates the CPU usage percentage."""
    cpu_delta = stats['cpu_stats']['cpu_usage']['total_usage'] - stats['precpu_stats']['cpu_usage']['total_usage']
    system_delta = stats['cpu_stats']['system_cpu_usage'] - stats['precpu_stats']['system_cpu_usage']
    cpu_percent = 0.0

    if system_delta > 0.0 and cpu_delta > 0.0:
        cpu_percent = (cpu_delta / system_delta) * len(stats['cpu_stats']['cpu_usage']['percpu_usage']) * 100.0

    return cpu_percent

if __name__ == '__main__':
    try:
        # You can adjust the interval as needed
        main(interval=2)
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)
