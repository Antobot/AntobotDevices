#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import psutil
import time
import subprocess
import datetime
import signal
import sys
import os
import re


class ROSNodeMonitor:
    def __init__(self, log_file="ros_node_monitor.txt", interval=1):
        self.log_file = log_file
        self.interval = interval
        self.running = True
        self.process_cache = {}   # cache pid -> psutil.Process

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        print(f"\nReceived Ctrl+C, stopping monitor...")
        self.running = False

    def get_all_nodes(self):
        """Get all running ROS nodes"""
        try:
            result = subprocess.run(['rosnode', 'list'],
                                    capture_output=True, text=True)
            if result.returncode == 0:
                return result.stdout.strip().splitlines()
        except Exception as e:
            print(f"rosnode list failed: {e}")
        return []

    def get_node_pid(self, node_name):
        """Get PID of a given node"""
        try:
            result = subprocess.run(['rosnode', 'info', node_name],
                                    capture_output=True, text=True)
            if result.returncode == 0:
                match = re.search(r"Pid\s*:\s*(\d+)", result.stdout)
                if match:
                    return int(match.group(1))
        except Exception as e:
            print(f"rosnode info failed: {e}")
        return None

    def get_process_info(self, pid, node_name):
        """Get process information"""
        try:
            if pid not in self.process_cache:
                proc = psutil.Process(pid)
                proc.cpu_percent(None)  # warm up
                self.process_cache[pid] = proc

            process = self.process_cache[pid]
            cpu_percent = process.cpu_percent(interval=0)  # non-blocking

            memory_info = process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024
            memory_percent = process.memory_percent()

            return {
                'pid': pid,
                'node_name': node_name,
                'status': process.status(),
                'cpu_percent': cpu_percent,
                'memory_mb': memory_mb,
                'memory_percent': memory_percent,
                'create_time': datetime.datetime.fromtimestamp(process.create_time()),
                'num_threads': process.num_threads()
            }
        except psutil.NoSuchProcess:
            if pid in self.process_cache:
                del self.process_cache[pid]
            return None
        except Exception as e:
            print(f"failed to get process info (PID: {pid}): {e}")
            return None

    def write_log_header(self):
        with open(self.log_file, 'w', encoding='utf-8') as f:
            f.write(f"ROS Node Monitor\n")
            f.write(f"Start time: {datetime.datetime.now()}\n")
            f.write(f"Monitor interval: {self.interval}s\n")
            f.write("="*100 + "\n")
            f.write(f"{'Time':<20} {'Node':<30} {'PID':<8} {'CPU%':<8} "
                    f"{'Memory MB':<10} {'Memory%':<8} {'Threads':<8} {'Status':<10}\n")
            f.write("-"*100 + "\n")

    def write_log_entry(self, process_info):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(f"{timestamp:<20} {process_info['node_name']:<30} {process_info['pid']:<8} "
                   f"{process_info['cpu_percent']:<8.2f} {process_info['memory_mb']:<10.2f} "
                   f"{process_info['memory_percent']:<8.2f} {process_info['num_threads']:<8} "
                   f"{process_info['status']:<10}\n")

    def monitor(self):
        print(f"Log file: {os.path.abspath(self.log_file)}")
        print(f"Interval: {self.interval} seconds")
        print("Press Ctrl+C to stop\n")

        self.write_log_header()
        monitor_count = 0

        while self.running:
            try:
                nodes = self.get_all_nodes()
                if not nodes:
                    print("[WARN] No running nodes found")
                for node in nodes:
                    pid = self.get_node_pid(node)
                    if pid:
                        process_info = self.get_process_info(pid, node)
                        if process_info:
                            monitor_count += 1
                            current_time = datetime.datetime.now().strftime("%H:%M:%S")
                            print(f"[{current_time}] {node:<30} "
                                  f"PID: {process_info['pid']:<6} "
                                  f"CPU: {process_info['cpu_percent']:<6.2f}% "
                                  f"Mem: {process_info['memory_mb']:<6.2f}MB "
                                  f"({process_info['memory_percent']:<5.2f}%) "
                                  f"Threads: {process_info['num_threads']}")
                            self.write_log_entry(process_info)

                time.sleep(self.interval)

            except Exception as e:
                print(f"Monitor error: {e}")
                time.sleep(self.interval)

        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write("\n" + "="*100 + "\n")
            f.write(f"End time: {datetime.datetime.now()}\n")
            f.write(f"Total records: {monitor_count}\n")

        print(f"\nMonitor stopped, {monitor_count} records written")
        print(f"Saved to: {os.path.abspath(self.log_file)}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='ROS Node CPU & Memory Monitor')
    parser.add_argument('-f', '--file', default='ros_node_monitor.txt',
                       help='log file name (default: ros_node_monitor.txt)')
    parser.add_argument('-i', '--interval', type=float, default=1.0,
                       help='monitor interval (default: 1.0 seconds)')

    args = parser.parse_args()

    try:
        with open(args.file, 'w') as f:
            pass
    except Exception as e:
        print(f"Cannot create log file {args.file}: {e}")
        return 1

    monitor = ROSNodeMonitor(args.file, args.interval)
    monitor.monitor()
    return 0


if __name__ == "__main__":
    sys.exit(main())

