#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import psutil
import time
import subprocess
import datetime
import argparse
import signal
import sys
import os
import re


class ROSNodeMonitor:
    def __init__(self, node_name, log_file="ros_node_monitor.txt", interval=1):
        self.node_name = node_name
        self.log_file = log_file
        self.interval = interval
        self.running = True

        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        """ctrl + c"""
        print(f"\nReceived Ctrl C, stopping monitor...")
        self.running = False

    def get_node_pid_from_rosnode(self):
        
        try:
            result = subprocess.run(
                ['rosnode', 'info', self.node_name],
                capture_output=True, text=True
            )
            if result.returncode == 0:
                match = re.search(r"Pid\s*:\s*(\d+)", result.stdout)
                if match:
                    return [int(match.group(1))]
        except Exception as e:
            print(f"rosnode info get PID faile: {e}")
        return []

    def get_node_processes(self):
        
        
        pids = self.get_node_pid_from_rosnode()

        if not pids:
            try:
                result = subprocess.run(
                    ['pgrep', '-f', self.node_name],
                    capture_output=True, text=True
                )
                if result.stdout.strip():
                    pids = [int(pid) for pid in result.stdout.strip().split('\n') if pid.isdigit()]
            except Exception as e:
                print(f"pgrep get PID faile: {e}")

        return pids

    def get_process_info(self, pid):
        
        try:
            process = psutil.Process(pid)

            
            cpu_percent = process.cpu_percent(interval=0.1)

            
            memory_info = process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024  #MB
            memory_percent = process.memory_percent()

            process_info = {
                'pid': pid,
                'name': process.name(),
                'status': process.status(),
                'cpu_percent': cpu_percent,
                'memory_mb': memory_mb,
                'memory_percent': memory_percent,
                'create_time': datetime.datetime.fromtimestamp(process.create_time()),
                'num_threads': process.num_threads()
            }
            return process_info

        except psutil.NoSuchProcess:
            return None
        except Exception as e:
            print(f"get PID faile (PID: {pid}): {e}")
            return None

    def write_log_header(self):
        with open(self.log_file, 'w', encoding='utf-8') as f:
            f.write(f"ROS node monitor\n")
            f.write(f"node name: {self.node_name}\n")
            f.write(f"start time: {datetime.datetime.now()}\n")
            f.write(f"monitor interva: {self.interval}s\n")
            f.write("="*80 + "\n")
            f.write(f"{'Time':<20} {'PID':<8} {'CPU%':<8} {'Memory MB':<10} {'Memory%':<8} {'Threads':<8} {'Status':<10}\n")
            f.write("-"*80 + "\n")

    def write_log_entry(self, process_info):
        
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(f"{timestamp:<20} {process_info['pid']:<8} {process_info['cpu_percent']:<8.2f} "
                   f"{process_info['memory_mb']:<10.2f} {process_info['memory_percent']:<8.2f} "
                   f"{process_info['num_threads']:<8} {process_info['status']:<10}\n")

    def monitor(self):
        
        print(f"start : {self.node_name}")
        print(f"log: {os.path.abspath(self.log_file)}")
        print(f"interval: {self.interval}秒")
        print("Ctrl+C to stop\n")

        
        self.write_log_header()

        monitor_count = 0
        no_process_count = 0

        while self.running:
            try:
                pids = self.get_node_processes()

                if not pids:
                    no_process_count += 1
                    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                    if no_process_count == 1: 
                        print(f"[{current_time}] cant find '{self.node_name}' process")
                        with open(self.log_file, 'a', encoding='utf-8') as f:
                            f.write(f"{current_time:<20} {'N/A':<8} {'N/A':<8} {'N/A':<10} {'N/A':<8} {'N/A':<8} {'no running':<10}\n")

                    if no_process_count % 10 == 0:
                        print(f"[{current_time}] node '{self.node_name}' still stop (have waitting for {no_process_count}times)")

                else:
                    no_process_count = 0

                    for pid in pids:
                        process_info = self.get_process_info(pid)
                        if process_info:
                            monitor_count += 1
                            current_time = datetime.datetime.now().strftime("%H:%M:%S")
                            print(f"[{current_time}] PID: {process_info['pid']:<6} "
                                  f"CPU: {process_info['cpu_percent']:<6.2f}% "
                                  f"Memory: {process_info['memory_mb']:<8.2f}MB "
                                  f"({process_info['memory_percent']:<5.2f}%) "
                                  f"Threads: {process_info['num_threads']}")
                            self.write_log_entry(process_info)

                time.sleep(self.interval)

            except Exception as e:
                print(f"monitor error: {e}")
                time.sleep(self.interval)

        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write("\n" + "="*80 + "\n")
            f.write(f"finish monitor time: {datetime.datetime.now()}\n")
            f.write(f"monitor times: {monitor_count}\n")

        print(f"\nmonitor stopped，record {monitor_count} datas")
        print(f"Saved in: {os.path.abspath(self.log_file)}")


def main():
    parser = argparse.ArgumentParser(description='monitor CPU && Memory')
    parser.add_argument('node_name', help='node name which to monitor')
    parser.add_argument('-f', '--file', default='ros_node_monitor.txt',
                       help='log name (ros_node_monitor.txt)')
    parser.add_argument('-i', '--interval', type=float, default=1.0,
                       help='interval (normal: 1.0)')

    args = parser.parse_args()

    try:
        with open(args.file, 'w') as f:
            pass
    except Exception as e:
        print(f"cant create log file {args.file}: {e}")
        return 1

    monitor = ROSNodeMonitor(args.node_name, args.file, args.interval)
    monitor.monitor()
    return 0


if __name__ == "__main__":
    sys.exit(main())

