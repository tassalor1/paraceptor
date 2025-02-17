#!/usr/bin/env python3
import pymavlink.mavutil as mavutil
from datetime import datetime
import curses
import threading
import time

class JetsonMonitor:
    def __init__(self):
        self.stats = [0, 0, 0, 0, 0, 0]  # [cpu, ram, gpu, fan, cpu_temp, gpu_temp]
        self.cv_messages = []
        self.max_messages = 10
        self.lock = threading.Lock()

    def update_stats(self, new_stats):
        with self.lock:
            self.stats = new_stats

    def add_cv_message(self, message):
        with self.lock:
            self.cv_messages.insert(0, message)
            if len(self.cv_messages) > self.max_messages:
                self.cv_messages.pop()

    def draw_stats(self, stdscr):
        def create_bar(value, width=20):
            filled = int(width * value / 100)
            return '[' + '#' * filled + '-' * (width - filled) + ']'

        while True:
            try:
                with self.lock:
                    stdscr.clear()
                    stdscr.addstr(0, 0, "Jetson System Monitor", curses.A_BOLD)
                    
                    stdscr.addstr(2, 0, "System Statistics:")
                    stdscr.addstr(3, 0, f"CPU Usage:    {create_bar(self.stats[0])} {self.stats[0]:.1f}%")
                    ram_mb = self.stats[1] / 1024
                    stdscr.addstr(4, 0, f"RAM Usage:    {ram_mb:.1f} MB")
                    stdscr.addstr(5, 0, f"GPU Usage:    {create_bar(self.stats[2])} {self.stats[2]:.1f}%")
                    stdscr.addstr(6, 0, f"Fan Speed:    {create_bar(self.stats[3])} {self.stats[3]:.1f}%")
                    stdscr.addstr(7, 0, f"CPU Temp:     {self.stats[4]:.1f}°C")
                    stdscr.addstr(8, 0, f"GPU Temp:     {self.stats[5]:.1f}°C")

                    stdscr.addstr(10, 0, "=" * 80)
                    
                    stdscr.addstr(11, 0, "Recent Messages:", curses.A_BOLD)
                    for i, msg in enumerate(self.cv_messages):
                        if i < curses.LINES - 13:
                            stdscr.addstr(13 + i, 0, msg)

                    stdscr.refresh()

            except curses.error:
                pass
            
            time.sleep(0.05)  #

def main():
    # Initialize MAVLink with shorter timeout
    mav = mavutil.mavlink_connection(
        '/dev/ttyUSB0',
        baud=57600,
        source_system=255
    )
    
    print("Connecting to MAVLink...")
    monitor = JetsonMonitor()
    
    def run_curses(stdscr):
        curses.curs_set(0)
        curses.start_color()
        stdscr.nodelay(1)
        
        # Start display thread
        display_thread = threading.Thread(target=monitor.draw_stats, args=(stdscr,))
        display_thread.daemon = True
        display_thread.start()
        
        # Main message processing loop
        while True:
            msg = mav.recv_match(type='STATUSTEXT', blocking=False)  
            if msg is not None and msg.text:
                try:
                    # Try to parse as list of numbers
                    text = msg.text.strip()
                    if text.startswith('[') and text.endswith(']'):
                        data = eval(text)
                        if isinstance(data, list) and len(data) == 6:
                            monitor.update_stats(data)
                    else:
                        timestamp = datetime.now().strftime("%H:%M:%S")
                        monitor.add_cv_message(f"[{timestamp}] {text}")
                except:
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    monitor.add_cv_message(f"[{timestamp}] {text}")
            
            time.sleep(0.01)  

    try:
        curses.wrapper(run_curses)
    finally:
        mav.close()

if __name__ == '__main__':
    main()
