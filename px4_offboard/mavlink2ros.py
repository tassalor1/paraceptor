#!/usr/bin/env python3

import pymavlink.mavutil as mavutil
import json
from datetime import datetime

def main():
    # Initialize MAVLink connection
    # Replace '/dev/ttyUSB0' with your telemetry port
    mav = mavutil.mavlink_connection(
        '/dev/ttyUSB0',
        baud=57600
    )
    
    print("Waiting for MAVLink messages...")
    
    try:
        while True:
            # Check for new MAVLink messages
            msg = mav.recv_match(type='STATUSTEXT', blocking=True)
            if msg is not None:
                try:
                    # The message is already a string, no need to decode
                    stats_data = json.loads(msg.text)
                    
                    # Print formatted output
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    print(f"\n[{timestamp}] Received system stats:")
                    print(f"  CPU Average: {stats_data}")
                    
                except json.JSONDecodeError as e:
                    print(f"Error parsing JSON: {e}")
                    print(f"Raw message: {msg.text}")
                except KeyError as e:
                    print(f"Missing key in data: {e}")
                    print(f"Raw message: {msg.text}")
                except Exception as e:
                    print(f"Error processing message: {e}")
                    print(f"Raw message: {msg.text}")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        mav.close()

if __name__ == '__main__':
    main()
