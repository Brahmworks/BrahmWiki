#!/usr/bin/env python3
"""
ESP32 Robot Controller Pub/Sub Test Client

This script communicates with the ESP32 robot controller via UDP
to test the pub/sub endpoints.

Usage:
    python3 test_client.py [--esp32-ip IP] [--test-name test_name]
"""

import socket
import json
import time
import argparse
import sys
from datetime import datetime

class ESP32RobotClient:
    """Client for communicating with ESP32 robot via pub/sub"""
    
    def __init__(self, esp32_ip=None, local_port=5556, remote_port=5555):
        """
        Initialize the client
        
        Args:
            esp32_ip: IP address of ESP32 (None for broadcast)
            local_port: Local UDP port to listen on
            remote_port: Remote UDP port ESP32 listens on
        """
        self.esp32_ip = esp32_ip or '<broadcast>'
        self.local_port = local_port
        self.remote_port = remote_port
        self.socket = None
        self.received_messages = []
        
    def connect(self):
        """Create UDP socket and enable broadcasting"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to local port for receiving
        self.socket.bind(('', self.local_port))
        self.socket.setblocking(False)  # Non-blocking for receiving
        
        print(f"[{self._timestamp()}] Client initialized")
        print(f"[{self._timestamp()}] Listening on port {self.local_port}")
        print(f"[{self._timestamp()}] Sending to {self.remote_port}")
        
    def disconnect(self):
        """Close socket"""
        if self.socket:
            self.socket.close()
            
    def _timestamp(self):
        """Get current timestamp"""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    def _send_message(self, msg_dict):
        """Send a message to ESP32"""
        try:
            msg_json = json.dumps(msg_dict)
            self.socket.sendto(
                msg_json.encode('utf-8'),
                ('<broadcast>', self.remote_port)
            )
            print(f"[{self._timestamp()}] TX: {msg_json}")
        except Exception as e:
            print(f"[{self._timestamp()}] ERROR sending message: {e}")
    
    def _receive_messages(self, timeout=0.1):
        """Receive incoming messages"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = self.socket.recvfrom(1024)
                msg_str = data.decode('utf-8')
                msg = json.loads(msg_str)
                self.received_messages.append(msg)
                print(f"[{self._timestamp()}] RX from {addr}: {msg_str}")
            except socket.error:
                pass  # No data available
            except json.JSONDecodeError as e:
                print(f"[{self._timestamp()}] ERROR parsing JSON: {e}")
    
    def publish(self, topic, payload):
        """Publish a message to a topic"""
        msg = {
            "topic": topic,
            "payload": payload
        }
        self._send_message(msg)
    
    def wait_for_response(self, topic=None, timeout=1.0):
        """Wait for incoming message"""
        self._receive_messages(timeout)
        
        if topic:
            for msg in self.received_messages:
                if msg.get('topic') == topic:
                    return msg
            return None
        elif self.received_messages:
            return self.received_messages.pop(0)
        return None
    
    def test_handshake(self):
        """Wait for ESP32 handshake"""
        print("\n=== TEST: Waiting for ESP32 Handshake ===")
        msg = self.wait_for_response(timeout=5.0)
        if msg and msg.get('topic') == 'system' and msg.get('payload', {}).get('type') == 'handshake':
            print(f"[{self._timestamp()}] SUCCESS: Received handshake from {msg['payload']['device']}")
            print(f"  MAC: {msg['payload'].get('mac')}")
            print(f"  IP:  {msg['payload'].get('ip')}")
            return True
        else:
            print(f"[{self._timestamp()}] TIMEOUT: No handshake received")
            return False
    
    def test_torso_command(self):
        """Test torso motor command"""
        print("\n=== TEST: Torso Command ===")
        payload = {"angle": 90, "speed": 500, "accel": 30}
        self.publish("torso_cmd", payload)
        
        time.sleep(0.2)
        self._receive_messages(timeout=0.5)
        
        print(f"[{self._timestamp()}] Command sent: {payload}")
    
    def test_neck_command(self):
        """Test neck motor command"""
        print("\n=== TEST: Neck Command ===")
        payload = {"angle": 15, "speed": 400, "accel": 25}
        self.publish("neck_cmd", payload)
        
        time.sleep(0.2)
        self._receive_messages(timeout=0.5)
        
        print(f"[{self._timestamp()}] Command sent: {payload}")
    
    def test_head_command(self):
        """Test head motor command"""
        print("\n=== TEST: Head Command ===")
        payload = {"angle": -10, "speed": 300, "accel": 20}
        self.publish("head_cmd", payload)
        
        time.sleep(0.2)
        self._receive_messages(timeout=0.5)
        
        print(f"[{self._timestamp()}] Command sent: {payload}")
    
    def test_touch_publish(self):
        """Test publishing touch event"""
        print("\n=== TEST: Touch Event Publication ===")
        for state in ["tap", "double", "up", "down", "long"]:
            payload = {"state": state}
            self.publish("touch", payload)
            time.sleep(0.1)
        print(f"[{self._timestamp()}] Touch events published")
    
    def test_led_publish(self):
        """Test publishing LED states"""
        print("\n=== TEST: LED State Publication ===")
        states = ["listen", "mute", "loading", "offline", "connecting", "low_battery"]
        for state in states:
            payload = {"state": state}
            self.publish("led", payload)
            time.sleep(0.1)
        print(f"[{self._timestamp()}] LED states published")
    
    def test_battery_publish(self):
        """Test publishing battery info"""
        print("\n=== TEST: Battery Info Publication ===")
        payload = {"batt": 85, "ps": True, "hb_out": 42}
        self.publish("batt", payload)
        print(f"[{self._timestamp()}] Battery info published: {payload}")
        time.sleep(0.2)
    
    def test_reboot_signal(self):
        """Test reboot signal publication"""
        print("\n=== TEST: Reboot Signal Publication ===")
        payload = {"reb_jet": False}  # False for testing, True to actually reboot
        self.publish("reb_jet", payload)
        print(f"[{self._timestamp()}] Reboot signal published: {payload}")
        time.sleep(0.2)
    
    def run_all_tests(self):
        """Run all tests"""
        print("=" * 60)
        print("ESP32 Robot Controller Pub/Sub Test Suite")
        print("=" * 60)
        
        # Wait for handshake
        if not self.test_handshake():
            print("\n[!] ESP32 not responding. Make sure it's running and connected.")
            return False
        
        # Run all tests
        time.sleep(0.5)
        self.test_torso_command()
        time.sleep(0.3)
        
        self.test_neck_command()
        time.sleep(0.3)
        
        self.test_head_command()
        time.sleep(0.3)
        
        self.test_touch_publish()
        time.sleep(0.3)
        
        self.test_led_publish()
        time.sleep(0.3)
        
        self.test_battery_publish()
        time.sleep(0.3)
        
        self.test_reboot_signal()
        
        print("\n" + "=" * 60)
        print("All tests completed!")
        print("=" * 60)
        return True


def main():
    parser = argparse.ArgumentParser(
        description='ESP32 Robot Controller Pub/Sub Test Client',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Test with broadcast (auto-discover ESP32)
  python3 test_client.py
  
  # Test with specific ESP32 IP
  python3 test_client.py --esp32-ip 192.168.1.100
  
  # Run specific test
  python3 test_client.py --test torso
        '''
    )
    parser.add_argument('--esp32-ip', help='ESP32 IP address (broadcast if not specified)')
    parser.add_argument('--test', choices=['torso', 'neck', 'head', 'touch', 'led', 'battery', 'reboot'],
                        help='Run a specific test instead of all')
    
    args = parser.parse_args()
    
    client = ESP32RobotClient(esp32_ip=args.esp32_ip)
    
    try:
        client.connect()
        
        if args.test:
            # Run single test
            if args.test == 'torso':
                client.test_torso_command()
            elif args.test == 'neck':
                client.test_neck_command()
            elif args.test == 'head':
                client.test_head_command()
            elif args.test == 'touch':
                client.test_touch_publish()
            elif args.test == 'led':
                client.test_led_publish()
            elif args.test == 'battery':
                client.test_battery_publish()
            elif args.test == 'reboot':
                client.test_reboot_signal()
        else:
            # Run all tests
            client.run_all_tests()
        
        # Keep listening for a bit longer
        print("\nListening for responses for 2 seconds...")
        time.sleep(2)
        client.wait_for_response(timeout=0.1)
        
    except KeyboardInterrupt:
        print("\n[!] Test interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        client.disconnect()
        print("\nClient disconnected")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
