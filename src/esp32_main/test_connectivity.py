#!/usr/bin/env python3
"""
Simplified ESP32 Test - Basic Connectivity Check
"""

import socket
import json
import time
import sys

def test_basic_connectivity():
    """Test basic UDP communication with ESP32"""
    
    print("=" * 60)
    print("ESP32 Basic Connectivity Test")
    print("=" * 60)
    print()
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 5556))
    sock.setblocking(False)
    
    print("[*] Socket created and bound to port 5556")
    print("[*] Waiting for ESP32 handshake (timeout: 10 seconds)...")
    print()
    
    # Wait for handshake
    start_time = time.time()
    handshake_received = False
    
    while time.time() - start_time < 10:
        try:
            data, addr = sock.recvfrom(1024)
            print(f"[+] Received from {addr}:")
            print(f"    Raw: {data.decode('utf-8')}")
            print()
            
            # Try to parse as JSON
            try:
                msg = json.loads(data.decode('utf-8'))
                print(f"[+] Parsed JSON: {json.dumps(msg, indent=2)}")
                print()
                
                if msg.get('topic') == 'system' and msg.get('payload', {}).get('type') == 'handshake':
                    handshake_received = True
                    print("[✓] HANDSHAKE CONFIRMED!")
                    print(f"    Device: {msg['payload'].get('device')}")
                    print(f"    IP: {msg['payload'].get('ip')}")
                    print(f"    MAC: {msg['payload'].get('mac')}")
                    break
            except json.JSONDecodeError as e:
                print(f"[!] JSON parse error: {e}")
                print()
        
        except socket.error:
            pass
        
        # Print status
        elapsed = time.time() - start_time
        if int(elapsed) % 1 == 0 and elapsed > 0:
            remaining = 10 - int(elapsed)
            print(f"[...] Waiting... {remaining} seconds remaining", end='\r')
        
        time.sleep(0.1)
    
    print()
    print()
    
    if handshake_received:
        print("=" * 60)
        print("[SUCCESS] ESP32 is communicating!")
        print("=" * 60)
        print()
        print("Testing message publishing...")
        print()
        
        # Send test message
        test_payload = {
            "topic": "led",
            "payload": {"state": "listen"}
        }
        
        msg = json.dumps(test_payload)
        print(f"[*] Sending: {msg}")
        
        try:
            sock.sendto(msg.encode('utf-8'), ('<broadcast>', 5555))
            print("[+] Message sent successfully")
            print()
        except Exception as e:
            print(f"[!] Error sending: {e}")
        
        time.sleep(1)
        
        print()
        print("=" * 60)
        print("[✓] All connectivity tests passed!")
        print("=" * 60)
        return True
    
    else:
        print("=" * 60)
        print("[FAIL] No handshake received from ESP32")
        print("=" * 60)
        print()
        print("Possible issues:")
        print("1. ESP32 not powered on or reset")
        print("2. Ethernet cable not connected")
        print("3. Network doesn't support UDP broadcast")
        print("4. DHCP server not available")
        print("5. Firewall blocking UDP port 5555/5556")
        print()
        print("Troubleshooting steps:")
        print("1. Check serial monitor: platformio device monitor")
        print("2. Verify Ethernet connection")
        print("3. Confirm DHCP is available on network")
        print("4. Check Windows firewall UDP rules")
        print()
        return False
    
    sock.close()

if __name__ == '__main__':
    success = test_basic_connectivity()
    sys.exit(0 if success else 1)
