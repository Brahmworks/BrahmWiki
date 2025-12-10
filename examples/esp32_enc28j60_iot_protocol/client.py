import socket
import time
import sys

def get_local_ip():
    try:
        # Connect to a dummy external IP to get the interface IP (doesn't send data)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "Unknown"

def main():
    print("UDP Client for ESP32 ENC28J60 Demo")
    print("-" * 40)
    
    local_ip = get_local_ip()
    print(f"Your PC IP Address: {local_ip}")
    
    target_ip = input("Enter ESP32 IP address (default 169.254.1.200): ").strip()
    if not target_ip:
        target_ip = "169.254.1.200"
        
    print("-" * 40)
    # Check subnet
    if local_ip != "Unknown":
        local_parts = local_ip.split('.')
        target_parts = target_ip.split('.')
        if len(local_parts) == 4 and len(target_parts) == 4:
            # Check for Link-Local (169.254.x.x) which uses /16 mask
            if target_parts[0] == '169' and target_parts[1] == '254':
                 if local_parts[0] != '169' or local_parts[1] != '254':
                    print(f"WARNING: ESP32 is using Link-Local IP ({target_ip}).")
                    print(f"Your PC ({local_ip}) seems to be on a different network (e.g. WiFi).")
                    print("Ensure the Ethernet cable is connected and PC has no Static IP (so it falls back to 169.254.x.x).")
                    print("You might need to disable WiFi temporarily to force traffic via Ethernet.")
                    print("-" * 40)
            elif local_parts[0:3] != target_parts[0:3]:
                print(f"WARNING: Your PC ({local_ip}) and ESP32 ({target_ip}) are on DIFFERENT subnets!")
                print("Communication will likely FAIL.")
                print(f"Please configure your PC Ethernet adapter to a Static IP like {target_parts[0]}.{target_parts[1]}.{target_parts[2]}.10")
                print("-" * 40)

    target_port = 3333
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0) # 2 second timeout
    
    message = "Hello from Windows!"
    print(f"Sending to {target_ip}:{target_port} -> {message}")
    
    try:
        start_time = time.time()
        sock.sendto(message.encode(), (target_ip, target_port))
        
        # Wait for echo
        data, addr = sock.recvfrom(1024)
        end_time = time.time()
        
        print(f"Received from {addr}: {data.decode()}")
        print(f"Round trip time: {(end_time - start_time)*1000:.2f} ms")
        print("SUCCESS: Communication Established!")
        
    except socket.timeout:
        print("Error: No response received.")
        print("Troubleshooting:")
        print("1. Check physical connection (Link LED on ENC28J60).")
        print("2. Disable Windows Firewall for this network or allow UDP port 3333.")
        print("3. Ensure PC and ESP32 are in the same IP range (169.254.x.x).")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
