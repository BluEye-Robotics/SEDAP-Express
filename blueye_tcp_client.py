import argparse
import socket
import threading
import time


# =========================
# Message builders
# =========================

def build_heartbeat(sender_id: str) -> str:
    return f"HEARTBEAT;{sender_id}\n"


def build_command_move(lat: float, lon: float) -> str:
    """
    Build a COMMAND message with case 5 (MOVE) for lat/lon POI.
    """
    return f"COMMAND;move_cmd;5;{lat:.6f};{lon:.6f}\n"


# =========================
# Message handlers
# =========================

def handle_ownunit(message: str, verbose: bool = False):
    """
    Parse and print OWNUNIT messages.
    Format: OWNUNIT;unit_id;lat;lon;alt;speed;course;heading;roll;pitch;name;sidc
    """
    parts = message.strip().split(";")

    if len(parts) < 8:
        return

    if parts[0] != "OWNUNIT":
        return

    try:
        unit_id = parts[1]
        lat = float(parts[2])
        lon = float(parts[3])
        alt = float(parts[4]) if parts[4] else 0.0
        speed = float(parts[5]) if parts[5] else 0.0
        course = float(parts[6]) if parts[6] else 0.0
        heading = float(parts[7]) if parts[7] else 0.0
        roll = float(parts[8]) if len(parts) > 8 and parts[8] else None
        pitch = float(parts[9]) if len(parts) > 9 and parts[9] else None
        name = parts[10] if len(parts) > 10 and parts[10] else ""
        sidc = parts[11] if len(parts) > 11 and parts[11] else ""

        print(
            f"[OWNUNIT] {unit_id}: "
            f"lat={lat:.6f}, lon={lon:.6f}, "
            f"alt={alt:.1f}m, spd={speed:.1f} m/s, "
            f"course={course:.1f}°, hdg={heading:.1f}°"
        )
        if roll is not None or pitch is not None:
            print(f"  roll={roll:.1f if roll is not None else 'N/A'}°, pitch={pitch:.1f if pitch is not None else 'N/A'}°")
        if name:
            print(f"  name={name}")
        # print the raw message for reference only if verbose
        if verbose:
            print(f"  Raw message: {message.strip()}")

    except (IndexError, ValueError) as e:
        print(f"[OWNUNIT] Parse error: {e}")


def handle_heartbeat(message: str):
    """
    Parse HEARTBEAT messages (optional, for debugging).
    Format: HEARTBEAT;sender_id
    """
    parts = message.strip().split(";")
    if len(parts) >= 2 and parts[0] == "HEARTBEAT":
        sender_id = parts[1]
        print(f"[HEARTBEAT] from {sender_id}")


def handle_generic(message: str, verbose: bool = False):
    """
    Parse and print GENERIC messages.
    Format: GENERIC;<Number>;<Time>;<Sender>;<Classification>;<Acknowledgement>;<MAC>;<ContentType>;<Encoding>;<Content>;<Rtsp stream>;<Azimuth_deg>;<Elevation_deg>;<HorizontalOpeningAngle_deg>;<VerticalOpeningAngle_deg>;<Range_m>;<Zoom>
    """
    parts = message.strip().split(";")

    if len(parts) < 11:
        return

    if parts[0] != "GENERIC":
        return

    try:
        number = parts[1]
        timestamp = parts[2]
        sender = parts[3]
        classification = parts[4]
        acknowledgement = parts[5]
        mac = parts[6]
        content_type = parts[7]
        encoding = parts[8]
        content = parts[9]
        rtsp_stream = parts[10]
        azimuth = float(parts[11]) if len(parts) > 11 and parts[11] else 0.0
        elevation = float(parts[12]) if len(parts) > 12 and parts[12] else 0.0
        h_fov = float(parts[13]) if len(parts) > 13 and parts[13] else 0.0
        v_fov = float(parts[14]) if len(parts) > 14 and parts[14] else 0.0
        range_m = float(parts[15]) if len(parts) > 15 and parts[15] else 0.0
        zoom = float(parts[16]) if len(parts) > 16 and parts[16] else 1.0

        print(
            f"[GENERIC] #{number} from {sender} at {timestamp}\n"
            f"  Content: {content}\n"
            f"  RTSP: {rtsp_stream}\n"
            f"  Camera: azimuth={azimuth:.1f}°, elevation={elevation:.1f}°, "
            f"FOV={h_fov:.1f}°x{v_fov:.1f}°, range={range_m:.0f}m, zoom={zoom:.1f}"
        )
        if verbose:
            print(f"  Raw message: {message.strip()}")

    except (IndexError, ValueError) as e:
        print(f"[GENERIC] Parse error: {e}")


# =========================
# Receive thread
# =========================

def receive_messages(sock: socket.socket, verbose: bool = False):
    """
    Continuously receive and parse messages from the server.
    """
    sock.settimeout(1.0)
    buffer = ""

    try:
        while True:
            try:
                data = sock.recv(4096)
                if not data:
                    print("[TCP] Connection closed by server")
                    break

                buffer += data.decode()

                # Process complete messages (split by newline)
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()

                    if not line:
                        continue

                    if line.startswith("OWNUNIT"):
                        handle_ownunit(line, verbose)
                    elif line.startswith("HEARTBEAT"):
                        handle_heartbeat(line)
                    elif line.startswith("GENERIC"):
                        handle_generic(line, verbose)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[TCP] Error receiving: {e}")
                break

    finally:
        print("[TCP] Receive thread stopped")


# =========================
# Main client
# =========================

def run_client(host="127.0.0.1", port=5555, mode="receive", command_lat=None, command_lon=None, verbose=False):
    """
    Connect to the SEDAP-Express server and handle communication.

    Args:
        host: Server hostname or IP
        port: Server port
        mode: "receive" for continuous monitoring, "command" for sending a COMMAND
        command_lat: Latitude for COMMAND message (required if mode="command")
        command_lon: Longitude for COMMAND message (required if mode="command")
        verbose: Enable verbose output (show raw messages)
    """
    print(f"[TCP] Connecting to {host}:{port}...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        sock.connect((host, port))
        print(f"[TCP] Connected to {host}:{port}")

        # Send periodic heartbeats
        sender_id = "python-client"
        last_hb = 0

        if mode == "command":
            # Command mode: send COMMAND and exit
            if command_lat is None or command_lon is None:
                print("[ERROR] Latitude and longitude required for command mode")
                return

            print(f"\n--- Sending COMMAND (MOVE) to lat={command_lat}, lon={command_lon} ---\n")
            command_msg = build_command_move(command_lat, command_lon)
            sock.sendall(command_msg.encode())

            # Send a heartbeat and wait a moment for any response
            sock.sendall(build_heartbeat(sender_id).encode())
            time.sleep(1)
            print("[TCP] Command sent successfully")

        else:
            # Start receive thread
            recv_thread = threading.Thread(
                target=receive_messages,
                args=(sock, verbose),
                daemon=True
            )
            recv_thread.start()
            # Receive mode: continuous monitoring
            print("\n--- Receiving OWNUNIT messages (press Ctrl+C to exit) ---\n")

            # Keep connection alive with heartbeats
            while True:
                now = time.time()

                if now - last_hb >= 1.0:
                    sock.sendall(build_heartbeat(sender_id).encode())
                    last_hb = now

                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[TCP] Interrupted by user")
    except Exception as e:
        print(f"[TCP] Error: {e}")
    finally:
        sock.close()
        print("[TCP] Connection closed")


# =========================
# Entry point
# =========================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="TCP client for SEDAP-Express protocol (OWNUNIT/COMMAND messages)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Receive mode (continuous monitoring):
  python blueye_tcp_client.py -r

  # Command mode (send lat/lon command and exit):
  python blueye_tcp_client.py -c 63.12345 10.123456

  # Connect to remote server:
  python blueye_tcp_client.py -r --host 192.168.1.100 --port 5555
        """
    )

    # Mode selection (mutually exclusive)
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument(
        "-r", "--receive",
        action="store_true",
        help="Receive mode: continuously monitor OWNUNIT messages"
    )
    mode_group.add_argument(
        "-c", "--command",
        nargs=2,
        metavar=("LAT", "LON"),
        type=float,
        help="Command mode: send COMMAND message with lat/lon coordinates and exit"
    )

    # Connection parameters
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Server hostname or IP address (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5555,
        help="Server port (default: 5555)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output (show raw messages)"
    )

    args = parser.parse_args()

    # Determine mode and parameters
    if args.receive:
        run_client(host=args.host, port=args.port, mode="receive", verbose=args.verbose)
    elif args.command:
        lat, lon = args.command
        run_client(
            host=args.host,
            port=args.port,
            mode="command",
            command_lat=lat,
            command_lon=lon,
            verbose=args.verbose
        )
