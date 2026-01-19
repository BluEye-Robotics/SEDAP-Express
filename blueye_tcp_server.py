import socket
import threading
import time
import blueye.protocol as bp
from blueye.sdk import Drone
import numpy as np


# =========================
# Global state for position
# =========================

class PositionState:
    """Thread-safe storage for latest position estimate."""
    def __init__(self):
        self.lat = 63.439396904796006,  # Default fallback position
        self.lon = 10.42752058910458
        self.heading = 0.0
        self.course_over_ground = 0.0
        self.speed = 0.0
        self.is_valid = False
        self.depth = 0.0
        self.lock = threading.Lock()

    def update(self, lat: float, lon: float, heading: float = 0.0, course_over_ground: float = 0.0, speed: float = 0.0, is_valid: bool = True):
        with self.lock:
            self.lat = lat
            self.lon = lon
            self.heading = heading
            self.course_over_ground = course_over_ground
            self.speed = speed
            self.is_valid = is_valid

    def update_depth(self, depth: float):
        with self.lock:
            self.depth = depth

    def get(self):
        with self.lock:
            return self.lat, self.lon, self.heading, self.course_over_ground, self.speed, self.is_valid, self.depth


position_state = PositionState()


# =========================
# Message builders
# =========================

def build_heartbeat(sender_id: str) -> str:
    return f"HEARTBEAT;{sender_id}\n"


def build_ownunit(unit_id: str,
                  lat: float,
                  lon: float,
                  alt: float = 0.0,
                  speed: float = 0.0,
                  course: float = 0.0,
                  heading: float = 0.0,
                  roll: float = None,
                  pitch: float = None,
                  name: str = "BLUEYE_ROV",
                  sidc: str = None) -> str:
    """
    Build OWNUNIT message according to SEDAP-Express specification.
    Field order: latitude, longitude, altitude, speed, course, heading, roll, pitch, name, sidc
    """
    roll_str = f"{roll:.1f}" if roll is not None else ""
    pitch_str = f"{pitch:.1f}" if pitch is not None else ""
    name_str = name if name is not None else ""
    sidc_str = sidc if sidc is not None else ""

    return (
        f"OWNUNIT;"
        f"{unit_id};"
        f"{lat:.6f};"
        f"{lon:.6f};"
        f"{alt:.1f};"
        f"{speed:.1f};"
        f"{course:.1f};"
        f"{heading:.1f};"
        f"{roll_str};"
        f"{pitch_str};"
        f"{name_str};"
        f"{sidc_str}\n"
    )


# =========================
# COMMAND handling
# =========================

def handle_command(message: str):
    """
    Parses COMMAND messages.
    Case 5 = MOVE → lat / lon POI
    """
    parts = message.strip().split(";")

    if len(parts) < 3:
        return

    if parts[0] != "COMMAND":
        return

    command_id = parts[1]
    case = parts[2]

    if case == "5":
        try:
            lat = float(parts[3])
            lon = float(parts[4])
            print("======================================================")
            print(f"[COMMAND] MOVE received → POI lat={lat}, lon={lon}")
            print("======================================================")
        except (IndexError, ValueError):
            print("[COMMAND] Invalid MOVE command format")

    else:
        print(f"[COMMAND] Unsupported case {case}")


# =========================
# Drone telemetry callback
# =========================

def callback_position_estimate(msg_type: str, msg: bp.PositionEstimateTel):
    """Callback for the PositionEstimateTel message."""
    position_estimate = msg.position_estimate
    lat = position_estimate.global_position.latitude
    lon = position_estimate.global_position.longitude
    course_over_ground = position_estimate.course_over_ground*180/np.pi % 360
    heading = position_estimate.heading*180/np.pi % 360
    speed = np.sqrt(np.pow(position_estimate.surge_rate, 2) + np.pow(position_estimate.sway_rate, 2))
    is_valid = position_estimate.is_valid

    # Update global position state
    position_state.update(lat, lon, heading, course_over_ground=course_over_ground, speed=speed, is_valid=is_valid)
    print(f"[POSITION] Updated: lat={lat:.6f}, lon={lon:.6f}, course={course_over_ground:.1f} heading={heading:.1f}, speed={speed:.1f} valid={is_valid}")

def callback_depth(msg_type: str, msg: bp.DepthTel):
    """Callback for the Depth message."""
    depth = msg.depth.value
    position_state.update_depth(depth)


# =========================
# Client handler
# =========================

def client_handler(conn: socket.socket, addr):
    print(f"[TCP] Client connected from {addr}")

    unit_id = "blueye-rov"
    sender_id = "blueye-server"

    last_hb = 0
    last_ownunit = 0

    conn.settimeout(0.1)

    try:
        while True:
            now = time.time()

            # ---- HEARTBEAT @ 1 Hz ----
            if now - last_hb >= 1.0:
                conn.sendall(build_heartbeat(sender_id).encode())
                last_hb = now

            # ---- OWNUNIT @ 5 Hz ----
            if now - last_ownunit >= 0.2:
                # Get latest position from drone
                lat, lon, heading, course_over_ground, speed, is_valid, depth = position_state.get()

                conn.sendall(
                    build_ownunit(
                        unit_id=unit_id,
                        lat=lat,
                        lon=lon,
                        alt=depth,
                        speed=speed,
                        course=course_over_ground,
                        heading=heading
                    ).encode()
                )
                last_ownunit = now

            # ---- Receive COMMANDs ----
            try:
                data = conn.recv(1024)
                if data:
                    messages = data.decode().splitlines()
                    for msg in messages:
                        handle_command(msg)
            except socket.timeout:
                pass

    except (ConnectionResetError, BrokenPipeError):
        print("[TCP] Client disconnected")

    finally:
        conn.close()


# =========================
# TCP server
# =========================

def start_server(host="0.0.0.0", port=5555):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(1)

    print(f"[TCP] SEDAP-Express server listening on {host}:{port}")

    while True:
        conn, addr = server.accept()
        threading.Thread(
            target=client_handler,
            args=(conn, addr),
            daemon=True
        ).start()


# =========================
# Entry point
# =========================

if __name__ == "__main__":
    # Connect to the Blueye drone
    print("[DRONE] Connecting to Blueye drone...")
    my_drone = Drone(connect_as_observer=True)

    # Add callback for position updates
    callback_pos_id = my_drone.telemetry.add_msg_callback(
        [bp.PositionEstimateTel],
        callback_position_estimate
    )
    print("[DRONE] Position telemetry callback registered")

    # Add callback for depth updates
    callback_depth_id = my_drone.telemetry.add_msg_callback(
        [bp.DepthTel],
        callback_depth
    )
    print("[DRONE] Depth callback registered")

    # Start TCP server in a separate thread
    server_thread = threading.Thread(
        target=start_server,
        daemon=False
    )
    server_thread.start()

    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
    finally:
        my_drone.disconnect()
        print("[DRONE] Disconnected")