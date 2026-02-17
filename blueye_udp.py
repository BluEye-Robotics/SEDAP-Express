import socket
import threading
import time
import argparse
import blueye.protocol as bp
from blueye.sdk import Drone
import numpy as np


# =========================
# Global state for position
# =========================

class PositionState:
    """Thread-safe storage for latest position estimate."""
    def __init__(self):
        self.lat = 63.439396904796006  # Default fallback position
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
            self.depth = -depth

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


def build_generic_rtsp(number: str,
                       timestamp: str,
                       sender: str,
                       rtsp_url: str,
                       azimuth: float = 0.0,
                       elevation: float = 0.0,
                       h_fov: float = 65.0,
                       v_fov: float = 45.0,
                       range_m: float = 10.0,
                       zoom: float = 1.0) -> str:
    """
    Build GENERIC message for RTSP stream according to SEDAP-Express specification.

    Structure:
    GENERIC;<Number>;<Time>;<Sender>;<Classification>;<Acknowledgement>;<MAC>;<ContentType>;<Encoding>;<Content>;<Rtsp stream>;
    <Azimuth_deg>;<Elevation_deg>;<HorizontalOpeningAngle_deg>;<VerticalOpeningAngle_deg>;<Range_m>;<Zoom>
    """
    return (
        f"GENERIC;"
        f"{number};"
        f"{timestamp};"
        f"{sender};"
        f"U;"
        f"false;"
        f"mac;"
        f"ASCII;"
        f"NONE;"
        f"RTSP;"
        f"{rtsp_url};"
        f"{azimuth:.1f};"
        f"{elevation:.1f};"
        f"{h_fov:.1f};"
        f"{v_fov:.1f};"
        f"{range_m};"
        f"{zoom:.1f}\n"
    )


# =========================
# Drone telemetry callbacks
# =========================

def callback_position_estimate(msg_type: str, msg: bp.PositionEstimateTel):
    """Callback for the PositionEstimateTel message."""
    position_estimate = msg.position_estimate
    lat = position_estimate.global_position.latitude
    lon = position_estimate.global_position.longitude
    course_over_ground = position_estimate.course_over_ground * 180 / np.pi % 360
    heading = position_estimate.heading * 180 / np.pi % 360
    speed = np.sqrt(np.pow(position_estimate.surge_rate, 2) + np.pow(position_estimate.sway_rate, 2))
    is_valid = position_estimate.is_valid

    position_state.update(lat, lon, heading, course_over_ground=course_over_ground, speed=speed, is_valid=is_valid)
    print(f"[POSITION] Updated: lat={lat:.6f}, lon={lon:.6f}, course={course_over_ground:.1f} heading={heading:.1f}, speed={speed:.1f} valid={is_valid}")


def callback_depth(msg_type: str, msg: bp.DepthTel):
    """Callback for the Depth message."""
    depth = msg.depth.value
    position_state.update_depth(depth)


# =========================
# Global config for RTSP
# =========================

rtsp_url = None


# =========================
# UDP sender
# =========================

shutdown_flag = threading.Event()


def start_udp_sender(address: str, port: int, broadcast: bool = False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if broadcast:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    dest = (address, port)
    mode = "broadcast" if broadcast else "unicast"
    print(f"[UDP] SEDAP-Express sending {mode} to {address}:{port}")

    unit_id = "blueye-rov"
    sender_id = "blueye-server"

    last_hb = 0
    last_ownunit = 0
    last_generic = 0
    generic_counter = 0

    try:
        while not shutdown_flag.is_set():
            now = time.time()

            # ---- HEARTBEAT @ 1 Hz ----
            if now - last_hb >= 1.0:
                sock.sendto(build_heartbeat(sender_id).encode(), dest)
                last_hb = now

            # ---- OWNUNIT @ 5 Hz ----
            if now - last_ownunit >= 0.2:
                lat, lon, heading, course_over_ground, speed, is_valid, depth = position_state.get()

                sock.sendto(
                    build_ownunit(
                        unit_id=unit_id,
                        lat=lat,
                        lon=lon,
                        alt=depth,
                        speed=speed,
                        course=course_over_ground,
                        heading=heading
                    ).encode(),
                    dest
                )
                last_ownunit = now

            # ---- GENERIC (RTSP) @ 1 Hz ----
            if rtsp_url and now - last_generic >= 1.0:
                generic_counter += 1
                timestamp = hex(int(now * 1000))[2:].upper()
                number = hex(generic_counter)[2:].upper()

                sock.sendto(
                    build_generic_rtsp(
                        number=number,
                        timestamp=timestamp,
                        sender=sender_id,
                        rtsp_url=rtsp_url,
                        azimuth=heading,
                        elevation=0.0
                    ).encode(),
                    dest
                )
                last_generic = now

            # Small sleep to avoid busy-waiting
            time.sleep(0.01)

    except OSError as e:
        print(f"[UDP] Socket error: {e}")

    finally:
        sock.close()
        print("[UDP] Socket closed")


# =========================
# Entry point
# =========================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SEDAP-Express UDP sender for Blueye drone")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "-a", "--address",
        type=str,
        help="Destination IP address for unicast"
    )
    group.add_argument(
        "-b", "--broadcast",
        action="store_true",
        help="Send as broadcast (uses 255.255.255.255)"
    )

    parser.add_argument(
        "-p", "--port",
        type=int,
        required=True,
        help="Destination UDP port"
    )
    parser.add_argument(
        "-g", "--generic-rtsp",
        type=str,
        help="Enable GENERIC message with RTSP stream URL (sent at 1 Hz)"
    )

    args = parser.parse_args()

    # Determine destination address
    if args.broadcast:
        dest_address = "255.255.255.255"
        is_broadcast = True
    else:
        dest_address = args.address
        is_broadcast = False

    # Set global RTSP URL if provided
    if args.generic_rtsp:
        rtsp_url = args.generic_rtsp
        print(f"[CONFIG] GENERIC/RTSP enabled with URL: {rtsp_url}")

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

    # Start UDP sender in a separate thread
    sender_thread = threading.Thread(
        target=start_udp_sender,
        args=(dest_address, args.port, is_broadcast),
        daemon=False
    )
    sender_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
    finally:
        shutdown_flag.set()
        my_drone.disconnect()
        print("[DRONE] Disconnected")
        sender_thread.join(timeout=2.0)
        print("[MAIN] Shutdown complete")
