# SEDAP Server - forwarding the blueye position to SEDAP
This project connects to the Blueye drone with the Blueye SDK and forwards the position, heading, and depth to third-party software using the SEDAP-Express protocol over TCP or UDP.

## Setup

Install uv if you haven't already:
```
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Install dependencies:
```
uv sync
```

## Usage

### TCP Server

Run the TCP server (`-g` is optional for RTSP stream):
```
uv run python blueye_tcp_server.py -g rtsp://192.168.1.101:8554/test
```

Run the TCP client with `-r` to print received messages:
```
uv run python blueye_tcp_client.py -r
```

Issue a command with a lat/long coordinate (in decimal degrees):
```
uv run python blueye_tcp_client.py -c 63.12345 10.12345
```

The server will then print the command as follows:
```
[TCP] Client connected from ('127.0.0.1', 59084)
=========================================================
[COMMAND] MOVE received → POI lat=63.12345, lon=10.12345
=========================================================
```

### UDP Sender

The UDP sender supports two modes: **unicast** to a specific IP address, or **broadcast** on a given port.

Send unicast to a specific address and port:
```
uv run python blueye_udp.py -a 192.168.1.100 -p 5555
```

Send broadcast on a port:
```
uv run python blueye_udp.py -b -p 5555
```

With RTSP stream (`-g` is optional):
```
uv run python blueye_udp.py -b -p 5555 -g rtsp://192.168.1.101:8554/test
```

| Argument               | Description                            |
| ---------------------- | -------------------------------------- |
| `-a`, `--address`      | Destination IP address (unicast mode)  |
| `-b`, `--broadcast`    | Send as broadcast on `255.255.255.255` |
| `-p`, `--port`         | Destination UDP port (required)        |
| `-g`, `--generic-rtsp` | RTSP stream URL for GENERIC messages   |

> **Note:** `-a` and `-b` are mutually exclusive — you must choose one.

### Output Messages

Both the TCP server and UDP sender produce the same SEDAP-Express messages:
```
OWNUNIT;blueye-rov;63.439764;10.426196;-3.2;0.6;263.6;273.0;;;BLUEYE_ROV; # at 5 Hz
HEARTBEAT;blueye-server; # at 1 Hz
GENERIC;43;19BDFE151A0;blueye-server;U;false;mac;ASCII;NONE;RTSP;rtsp://192.168.1.101:8554/test;0.0;0.0;65.0;45.0;10.0;1.0 # at 1 Hz (only when -g is provided)
```

For more documentation on the protocol go to sec III 1. in [SEDAP-Express ICD v1.2.pdf](https://github.com/UNIITY-Team/SEDAP-Express/blob/master/Documentation/SEDAP-Express%20ICD%20v1.2.pdf)
