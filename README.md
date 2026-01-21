# SEDAP Server - forwarding the blueye position to SEDAP
This is a small TCP server that connects to the Blueye drone with the Blueye SDK and serves the position, heading, and depth to third-party software called SEDAP.

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

Run the TCP server: (-g is optional for RTSP stream)
```
uv run python blueye_tcp_server.py -g rtsp://192.168.1.101:8554/test
```

Run the TCP client: with -r for to print receiving messages.
```
uv run python blueye_tcp_client.py -r
```

Issue a command with a lat/long coordinate (in decimal degrees):
```
uv run python blueye_tcp_client.py -c 63.12345 10.12345
```

The Server will then print the command as follows:
```
[TCP] Client connected from ('127.0.0.1', 59084)
=========================================================
[COMMAND] MOVE received → POI lat=63.12345, lon=10.12345
=========================================================
```

The server will produce the following messages:
```
OWNUNIT;blueye-rov;63.438731;10.426696;-0.4;0.1;0.0;0.0;;;BLUEYE_ROV; # at 5 Hz
HEARTBEAT;blueye-server; # at 1 Hz
GENERIC;43;19BDFE151A0;blueye-server;U;false;mac;ASCII;NONE;RTSP;rtsp://192.168.1.101:8554/test;0.0;0.0;65.0;45.0;10.0;1.0 # at 1 Hz
```

For more documentation on the protocol go to sec III 1. in [SEDAP-Express ICD v1.2.pdf](https://github.com/UNIITY-Team/SEDAP-Express/blob/master/Documentation/SEDAP-Express%20ICD%20v1.2.pdf)
