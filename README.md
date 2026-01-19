# SEDAP Server - send blueye position to SEDAP
This is a small TCP server that connects to the Blueye drone with the Blueye SDK and serves the position, heading and depth to third-party software called SEDAP.

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

Run the TCP server:
```
uv run python blueye_tcp_server.py
```

Run the TCP client: with -r for to print receiving messages.
```
uv run python blueye_tcp_client.py -r
```

Issue a command with a lat/long coordinate (in decimal degrees):
```
uv run python blueye_tcp_client.py -c 63.12345 10.12345
```

The server will produce the following messages:
```
OWNUNIT;blueye-rov;63.438731;10.426696;-0.4;0.1;0.0;0.0;;;BLUEYE_ROV; # at 5 Hz
HEARTBEAT from blueye-server; # at 1 Hz
```

For more documentation on the protocol go to sec III 1. in [SEDAP-Express ICD v1.2.pdf](https://github.com/UNIITY-Team/SEDAP-Express/blob/master/Documentation/SEDAP-Express%20ICD%20v1.2.pdf)