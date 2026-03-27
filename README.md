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

Both the TCP server and UDP sender produce the same SEDAP-Express v1.4 messages.

All messages share a common header:
```
<MessageType>;<Number>;<Time>;<Sender>;<Classification>;<Acknowledgement>;<MAC>;...
```

| Header Field       | Description                                      | Example            |
| ------------------ | ------------------------------------------------ | ------------------ |
| Number             | Message sequence number (hex byte, 00-FF)        | `01`               |
| Time               | Timestamp in ms since epoch (12-digit hex)       | `0191C643A8AF`     |
| Sender             | ID of the sending system                         | `blueye-server`    |
| Classification     | Security classification (U/R/C/S/TS)             | `U`                |
| Acknowledgement    | Whether acknowledgement is requested             | `FALSE`            |
| MAC                | Message Authentication Code (optional)           |                    |

#### OWNUNIT (5 Hz)
```
OWNUNIT;<Number>;<Time>;<Sender>;<Classification>;<Acknowledgement>;<MAC>;<Latitude>;<Longitude>;<Altitude>;<Speed>;<Course>;<Heading>;<Roll>;<Pitch>;<Name>;<SIDC>
```
```
OWNUNIT;01;0191C643A8AF;blueye-server;U;FALSE;;63.439764;10.426196;-3.2;0.6;263.6;273.0;;;BLUEYE_ROV;
```

| Field     | Description                            | Example        |
| --------- | -------------------------------------- | -------------- |
| Latitude  | WGS84 latitude in decimal degrees      | `63.439764`    |
| Longitude | WGS84 longitude in decimal degrees     | `10.426196`    |
| Altitude  | Altitude/depth in meters               | `-3.2`         |
| Speed     | Speed in m/s                           | `0.6`          |
| Course    | Course over ground in degrees (0-360)  | `263.6`        |
| Heading   | Heading in degrees (0-360)             | `273.0`        |
| Roll      | Roll angle in degrees (optional)       |                |
| Pitch     | Pitch angle in degrees (optional)      |                |
| Name      | Display name of the unit               | `BLUEYE_ROV`   |
| SIDC      | Symbol Identification Code (optional)  |                |

#### HEARTBEAT (1 Hz)
```
HEARTBEAT;<Number>;<Time>;<Sender>;<Classification>;<Acknowledgement>;<MAC>
```
```
HEARTBEAT;01;0191C643A8AF;blueye-server;U;FALSE;
```

#### GENERIC — RTSP stream (1 Hz, only when `-g` is provided)
```
GENERIC;<Number>;<Time>;<Sender>;<Classification>;<Acknowledgement>;<MAC>;<ContentType>;<Encoding>;<Content>;<RtspStream>;<Azimuth>;<Elevation>;<HFov>;<VFov>;<Range>;<Zoom>
```
```
GENERIC;01;0191C643A8AF;blueye-server;U;false;mac;ASCII;NONE;RTSP;rtsp://192.168.1.101:8554/test;0.0;0.0;65.0;45.0;10.0;1.0
```

| Field       | Description                                | Example                              |
| ----------- | ------------------------------------------ | ------------------------------------ |
| ContentType | MIME content type                          | `ASCII`                              |
| Encoding    | Content encoding                           | `NONE`                               |
| Content     | Content descriptor                         | `RTSP`                               |
| RtspStream  | RTSP stream URL                            | `rtsp://192.168.1.101:8554/test`     |
| Azimuth     | Camera azimuth in degrees                  | `0.0`                                |
| Elevation   | Camera elevation in degrees                | `0.0`                                |
| HFov        | Horizontal field of view in degrees        | `65.0`                               |
| VFov        | Vertical field of view in degrees          | `45.0`                               |
| Range       | Sensor range in meters                     | `10.0`                               |
| Zoom        | Zoom factor                                | `1.0`                                |

For more documentation on the protocol go to sec III 1. in [SEDAP-Express ICD v1.4.pdf](https://github.com/UNIITY-Team/SEDAP-Express/blob/master/Documentation/SEDAP-Express%20ICD%20v1.4.pdf)
