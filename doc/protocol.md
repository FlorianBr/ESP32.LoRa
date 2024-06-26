# LoRa protocol

## Frame

### Frame Header

| Version | Frame Type | Device Type | Command   | System ID | Payload length | CRC    |
|---------|------------|-------------|-----------|-----------|----------------|--------|
| 1 Byte  | 1 Byte     | 1 Byte      | 1 Byte    | 8 Byte    | 2 Byte         | 1 Byte |

Frame Types:

- 0 ... None
- 10 ... Request
- 20 ... Response
- 30 ... Broadcast

Device Type:

- 0 ... None
- 10 ... Gateway
- 20 ... End Device

Commands:

- 0 ... None
- 1 ... Lifesign
- 10 ... Data Read
- 20 ... Data Write

System ID:
0 for broadcasts, otherwise destination ID

### ID Response/Broadcast

| Header  | Device Type | Version Major | Version Minor | Uptime    | CRC    |
|---------|-------------|---------------|---------------|-----------|--------|
| 7 Byte  | 2 Byte      | 1 Byte        | 1 Byte        | 4 Byte    | 1 Byte |

### Data Access

| Header  | Endpoint Nr. | Command |  Data      |
|---------|--------------|---------|------------|
| 7 Byte  | 1 Byte       | 1 Byte  | 0..x Byte  |
