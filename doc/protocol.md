# LoRa protocol

## Frame

### Frame Header

| Version | Frame Type | Device Type | Command   | Payload length | CRC    |
|---------|------------|-------------|-----------|----------------|--------|
| 1 Byte  | 1 Byte     | 1 Byte      | 1 Byte    | 2 Byte         | 1 Byte |

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

### ID Response/Broadcast

| Header  | System ID | Device Type | Version Major | Version Minor | Uptime    | CRC    |
|---------|-----------|-------------|---------------|---------------|-----------|--------|
| 7 Byte  | 8 Byte    | 2 Byte      | 1 Byte        | 1 Byte        | 4 Byte    | 1 Byte |
