/**
 * Diagnostic register types.
 */

`ifndef SPIO_HSS_MUTLIPLEXER_REG_BANK_H
`define SPIO_HSS_MUTLIPLEXER_REG_BANK_H

`define REGA_BITS        6
`define REGD_BITS       32

`define VERS_REG         0   // (RO) Version: top 24 bits: Module btm 8 bits: Protocol
`define CRCE_REG         1   // (RO) CRC error counter
`define FRME_REG         2   // (RO) Frame error counter
`define BUSY_REG         3   // (RO) Packet dispatcher busy counter
`define LNAK_REG         4   // (RO) Local nack'd frame counter
`define RNAK_REG         5   // (RO) Remote nack counter
`define LACK_REG         6   // (RO) Local ack'd frame counter
`define RACK_REG         7   // (RO) Remote ack counter
`define LOOC_REG         8   // (RO) Local out-of-credit counter
`define ROOC_REG         9   // (RO) Remote out-of-credit counter
`define CRDT_REG         10  // (RO) Credit
`define SFRM_REG         11  // (RO) Frame assembler valid sent frame counter
`define TFRM_REG         12  // (RO) Frame transmitter frame counter
`define DFRM_REG         13  // (RO) Frame disassembler valid frame counter
`define RFRM_REG         14  // (RO) Packet dispatcher valid receieved frame counter
`define EMPT_REG         15  // (RO) Empty frame assembler queues
`define FULL_REG         16  // (RO) Full frame assembler queues
`define CFCL_REG         17  // (RO) Local channel flow control status
`define CFCR_REG         18  // (RO) Remote channel flow control status
`define IDSO_REG         19  // (RW) IDle Sentinel Output value (value sent in "idle" frames)
`define IDSI_REG         20  // (RO) IDle Sentinel Input value (latest received sentinel)
`define HAND_REG         21  // (RO) Handshake: bit 0: complete bit 1: version mismatch
`define RECO_REG         22  // (RO) Link reconnection counter
`define STOP_REG         23  // (RW) 1 = Stop sending data frames (NB: Will still receive them)


`endif
