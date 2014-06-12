Universal Asynchronous Receiver/Transmitter Packet Transceiver
==============================================================

This module provides a simple RS232-style UART implementation with a simple
protocol for transmitting and receiving SpiNNaker packets with a PC or other
device.

This module may be configured either as a master or a slave. Masters are
expected to initiate a connection by sending a simple synchronisation sequence.
The slave is expected to respond to this with its own synchronisation sequence.
From here on, the connection becomes completely symmetric (i.e. two independent
streams of SpiNNaker packets).

Warning: This block will drop SpiNNaker packets with invalid parity without
sending them over the serial link.


Protocol
--------

The protocol is more-or-less a standard UART serial link with (optional)
CTS-based flow control. It also includes a simple synchronisation mechanism
which requires one party to be identified as a 'master' and one as a 'slave'.
The master initiates the flow-control procedure but otherwise has no special
role.

### UART Protocol

Selectable baud-rate, 8-bit bytes, one start bit, one stop bit.

### Flow control

The receiver produces a CTS signal which goes low when the UART receive buffer
reaches a high-water mark.

The transmitter also accepts a CTS signal which a remote device may use to
prevent further bytes being transmitted until CTS is raised.

No other signals (e.g. RTS, XON/XOFF) are produced.

### Error correction/detection (or lack there-of)

It is assumed that the link is a perfect link and bit errors cannot occur.

SpiNNaker packets arriving at the transmitter with parity errors will be
dropped. Such packets are used as 'out-of-band' messages by the synchronisation
protocol and thus are reserved.

### SpiNNaker packet transmission

SpiNNaker packets are sent in little-endian order (i.e. least-significant byte
(the header byte) first). This byte must be used by the receiver to determine
how many bytes the rest of the packet will occupy (either 4 or 8 more bytes for
40 and 72 bit packets respectively).

See the SpiNNaker data sheet section 8.2 for a description of the header byte
format.

### Synchronisation

To make it possible to know when a header byte is being received, a
synchronisation mechanism should be used when the master first opens a
connection. It is assumed that the slave will be ready immediately but will
ignore all incoming traffic until it receives a synchronisation sequence.

To synchronise, the master must send a sequence of 13 or more null bytes (0x00)
followed by a single all-ones byte (0xFF). The CTS signal may be ignored when
sending synchronisation packets.  The master is synchronised once it receives a
packet with a parity error followed by an all-ones byte (both of which must be
discarded).

The slave must monitor the incoming stream of packets for packets with incorrect
parity. Such packets must be discarded. The slave must then stop transmitting
(after completing transmission of its current packet) and ignore all further
received bytes until an all-ones byte arrives (which it must also discard). At
this point, the slave must transmit a sequence of at least 13 null bytes
followed by a single all-ones byte. The slave is now synchronised and may resume
transmission starting with the head of a SpiNNaker packet.

#### Resynchronising

Once a link is already synchronised it should not need resynchronising, however
if the master does reinitiate synchronisation, the protocol is designed such
that no packets should be lost since packet transmission is never aborted.

#### Protocol Rationale

When a sender receives this sequence, it may (in the worst case) have previously
received a header byte for a SpiNNaker packet with a payload and thus will
expect 8 further bytes for which zeros are transmitted.

The next five zero-bytes define a multicast packet without a payload which has a
parity error (since packets must have even parity). This special value can only
occur in organic data when a packet has a parity error and so it is considered
safe to discard such packets.

When a receiver gets an all-zeros packet with a parity error it then ignores all
incoming bytes until it receives a single all-ones (0xFF) byte. This byte is
discarded and the system is considered synchronised.

#### Robustness

Though the protocol ensures that synchronisation is eventually achieved, the
protocol is not robust in the presence of noise in the receive buffer prior to
synchronisation. Without the use of a handshake-based synchronisation protocol,
it is not possible to reliably determine when data received is synchronised.

Since in most cases it is assumed that receive buffers are empty on startup,
this issue should not affect most users.

Authors
-------

Jonathan Heathcote.

