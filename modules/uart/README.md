Universal Asynchronous Receiver/Transmitter Packet Transceiver
==============================================================

This pair of modules provides a simple UART interface for exchanging SpiNNaker
packets.


Protocol
--------

### UART Protocol

Selectable baud-rate, 8-bit bytes, one start bit, one stop bit, optional CTS
flow control.

### SpiNNaker packet transmission

SpiNNaker packets are sent in little-endian order (i.e. least-significant byte
(the header byte) first). This byte must be used by the receiver to determine
how many bytes the rest of the packet will occupy (either 4 or 8 more bytes for
40 and 72 bit packets respectively).

### Synchronisation

TODO

Authors
-------

Jonathan Heathcote.

