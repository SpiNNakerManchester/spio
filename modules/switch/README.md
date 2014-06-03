1-to-N Switch
=============

A module which forwards values from a single rdy/vld input to a set of of N
rdy/vld outputs (i.e. unicast or multicast) determined by an additional incoming
signal. This module is intended to be used as part of a
router, forming part of a crossbar switch. A sketch of the high-level function
of the block is given below:

	                      ,------------,
	                      |            |
	                      |            |--/-> OUT_DATA0
	                      |            |----> OUT_VLD0
	                      |            |<---- OUT_RDY0
	                      |            |
	         IN_DATA --/->|            |--/-> OUT_DATA1
	IN_OUTPUT_SELECT --/->|   1-to-n   |----> OUT_VLD1
	          IN_VLD ---->|   switch   |<---- OUT_RDY1
	          IN_RDY <----|            |
	                      |            |      ....
	                      |            |
	                      |            |--/-> OUT_DATAN
	                      |            |----> OUT_VLDN
	                      |            |<---- OUT_RDYN
	                      '------------'

Blocked Outputs & Packet Dropping
---------------------------------

The module produces signals to indicate when some outputs are blocking the
progress of a packet (and on what ports) to allow an external timeout mechanism
to be implemented. This mechanism can generate a one-clock pulse which will
force the module to drop the current packet.

Note that since the rdy/vld protocol does not allow `vld' to be deasserted, an
output is only considered blocked when it is already valid but a new value
wishes to use that output. It is this new value which will be dropped by the
one-clock drop pulse. There is no way to drop a value which has already been
marked valid.


Author
------

Jonathan Heathcote.
