Switch
======

A module which switches a single rdy/vld input between a set of of N rdy/vld
outputs. The input may be multicast to a number of outputs or dropped entirely.
This module is intended to be used either independently as a minimal router or
integrated into a more fully featured router.

An additional signal is provided along with the input data which indicates which
(if any) of the N outputs the value should be forwarded to.

The module produces signals to indicate when it is blocked (and on what ports)
to allow an external timeout mechanism to be implemented. This mechanism can
generate a one-clock pulse which will force the module to drop the current
packet.

Blocked Outputs
---------------

Note that since the rdy/vld protocol does not allow `vld' to be deasserted, an
output is only considered blocked when it is already valid but a new value
wishes to use that output. It is this new value which will be dropped by the
one-clock drop pulse. There is no way to drop a value which has already been
marked valid.


Author
------

Jonathan Heathcote.
