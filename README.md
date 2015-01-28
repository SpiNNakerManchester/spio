spI/O
=====

A library of FPGA designs and re-usable modules for I/O (and internal
connectivity) in SpiNNaker systems.

The `designs` directory contains ready-to-synthesise FPGA designs based on the
modules in the library, for example, the SpiNNaker SpiNN-5 board FPGA design.
See `designs/README.md` for general advice on how to build these designs.

The `modules` directory contains a selection of reusable Verilog modules which
all share the common interface described below. See the `README.md` included
with each module for general information or see the module itself for a concrete
interface description.

Test benches are included with some designs and are denoted by the suffix `_tb`.


Common Interface
----------------

SpiNNaker speaks 'SpiNNaker Packets' which are described in the [SpiNNaker
datasheet](https://solem.cs.man.ac.uk/documentation/datasheet/SpiNN2DataShtV202.pdf).
As a result, all attempts to interface with SpiNNaker must be able to sensibly
express and expose themselves in this form. Higher-level protocols built on top
of SpiNNaker packets are also available such as the [SpiNNaker Datagram Protocol
(SDP)](https://solem.cs.man.ac.uk/documentation/spinn-app-4.pdf) if required,
but ultimately everything ends up just being SpiNNaker packets. As a result, all
modules in this library share the following common interface designed to
communicate SpiNNaker packets.

To send packets from A to B, the following interface is used:

	    ,--------,             ,--------,
	    |        | data[71:0]  |        |
	    |        |======/=====>|        |
	    |        |             |        |
	    |    A   |     vld     |    B   |
	    |        |------------>|        |
	    |        |             |        |
	    |        |     rdy     |        |
	    |   /\   |<------------|   /\   |
	    '--'--`--'             '--'--`--'
	        |                       |
	clk ----+-----------------------'

The `data` bus is 72-bits wide, the width of a SpiNNaker packet with a payload.
Devices which never produce packets with payloads may tie the upper 32 bits to
any value (but not be left floating). Devices that cannot consume packets with
payloads may safely ignore these upper 32 bits.

If the `vld` signal is asserted then the data that A wishes to transfer is on the
`data` bus. The `rdy` signal is asserted whenever B is capable of accepting new
data. If `rdy` and `vld` are asserted in the same cycle, the value on the data
bus is captured by B. In the next cycle, if both signals are still asserted then
the new data on the bus is considered transferred.

Once the `vld` signal has been asserted it must not be deasserted until `rdy`
has gone high (i.e. until the value is transferred). Similarly, `rdy` must also
not be deasserted once asserted until `vld` goes high (i.e. a value is
transferred).


Authors
-------

The designs in this repository are largely the work of:

* Luis Plana
* Jeff Pepper
* Jonathan Heathcote


Copyright
---------

For the moment, all the designs are copyright of the University of Manchester,
all rights reserved. This will hopefuly change in the near future.
