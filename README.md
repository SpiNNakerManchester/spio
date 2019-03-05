spI/O
=====

A library of FPGA designs and reusable modules for I/O (and internal
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
datasheet](http://spinnakermanchester.github.io/docs/SpiNN2DataShtV202.pdf).
As a result, all attempts to interface with SpiNNaker must be able to sensibly
express and expose themselves in this form. Higher-level protocols built on top
of SpiNNaker packets are also available such as the [SpiNNaker Datagram Protocol
(SDP)](http://spinnakermanchester.github.io/docs/spinn-app-4.pdf) if required,
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


spiNNlink
---------

spiNNlink is the high-speed, serial (hss) board-to-board SpiNNaker interconnect.
It is implemented on three Spartan-6 FPGAs present on the SpiNN-5 board and is
designed to provide transparent board-to-board connectivity. The details of the
spiNNlink protocol are described in the [spiNNlink Frame Transport 
Specification](http://spinnakermanchester.github.io/docs/spiNNlink_frame_transport.pdf).


Verilog code for the spiNNlink modules is located in
[modules/hss_multiplexer](https://github.com/SpiNNakerManchester/spio/tree/master/modules/hss_multiplexer).
See 
[designs/spinnaker_fpgas](https://github.com/SpiNNakerManchester/spio/tree/master/designs/spinnaker_fpgas)
for an example of how to use spiNNlink.


Authors
-------

The designs in this repository are largely the work of:

* Luis Plana
* Jonathan Heathcote
* Jeff Pepper

With the collaboration of:

* Simon Davidson
* Jim Garside
* Steve Temple
* Steve Furber

All affiliated with The University of Manchester.


Copyright
---------

Copyright (C) 2012-2016 The University of Manchester.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
