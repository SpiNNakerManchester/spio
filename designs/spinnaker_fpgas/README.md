The Default SpiNNaker FPGA Design
=================================

This design for the SpiNN-5 board's Spartan-6 FPGAs is designed to provide transparent
board-to-board connectivity.

This design uses two GTP dual tiles to provide four gigabit transceiver links.  These four links
are all clocked from the same external clock source and the entire design is intended to be clocked
from multiples/divisions of this clock.

By convention, the four HSS links are named M0, M1, M2 and M3 corresponding to Tile 0 Block 0, Tile
0 Block 1, Tile 1 Block 0 and Tile 1 block 1 respectively.

M0-3 are broken out to S-ATA connectors on the SpiNN-5 board while M3 is connected in a ring
network which is connected (in the TX direction) as  FPGA 0 -> FPGA 1 -> FPGA 2 -> FPGA 0.

M0 and M1 are always used for board-to-board links while M2 is always used for peripherals.

Authors
-------

This top-level design was produced by Jonathan Heathcote and is derived from the original design by
Luis Plana and, before him, Jeff Pepper.


Dependencies
------------

This design makes use of the following modules from the spI/O module collection:
* `hss_multiplexer/*`
* `spinnaker_link/*`
* `status_led_generator/*`
* `rr_arbiter/*`


Target Platform
---------------

This design is for a Xilinx Spartan-6 XC6SLX45T FPGA in a FG(G)484 package with a speed grade of
-3, as fitted in the SpiNN-5 PCB.


