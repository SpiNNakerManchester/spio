Raggedstone 2 SpiNNaker -- AER Interface
========================================

A minimal design for the Raggedstone 2 development board which provides a
proof-of-concept SpiNNaker-to-AER interface via FPGA general purpose I/O pins.

A note in the `doc` directory describes the design and the user interface.

Authors
-------

This design was produced by Luis Plana, Jeffrey Pepper and Simon Davidson.


Dependencies
------------

This design makes use of the following modules from the spI/O module collection:
* `aer_interface/*`
* `spinnaker_link/*`
* `switch/*`


Target Platform
---------------

This design is for a Xilinx Spartan-6 XC6SLX45T FPGA in a FG(G)484
package with a speed grade of -2, as fitted in the Raggedstone 2
board.
