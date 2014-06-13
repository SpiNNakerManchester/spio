Raggedstone 2 Low-Speed FTDI-Based USB SpiNNaker Interface
==========================================================

A minimal design for the Raggedstone 2 development board which provides a
proof-of-concept SpiNNaker-to-host USB interface via the onboard FTDI based
USB "serial" port.

The top button (SW2) resets the design.


Authors
-------

This top-level design was produced by Jonathan Heathcote.


Dependencies
------------

This design makes use of the following modules from the spI/O module collection:
* `hss_multiplexer/*`
* `status_led_generator/*`
* `uart/*`


Target Platform
---------------

This design is for a Xilinx Spartan-6 XC6SLX45T FPGA in a FG(G)484 package with a speed grade of
-2, as fitted in the Raggedstone 2 board.


Chipscope VIO
-------------

A chipscope virtual I/O port can be added to aid in debugging allowing access to
both the HSS multiplexer's debug register banks and also various internal
signals.
