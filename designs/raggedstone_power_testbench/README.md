Raggedstone 2 Experimental Power Management Testbench
=====================================================

A testbench for experimenting with power management features of the GTP tiles on
a Spartan 6 FPGA. This testbench is for experimental work and not intended to be
useful and should not make it into the master branch.

Usage
-----

The four LEDs on the raggedstone board are configured as follows from
left-to-right:

* Blinking LED (is the clock up?)
* PLL Powerdown Signal
* RX & TX Powerdown Signals
* Sticky PRBS Checker Error Flag

The top button cycles through different combinations of PLL/RX&TX power-down
settings as displayed on the LEDs.

The bottom button clears the PRBS checker flag.


Authors
-------

This design was produced by Jonathan Heathcote.


Dependencies
------------

None at present.


Target Platform
---------------

This design is for a Xilinx Spartan-6 XC6SLX45T FPGA in a FG(G)484 package with a speed grade of
-2, as fitted in the Raggedstone 2 board.
