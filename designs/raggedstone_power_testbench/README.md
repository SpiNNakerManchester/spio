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

The 7-segment display shows (in hex) the number of 150 MHz cycles it took for the
`rxvalid` signal to arrive from the GTP tile since the power-down mode was last
changed. The counter saturates at 0xFFFF.

The top button cycles through different combinations of PLL/RX&TX power-down
settings as displayed on the LEDs. These alternate between power-down and
powered-up states.

The bottom button clears the PRBS checker flag and also clears the 7-segment
display to allow better power measurements to be taken.


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
