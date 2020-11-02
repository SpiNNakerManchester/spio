SpiNNaker high-speed serial interface to DVS 
============================================

This repository contains an FPGA design to interace an event-based sensor,
such as an event camera or DVS, to a SpiNNaker system through a High-Speed
Serial Link (HSSL).

The design targets the Xilinx ZYNQ-based ZCU102 development board. It
connects to the SpiNNaker system using a SATA cable. The design implements
the spiNNlink protocol used in SpiNNaker systems for board-to-board interconnect.

The design is built on top of [spI/O](https://github.com/SpiNNakerManchester/spio),
the library of FPGA designs and reusable modules for I/O and
internal connectivity in SpiNNaker systems.

spiNNlink
---------

spiNNlink is the high-speed, serial (hss) board-to-board SpiNNaker interconnect.
It is implemented on three Spartan-6 FPGAs present on the SpiNN-5 board and is
designed to provide transparent board-to-board connectivity. The details of the
spiNNlink protocol are described in the [spiNNlink Frame Transport
Specification](http://spinnakermanchester.github.io/docs/spiNNlink_frame_transport.pdf).

Verilog code for the spiNNlink modules is located in
[spio/modules/hss_multiplexer](https://github.com/SpiNNakerManchester/spio/tree/master/modules/hss_multiplexer).
See
[spio/designs/spinnaker_fpgas](https://github.com/SpiNNakerManchester/spio/tree/master/designs/spinnaker_fpgas)
for an example of how to use spiNNlink.

The following publication describes the SpiNNlink specification and implementation:

LA Plana, J Garside, J Heathcote, J Pepper, S Temple, S Davidson, M Luján and S Furber,
*spiNNlink: FPGA-Based Interconnect for the Million-Core SpiNNaker System*, in IEEE Access,
vol. 8, pp. 84918-84928, 2020, doi:
[10.1109/ACCESS.2020.2991038](https://doi.org/10.1109/ACCESS.2020.2991038).

Authors
-------

The design in this repository is largely the work of:

* Luis A. Plana (The University of Manchester)

Acknowledgments
---------------

Ongoing development of the SpiNNaker Project is supported by
the EU ICT Flagship Human Brain Project under Grant H2020-945539.
LA Plana is supported by the RAIN Hub, which is
funded by the Industrial Strategy Challenge Fund, part of the government’s
modern Industrial Strategy. The fund is delivered by UK Research and
Innovation and managed by EPSRC under grant EP/R026084/1.

We gratefully acknowledge these institutions for their support.
