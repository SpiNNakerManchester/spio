spI/O
=====

A library of FPGA designs and reusable modules for I/O and internal
connectivity in SpiNNaker systems.

The `modules` directory contains a selection of reusable Verilog modules which
all share a [common interface](https://github.com/SpiNNakerManchester/spio/blob/master/modules/README.md).
See the `README.md` included with each module for general information or see the module itself for a concrete
interface description.

The `designs` directory contains ready-to-synthesise FPGA designs based on the
modules in the library. For example,
[designs/spinnaker_fpgas](https://github.com/SpiNNakerManchester/spio/tree/master/designs/spinnaker_fpgas)
contains the complete code for __spiNNlink__, the SpiNNaker SpiNN-5 board FPGA design.
See [designs/README.md](https://github.com/SpiNNakerManchester/spio/tree/master/designs)
for general advice on how to build these designs.

Test benches are included with some modules and designs and are denoted by the suffix `_tb`.


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

The following publication describes the SpiNNlink specification and implementation:

LA Plana, J Garside, J Heathcote, J Pepper, S Temple, S Davidson, M Luján and S Furber, *spiNNlink: FPGA-Based Interconnect for the Million-Core SpiNNaker System*, in IEEE Access, vol. 8, pp. 84918-84928, 2020, doi: [10.1109/ACCESS.2020.2991038](https://doi.org/10.1109/ACCESS.2020.2991038).


[Open Issues](https://github.com/SpiNNakerManchester/spio/issues)
-----------

The __spI/O__ library was developed as part of several research projects and PhD programmes
which are now finished. Currently there is no active development/maintenance work.

This repository maintains a list of [issues](https://github.com/SpiNNakerManchester/spio/issues)
which describe the state of the library.
Open issues include known bugs, potential enhancements and open questions.


Authors
-------

The designs in this repository are largely the work of:

* Luis A. Plana
* Jonathan Heathcote
* Jeff Pepper

With the collaboration of:

* Simon Davidson
* Jim Garside
* Steve Temple
* Steve Furber

All affiliated, at the time of their contribution, with The University of Manchester.


License
-------

See the LICENSE file for license rights and limitations (BSD 3-Clause).


Acknowledgments
---------------

Work on this repository started as part of the project
'A scalable chip multiprocessor for large-scale neural simulation'.
The project was supported by EPSRC (the UK Engineering and Physical Sciences
Research Council) under grant EP/D07908X/1. Further development was supported
by the European Research Council under the European Union's Seventh Framework
Programme (FP7/2007-2013) / ERC grant agreement 320689.

Aspects of the spiNNlink design, in particular, benefited from a collaboration
with colleagues from Instituto de Microelectrónica de Sevilla (IMSE-CNM), Spain.

We gratefully acknowledge these institutions and individuals for their support.
