The Default SpiNNaker FPGA Design
=================================

This design for the SpiNN-5 board's Spartan-6 FPGAs is designed to provide
transparent board-to-board connectivity.

This design uses two GTP dual tiles to provide four gigabit transceiver links.
These four links are all clocked from the same external clock source and the
entire design is intended to be clocked from multiples/divisions of this clock.

By convention, the four HSS links are named M0, M1, M2 and M3 corresponding to
Tile 0 Block 0, Tile 0 Block 1, Tile 1 Block 0 and Tile 1 block 1 respectively.

M0-2 are broken out to S-ATA connectors on the SpiNN-5 board while M3 is
connected in a ring network which is connected (in the TX direction) as  FPGA 0
-> FPGA 1 -> FPGA 2 -> FPGA 0.

M0 and M1 are always used for board-to-board links while M2 is always used for
peripherals.

Authors
-------

This top-level design was produced by Jonathan Heathcote and is derived from the
original design by Luis Plana and, before him, Jeff Pepper.


Dependencies
------------

This design makes use of the following modules from the spI/O module collection:
* `hss_multiplexer/*`
* `spinnaker_link/*`
* `link_speed_interface/*`
* `status_led_generator/*`
* `rr_arbiter/*`
* `switch/*`
* `packet_counter/*`

Synthesis
---------

The design differs in pin configuration between the three FPGAs on a SpiNNaker
board. As a result, it must be synthesised three times with the `FPGA_ID`
parameter set to `0`, `1` and `2` respectively. Care must be taken to load only
the correct image onto each FPGA since some pins that are inputs for one FPGA
are outputs for another.

Target Platform
---------------

This design is for a Xilinx Spartan-6 XC6SLX45T FPGA in a FG(G)484 package with
a speed grade of -3, as fitted in the SpiNN-5 PCB.


Opening Designs in Xilinx ISE
-----------------------------

The Core Generator for the GTP tiles frustratingly generates example code
for the two GTP tiles which it then compiles and complains that they conflict
with eachother. Deleting these example files will result in warnings about
missing files however just deleting the contents of them will solve the problem.
A Bash snippet to be executed from this project's directory is listed below:

	for f in ipcore_dir/gtp_x*_y0_*/example_design/*; do echo "" > "$f"; done

Chipscope VIO
-------------

A chipscope virtual I/O port can be added to aid in debugging. Various internal
signals are exposed, see `spinnaker_fpgas_top.v`.


SPI Interface
-------------

A number of diagnostic registers associated with the HSS links are presented via
an SPI interface. There is one register bank per link (i.e. per SATA socket).
These banks can be found at the base addresses given in the table below.

	Link        FPGA Number  Base Address
	----------  -----------  ------------
	East        0            0x00000000
	South       0            0x00010000
	Periph 0    0            0x00020000
	South-West  1            0x00000000
	West        1            0x00010000
	Periph 1    1            0x00020000
	North       2            0x00000000
	North-East  2            0x00010000
	Periph 2    2            0x00020000

Offsets into each register bank are given below. All registers are read/written
as 32 bit words but the actual number of useful bits may differ.

	Name  Number Offset  Access  Size  Description
	----  ------ ------  ------  ----  ---------------------------------------------
	VERS       0   0x00  RO        32  Version {top 24: Module, bottom 8: Protocol}
	CRCE       1   0x04  RO        32  CRC error counter
	FRME       2   0x08  RO        32  Frame error counter
	BUSY       3   0x0C  RO        32  Packet dispatcher busy counter
	LNAK       4   0x10  RO        32  Local nack'd frame counter
	RNAK       5   0x14  RO        32  Remote nack counter
	LACK       6   0x18  RO        32  Local ack'd frame counter
	RACK       7   0x1C  RO        32  Remote ack counter
	LOOC       8   0x20  RO        32  Local out-of-credit counter
	ROOC       9   0x24  RO        32  Remote out-of-credit counter
	CRDT      10   0x28  RO         3  Credit
	SFRM      11   0x2C  RO        32  Frame assembler valid sent frame counter
	TFRM      12   0x30  RO        32  Frame transmitter frame counter
	DFRM      13   0x34  RO        32  Frame disassembler valid frame counter
	RFRM      14   0x38  RO        32  Packet dispatcher valid receieved frame ctr.
	EMPT      15   0x3C  RO         8  Empty frame assembler queues
	FULL      16   0x40  RO         8  Full frame assembler queues
	CFCL      17   0x44  RO         8  Local channel flow control status
	CFCR      18   0x48  RO         8  Remote channel flow control status
	IDSO      19   0x4C  RW        16  IDle Sentinel Output val. (sent in idle frms)
	IDSI      20   0x50  RO        16  IDle Sentinel Input val. (latest received)
	HAND      21   0x54  RO         2  Handshake {bit1: version err, bit0: complete}
	RECO      22   0x58  RO        32  Link reconnection (re-handshake) counter
	STOP      23   0x5C  RW         1  1 = Stop sending data frames (NB: Will still receive them)

If in doubt, a definitive definition of the SPI address space should be saught
from the code. The address decoding scheme used to select which HSS link is
queried is defined in `spinnaker_fpgas_address_decode.v`.  The addresses of the
available registers in each HSS link are defined in
`../../modules/hss_multiplexer/spio_hss_multiplexer_reg_bank.h`.  The SPI
protocol and its parameters are defined in `spinnaker_fpgas_spi.v`.

There is also a register bank associated with the top-level design at base
address: 0x00040000

	Name  Number Offset  Access  Size  Description
	----  ------ ------  ------  ----  ---------------------------------------------
	VERS       0   0x00  RO        32  Top-level design version
	FLAG       1   0x04  RO         6  Compile flags {   5: chip scope
	                                                 ,   4: peripheral support
	                                                 ,   3: ring support
	                                                 ,   2: north/south on front
	                                                 , 1-0: FPGA ID
	                                                 }
	PKEY       2   0x08  RW        32  Peripheral MC route key
	PMSK       3   0x0C  RW        32  Peripheral MC route mask
	SCRM       5   0x10  RW         4  Scrambler on  {   3: ring link
	                                                 ,   2: peripheral link
	                                                 ,   1: board-to-board link1
	                                                 ,   0: board-to-board link0
	                                                 }

Finally, there are SpiNNaker-link packet counters on all SpiNNaker - FPGA links.

The sent packet counters (FPGA -> SpiNNaker packets) are at base address: 0x00050000

	Name  Number Offset  Access  Size  Description
	----  ------ ------  ------  ----  ---------------------------------------------
	PSL0       0   0x00  RO        32  packets sent on link 0
	PSL1       1   0x04  RO        32  packets sent on link 1
	PSL2       2   0x08  RO        32  packets sent on link 2
	PSL3       3   0x0c  RO        32  packets sent on link 3
	PSL4       4   0x10  RO        32  packets sent on link 4
	PSL5       5   0x14  RO        32  packets sent on link 5
	PSL6       6   0x18  RO        32  packets sent on link 6
	PSL7       7   0x1c  RO        32  packets sent on link 7
	PSL8       8   0x20  RO        32  packets sent on link 8
	PSL9       9   0x24  RO        32  packets sent on link 9
	PSL10     10   0x28  RO        32  packets sent on link 10
	PSL11     11   0x2c  RO        32  packets sent on link 11
	PSL12     12   0x30  RO        32  packets sent on link 12
	PSL13     13   0x34  RO        32  packets sent on link 13
	PSL14     14   0x38  RO        32  packets sent on link 14
	PSL15     15   0x3c  RO        32  packets sent on link 15

The received packet counters (SpiNNaker -> FPGA packets) are at base address: 0x00060000

	Name  Number Offset  Access  Size  Description
	----  ------ ------  ------  ----  ---------------------------------------------
	PRL0       0   0x00  RO        32  packets received on link 0
	PRL1       1   0x04  RO        32  packets received on link 1
	PRL2       2   0x08  RO        32  packets received on link 2
	PRL3       3   0x0c  RO        32  packets received on link 3
	PRL4       4   0x10  RO        32  packets received on link 4
	PRL5       5   0x14  RO        32  packets received on link 5
	PRL6       6   0x18  RO        32  packets received on link 6
	PRL7       7   0x1c  RO        32  packets received on link 7
	PRL8       8   0x20  RO        32  packets received on link 8
	PRL9       9   0x24  RO        32  packets received on link 9
	PRL10     10   0x28  RO        32  packets received on link 10
	PRL11     11   0x2c  RO        32  packets received on link 11
	PRL12     12   0x30  RO        32  packets received on link 12
	PRL13     13   0x34  RO        32  packets received on link 13
	PRL14     14   0x38  RO        32  packets received on link 14
	PRL15     15   0x3c  RO        32  packets received on link 15
