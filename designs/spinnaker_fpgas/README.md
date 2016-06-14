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

BMP Firmware Version
--------------------

The Board Monitor Processor (BMP) of the SpiNNaker board you load this design
onto must have at least BMP firmware version v2.0.0. To determine what firmware
version your BMP is using you can use `bmpc` like so:

	$ echo sver | bmpc YOUR_BOARD_IP_HERE
	# bmpc - version 2.2.0
	YOUR_BOARD_IP_HERE:0 > sver
	BC&MP 2.0.0 at Spin5-BMP:0 (built Wed Mar 16 14:42:38 2016) [C=0, F=240, B=1]

In the example above, the last line indicates "BC&MP 2.0.0" is running on the
BMP meaning this board is sufficiently up-to-date. The `bmpc` command is
distributed as part of the SpiNNaker low-level software tools which can be
downloaded
[here](http://apt.cs.manchester.ac.uk/projects/SpiNNaker/downloads/).

Please contact the group if your board requires a firmware update and we'll
arrange this.

If your BMP firmware is out of date, this FPGA design will still work but on
power-on all of the FPGA-to-SpiNNaker-chip links will be disabled and the LEDs
will show an error pattern (on, flashing briefly off every second or so). You
can enable the links and clear the error signalling by writing the `SLEN` and
`LEDO` registers using the `xreg` command in `bmpc`. In BMP firmware versions
since v2.0.0, these registers are configured immediately after bitfile loading
is completed.

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
	PKEY       2   0x08  RW        32  Peripheral MC route key (default: 0xFFFFFFFF)
	PMSK       3   0x0C  RW        32  Peripheral MC route mask (default: 0x00000000)
	SCRM       4   0x10  RW         4  Scrambler on (default: 0xF)
	                                   { 3: ring link
	                                   , 2: peripheral link
	                                   , 1: board-to-board link1
	                                   , 0: board-to-board link0
	                                   }
	SLEN       5   0x14  RW        32  Enable SpiNNaker chip (2-of-7) link.
	                                   (Default: 0x00000000)
	                                   { 0: Link 0 SpiNN->FPGA enable
	                                   , 1: Link 0 FPGA->SpiNN enable
	                                   , 2: Link 1 SpiNN->FPGA enable
	                                   , 3: Link 1 FPGA->SpiNN enable
	                                   , ...
	                                   }
	LEDO       6   0x18  RW         8  Override status LED (default: 0x0F)
	                                   { 7: DIM_RING
	                                   , 6: DIM_PERIPH
	                                   , 5: DIM_B2B1
	                                   , 4: DIM_B2B0
	                                   , 3: FORCE_ERROR_RING
	                                   , 2: FORCE_ERROR_PERIPH
	                                   , 1: FORCE_ERROR_B2B1
	                                   , 0: FORCE_ERROR_B2B0
	                                   }

Note that disabling a link with the SLEN register tristates the associated link
pins and holds in reset the corresponding SpiNNaker link interface block in the
FPGA. This register may be useful to allow specific 2-of-7 link ports on,
e.g., SpiNN5 boards to be connected to external devices while other links are
connected via high-speed serial to neighbouring boards as usual. By default the
register is all-0s (i.e. all links disabled) and must be configured immediately
after power-up and before SpiNNaker system boot if any other configuration is
to be used. On a SpiNN5 board, the BMP can be used to configure automatically
the links after power up using the `xreg' command. The STOP register should be
preferred for the purposes of simply isolating boards in a multi-board system.

The LEDO register may be used to override the status displayed by LED to
indicate special device conditions. On reset, this register is configured to
force all LEDs to display the error signal (i.e. on with brief, infrequent
off-pulses). It is intended that once the SLEN register has been configured,
the LEDO register should also be written to clear the `FORCE_ERROR_*` bits,
indicating that the device has been configured and is ready to use. The `DIM_*`
bits of LEDO cause the specified LEDs to be dimmed. It is suggested that when
the 2-of-7 link exposed by SpiNN-5 boards is not being driven by the FPGA the
corresponding LED is dimmed.

Finally, there is a set of counters associated to every SpiNNaker - FPGA link.

The counters on FPGA -> SpiNNaker links are at base address: 0x00050000

	Name    Number Offset    Access  Size  Description
	----    ------ ------    ------  ----  ---------------------------------------------
	PSTL[n]      n 4*n       RO        32  packets sent on link n
	ACKE[n]      n 4*(n+16)  RO        32  unexpected acknowledges on link n
	TMOE[n]      n 4*(n+32)  RO        32  timeouts on link n

The counters on SpiNNaker -> FPGA links are at base address: 0x00060000

	Name    Number Offset    Access  Size  Description
	----    ------ ------    ------  ----  ---------------------------------------------
	PRVL[n]      n 4*n       RO        32  packets received on link n
	FLTE[n]      n 4*(n+16)  RO        32  incorrect 2-of-7 flits received on link n
	FRME[n]      n 4*(n+32)  RO        32  framing errors on link n
	GLTE[n]      n 4*(n+48)  RO        32  glitches detected on link n
