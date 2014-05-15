High-Speed-Serial (HSS) Multiplexer
===================================

A module to multiplex eight bi-directional streams of packets over a single
high-speed-serial link on a Spartan-6 FPGA.

A high-level overview is shown below:

	             FPGA A                     :                     FPGA B
	             ``````                     :                     ``````
	                                        :
	          data/vld/rdy                  :                 data/vld/rdy 
	,--------,             ,--------,       :       ,--------,             ,--------,
	|   A0   |<===========>|        |       :       |        |<===========>|   B0   |
	'--------'             | High   |       :       | High   |             '--------'
	,--------,             | Speed  |   SATA:Cable  | Speed  |             ,--------,
	|   A1   |<===========>| Serial |<======:======>| Serial |<===========>|   B1   |
	'--------'             | Multi- |       :       | Multi- |             '--------'
	   ...        ....     | plexer |       :       | plexer |    ....        ...    
	,--------,             | A      |       :       | B      |             ,--------,
	|   A7   |<===========>|        |       :       |        |<===========>|   B7   |
	'--------'             '--------'       :       '--------'             '--------'
	                                        :

This module has been designed to be connected to a Xilinx Spartan-6 FPGA
supporting the GTP-Transceiver v1.11 IP CORE block. It should, however, be
possible to port it to similar systems.


Configuring the Spartan-6 GTP Tile
----------------------------------

Below is an outline of a set of sensible or mandatory parameters for configuring
the GTP-Transceiver IP CORE for your design.

The advice in this guide comes from decidedly hard-won experience at the Capo
Caccia 2014 neuromorphic workshop with substantial or significant contributions
from:
* Luis Plana
* Jonathan Heathcote
* Taras Iakymchuk


### 0. Selecting the core

The core to use is the 'Spartan-6 FPGA GTP Transceiver Wizard', v1.11. The
version specified is known by the authors to be significant for certain
development boards (such as the Raggedstone 2), the previous version does not
appear to work.


### 1. GTP Placement and Clocking

The HSS multiplexer will happily operate on any of the four or more GTP
transceivers typically available in Spartan-6 FPGAs. You should make sure at
least one is made available (note that you must enable a dual GTP tile at a
time so may be left with an unused GTP).

The GTP tiles can accept an external differential clock source ('REFCLK0/1
X*Y*') via an `IBUFDS` which should be connected to the appropriate pins on the
FPGA.

You should look up the appropriate pins in the Spartan-6 manual and add matching
constraints to the UCF.

If you wish to use the same clock source for all links, set GTP0 for each tile
to the same `REFCLK*` and GTP1 to use the PLL of GTP0 (this saves powering two
PLLs in each tile).


### 2. Line Rate and Protocol Template

The dynamic reconfiguration port is not used by the module and so may be left
disabled.

You should not use a protocol template and instead use the "Start from scratch"
option.

The target line rate and reference clock should be set according to your own
wishes and hardware. FPGA boards intended to support S-ATA typically feature a
150 MHz clock source. You typically want matching TX and RX line rates. Note
that the rate used may be restricted by the speed grade of the FPGA. Grade -3
has been known to work with 3.0 Gbps and 1.5 Gbps with grade -2, though these
may not be the maximum possible.

Encoding/decoding should be set to 8b/10b.

Data path width must be set to 32 bits.


### 3. 8b/10b Optional Ports

The `RXCHARISCOMMA` and `RXCHARISK` RX optional ports are used by the module to
make detecting commas easier and also to allow out-of-band 8b/10b K-characters
to be received.

The other ports are not required by the module and may be left disabled.


### 4. Synchronisation and Clocking

The TX and RX buffers (aka elastic buffers) should be enabled. This takes care
of clock-domain crossing involved with reading out data from the high-speed
serial link and also makes the built-in clock correction facility possible.

The TXUSRCLK/RXUSRCLK source should be set to `REFCLKPLL` which exposes the GTP
tile's incoming clock on its GTPCLKOUT[0] pin. This clock can then be divided
and multiplied to provide clocks for the rest of the system.

PPM Offset can be set to upto +/- 500 to allow better tolerance to differences
in clock quality on either side of the link. (Apparently???)

None of the available optional ports are used by the module and may be left
disabled.


### 5. RX Comma Alignment

The 'Decode valid comma only' should be selected (this option essentially just
sanity-checks the comma settings that follow).

The module uses the K28.5 comma character with the 'plus' variant being
`0101111100` and the 'minus' variant being `1010000011`. The comma mask should
be set to `1111111111` to match the whole comma character. The 'Any Byte
Boundary' option should be selected for 'Align to...' to ensure that all
arriving commas will be treated as byte-alignment characters.

The optional ports `ENPCOMMAALIGN` and `ENNCOMMAALIGN` should be enabled and
tied off high. This enables automatic byte alignment within the GTP tile. The
other optional ports are not used and my be left disabled.


### 6. Preemphasis, Termination and Equalisation

The preemphasis level, main driver differential swing and wideband/highpass
ratio values should be tuned by hand to provide the best possible bit-error-rate
for the hardware at hand. Xilinx's
[IBERT](http://www.xilinx.com/products/intellectual-property/chipscope_ibert_spartan6_gtp.htm)
tool is able to perform parameter sweeps over these values and report which ones
perform best. This tool can, however, be buggy at times. Selecting the all-zeros
options will probably work.

Internal AC coupling should be disabled since most boards feature their own
coupling capacitors. Termination voltage should be set to VTTRX as this helps to
absorb reflections.

None of the available optional ports are used by the module and may be left
disabled.


### 7. RX OOB, PRBS and Loss of Sync

Out of Band (OOB) signal detection is not used by the module and so may be
disabled.

The `RXLOSSOFSYNC` optional port is used by the module to detect when the link
is byte aligned and transmitting valid data. The loss-of-sync state machine
status should be selected as the meaning of the `RXLOSSOFSYNC` port.

The default values of 128 errors required to lose sync and 8 good bytes to
reduce error count by 1 appear sensible in practice.

The Pseudo-Random Bit Sequence (PRBS) facilities are for testing purposes and
are not used by the module and so may be left disabled.


### 8. RX PCI Express, SATA features

Neither the PCI Express nor SATA features are used by the module and so settings
here may be left as defaults.

None of the available optional ports are used by the module and may be left
disabled. With that said, the `LOOPBACK` port allows the designer to connect up
internal loop-back link between the RX and TX ports of the transmitter. This can
be convenient for initial testing and so may be selected. The port should be
tied to zeros for normal operation.


### 9. Channel Bonding, Clock Correction

Though the module does not use channel bonding, it should be enabled to
work-around a bug in the GTP hard tile's loss-of-sync state machine (see [Xilinx
AR # 29218](http://www.xilinx.com/support/answers/29218.htm)) (which is used by
the module). The channel bonding sequence length should be set to 1 and the
sequence's first (and only byte) is a K-character with value `00000000`. Since
this is an invalid K-character, it cannot appear in the stream and thus channel
bonding remains functionally disabled.

The 'use clock correction' option should be selected and the sequence length set
to 4. The minimum and maximum latency may be left as the defaults. Only a single
clock correction sequence is used.


### 10. Clock Correction Sequence

Sequence 1, bytes 1 to 4 should be set to the K char `00011100` (K28.0) which is
inserted into the sequence periodically by the module to allow for clock
correction.



Clocking Wizard IP CORE
-----------------------

To generate the clocks for use by the GTP tile's FPGA logic interface, the
'Clocking Wizard' IP CORE can be used. This is known to work with version 3.6 of
the wizard.

The tile requires two positive-edge aligned clock sources in addition to its
external clock source: `TXUSRCLK` and `TXUSRCLK2`.  `TXUSRCLK2` should tick
every time a 32-bit block of data is transmitted (that is, every 40 bits sent
down the link after 8b/10b coding).  `TXUSRCLK` should tick every time a 8-bit
value (10 bits after 8b/10b coding) is transmitted down the link.

For an example, with a 3 Gbit/s link driven by an external 150 MHz clock source:

* `TXUSRCLK`  = 3 GHz / 10 = 300 MHz
* `TXUSRCLK2` = 3 GHz / 40 = 75 MHz

Since the FPGA interface lives in the `TXUSRCLK2` clock domain, logic
communicating with the tile will typically also be clocked by this signal.

The GTP tile exposes its input clock signal on `GTPCLKOUT[0]` (if the TXUSRCLK
Source and RXUSRCLK Source and set to `REFCLKPLL` as described in the previous
section). This clock source must be use used to drive the clocking wizard since.

More details of the GTP blocks' clocking requirements can be found in the
subsection "Connecting TXUSRCLK and TXUSRCLK2" in the "FPGA TX Interface"
section of the GTP.

### 1. Clocking Features/Input Clocks

The 'frequency synthesis' and 'phase alignment' options should be selected.
'Minimise power' and 'dynamic phase shift' should be deselected.

'Mode' should be set to 'Auto Selection'. Everyone loves automatic, sensible
defaults!

Jitter optimisation should be set to 'Balanced'.

The input clock frequency should be set to the frequency of the external clock
driving the GTP tiles. The source should be set to 'Global buffer' to allow the
module to be connected to the GTP tile's `GTPCLKOUT[0]` pin via a `BUFIO2`.


### 2. Output Clock Settings

As a minimum, outputs at the frequencies required by `TXUSRCLK` and `TXUSRCLK2`
should be selected. These should have a phase of 0 degrees, a 50% duty cycle and
drive `BUFG` nets.


### 3. I/O and Feedback

The `LOCKED` optional output may be useful to hold the multiplexer module in
reset while the clock is not ready.

All other optional ports are not required and so may be left disabled.

The clock feedback source should be set to 'Automatic control on-chip'.

### 4. Remaining Pages

The remaining pages of the wizard are for informational purposes only. No
advanced overrides are needed in the simple case.
