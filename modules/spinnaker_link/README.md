Asynchronous 2-of-7 SpiNNaker Link Interface
============================================

A pair of modules which provide communication to and from the asynchronous,
2-of-7 coded SpiNNaker chip-to-chip links and the standard vld/rdy signals used
by other spI/O modules.

A high-level overview of the modules are is shown below:

	                SpiNNaker Link Sender
	                `````````````````````
	,--------,                       ,--------,
	|        |                       |        | data[71:0]  
	|        |  2of7[6:0]            | SpiNN- |<=====/======
	| SpiNN- |<=====/================| aker   |
	| aker   |                       | Link   |     vld
	| Chip   |     ack     ,------,  | Sender |<------------
	|        |------------>| sync |->|        |
	|        |             `---^--'  |        |     rdy
	|        |                 |     |   /\   |------------>
	'--------'                 |     '--'--`--'
	                           |         |
	                   clk ----+---------+------------------...


	              SpiNNaker Link Receiver
	              ```````````````````````
	,--------,                       ,--------,
	|        |                       |        | data[71:0]  
	|        |  2of7[6:0]  ,------,  | SpiNN- |======/=====>
	| SpiNN- |======/=====>| sync |=>| aker   |
	| aker   |             `---^--'  | Link   |     vld
	| Chip   |     ack         |     | Recei- |------------>
	|        |<----------------(-----| ver    |
	|        |                 |     |        |     rdy
	|        |                 |     |   /\   |<------------
	'--------'                 |     '--'--`--'
	                           |         |
	                   clk ----+---------+------------------...


Synchronisers
-------------

Since the signalling between this module and the SpiNNaker system is
asynchronous, an appropriate synchroniser must be used in front of any incoming
signals from SpiNNaker. Synchroniser module`spio_spinnaker_link_sync` is
provided, which is intended to be used for the ack signal into the Sender and
for the data bus into the Receiver.

The module implements a `standard' 2-flop synchroniser. Note that, because the
data bus uses 2-of-7 transition signals to encode the data, each signal
inidcates its own validity and, therefore, it's safe to use a 2-flop synchroniser
on every data line. This, of course, would be unsafe in a binary-encoded data bus.


Link enabling/disabling
-----------------------

Links to SpiNNaker chips can be individually enabled/disabled. Disabling an FPGA
link to SpiNNaker may be useful to allow specific 2-of-7 link ports on a SpiNN5
board to be connected to external devices while other links are connected via
high-speed serial links to neighbouring boards as usual.

A disabled link tristates the associated output pins and holds in reset the
corresponding SpiNNaker link interface block in the FPGA. By default, all links
are disabled to prevent potential conflicts with external devices.

Link enable/disable is controlled through FPGA register SLEN. On reset, the
register is all-0s (i.e., all links disabled) and must be configured immediately
after power-up and before SpiNNaker system boot if any other configuration is
to be used. On a SpiNN5 board, the BMP can be used to configure automatically
the links after power up using the `xreg' command.

The FPGA STOP register should be preferred for the purposes of simply isolating
boards in a multi-board system.


Authors
-------

This module is taken straight from the 'SpiNNlink' project which provides the
connectivity between boards in large SpiNNaker systems. It was written by Luis
Plana, based on work by Jeff Pepper.
