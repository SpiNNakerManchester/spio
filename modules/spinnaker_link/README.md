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


Authors
-------

This module is taken straight from the 'SpiNNlink' project which provides the
connectivity between boards in large SpiNNaker systems. It was written by Luis
Plana, based on work by Jeff Pepper.
