Status LED Signal Generator
===========================

This module attempts to suggest a standard LED status representation such that
designs interacting with SpiNNaker can use a consistent encoding for link state.

Patterns
--------

The following patterns are used (in order of priority)


### Error (e.g. protocol version mismatch, unconfigured): Brief, Inrequent brief off-pulses

	___________,,_________________________,,_________________________,,_______
	           ||                         ||                         ||

### Unconnected: Brief, infrequent on-pulses

	           ,,                         ,,                         ,,
	___________||_________________________||_________________________||_______

### Connected, Link Activity: Blinking.

	    ,____,    ,____,    ,____,    ,____,    ,____,    ,____,    ,____,
	____|    |____|    |____|    |____|    |____|    |____|    |____|    |____

### Connected, Idle: Slow throbbing (i.e. fading on/off)

	          ,,__,,                  ,,__,,                  ,,__,,
	__,,..--~~      ~~--..,,__,,..--~~      ~~--..,,__,,..--~~      ~~--..,,__

Author
------

Jonathan Heathcote
