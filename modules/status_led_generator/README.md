Status LED Signal Generator
===========================

This module attempts to suggest a standard LED status driver such that designs
interacting with SpiNNaker can use a consistent encoding of link state.

The following patterns are used (in order of priority)


* Error (e.g. protocol version mismatch): Frequent, brief off-pulses
    ____,,_________________________,,_________________________,,______________
        ||                         ||                         ||

* Unconnected: Brief, infrequent on-pulses
        ,,                         ,,                         ,,
    ____||_________________________||_________________________||______________

* Connected, Active: Blinking.
        ,____,    ,____,    ,____,    ,____,    ,____,    ,____,    ,____,
    ____|    |____|    |____|    |____|    |____|    |____|    |____|    |____

* Connected, Idle: Slow throbbing (i.e. fading on/off)
              ,,__,,                  ,,__,,                  ,,__,,
    __,,..--~~      ~~--..,,__,,..--~~      ~~--..,,__,,..--~~      ~~--..,,__

Author
------

Jonathan Heathcote
