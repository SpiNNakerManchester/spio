spinn_aer note LaTeX Source
===========================

This repository contains the LaTeX sources for the note entitled
'Interfacing AER devices to SpiNNaker using an FPGA'.


To Compile
----------

Build using the Makefile produces `spinn_aer.pdf`:

   	 make

To get rid of temporary files:

	 make clean

make requires `fig2dev` to convert figures in `fig` format to `pdf`


Files
-----

### Content

* `spinn_aer.tex` The LaTeX file.
* `appnote.tex` standard SpiNNaker note LaTeX formatting seetings. 

### Figures

* `cochlea_map.fig`
* `map3.fig`
* `out_map2.fig`
* `retina_maps.fig`
* `spinn_2_if_diag.fig`
