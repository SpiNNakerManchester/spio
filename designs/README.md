Ready-to-Use FPGA Designs
=========================

This directory is a collection of ready-to-use designs for various FPGA
platforms which make use of the reusable modules in this library. See the
`README.md` for a general description of the design's function.

Opening Designs in Xilinx ISE
-----------------------------

To minimise the possibility of weird breakages due to incompatible Xilinx
software versions, the repository only includes Verilog source files (`*.v` and
`*.h`) and IPCORE Core Generator files (`ipcore_dir/*.xco`). As a result, when
working with a freshly checked out version of the repository, you must create
your own Xilinx project and include the relevant files yourself as follows:

1. Select 'File', 'New Project'.
2. Initially set the 'Location' box to your `spio/designs` directory.
3. Enter the name of the design you went to open in the 'Name' field, this will
   change the Location and working directory boxes accordingly.
4. On the next page, fill in the FPGA model, package and speed grade as
   described in the design's `README.md`.
5. Once the Wizard is complete, Right-click the FPGA name in the project
   hierarchy viewer and select 'Add Source...'.
6. Add the Verilog (`*.v` and `*.h`) and IPCORE Core Generator files
   (`ipcore_dir/*.xco`) for the design.
7. Add any other files mentioned by the `README.md` in the same way, for example
   modules.
8. See the design's `README.md` for any further notes.
9. At this point you should have a working project which synthesises and
   simulates.

You may safely ignore any import errors relating to included files since these
are due to ISE having already found these files before they were imported
explicitly.
