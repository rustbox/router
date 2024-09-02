# Router

Rustbox builds a router!

The goal of this project is to build an open source Router from scratch, using as many open source tooling as possible.

## Goals

Desired Capabilities and Design Goals:
* 4 (or 5?) ports
* 1 Gigabit along each port
* RISC-V CPU MAC
    * A goal here, too, is to have the MAC design itself be open, as much as possible
    * Learning FPGAs is a desired outcome and would allow us to have an open CPU and Ethernet hardware

## Design

Ultimately, the Router will be components soldered on a PCB placed in a metal enclosure.

### RJ45 

Ethernet will come in via RJ45 in a 4-port connector.
    * Internal magnetics or external?

### PHY

We're currently designing the PHY portion with the MaxLinear GPY111
  * MDI from the magnetics will connect to the PHY IC
  * The PHY will convert MDI  ([Media Dependent Interface](https://en.wikipedia.org/wiki/Medium-dependent_interface)) to RGMII ([Reduced Gigabit Media Independent Interface](https://en.wikipedia.org/wiki/Media-independent_interface#RGMII))

### MAC

The MAC is the [Media Access Interface](https://en.wikipedia.org/wiki/Medium_access_control), the "brains" of the router. We're thinking of using the standard [Chipyard](https://chipyard.readthedocs.io/en/latest/) RISC-V CPU written in [Chisel](https://www.chisel-lang.org/) with our own RGMII interface, all flashed onto an FPGA.

We will also want an amount of DRAM connected to the PHY to buffer incoming and outgoing ethernet frames as needed, and for any needed storage for the SoC.


So our FPGA needs to have:
* Each ethernet port has 12 (14, including the management pins per port), so 56
* 28 address, 16 data lines, plus a handful for management for DRAM
* UART
* Total: upwards of ~110 IOs available
 
#### Frame Size

A standard ethernet frame size is 1500 bytes. So there should be enough internal embedded memory to hold some number of frames directly in internal memory before needing to be placed in DRAM.

Perhaps memory for Cache for the CPU?

We'd also prefer an FPGA with an open toolchain, and one with reproducible, Verilog (et al) based sources that can be generated into a bitstream just using command line tools.

#### Manufacturability

FPGAs often come in BGA format packages which is at the limit of our manufacturing capabilities. We would prefer QFN style packages or wide pitch BGA.

So far the Lattice ECP5/ECP5-5G series FPGAs seem promising: https://www.latticesemi.com/Products/FPGAandCPLD/ECP5

### Management

External serial port to interact with the system over UART

## Development Plan

Seth bought a BeagleV-Fire which includes a Polar Fire FPGA and an ethernet port. We'd like to experiment with the FPGA on the BeagleV to get used to those systems and also experiment with routing between multiple ethernet ports.

To make a second port we will utilize the build in SYZYGY connector which routes pins to the FPGA. We've designed a SYZYGY "pod" board which an RJ45 and PHY (GPY111) and puts RGMII over the SYZYGY connector to connect to the BeagleV FPGA. This allows us to have two ports to experiment with.

