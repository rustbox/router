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

We will also want an amount of DRAM connected to the PHY to buffer incoming and outgoing ethernet frames as needed, and for any needed storage for the (soft) SoC.

So our FPGA needs to have a certain amount of I/O pins:
* Each ethernet port needs 12 (14, including the management pins per port), so 56 pins
* 28 address, 16 data lines, plus a handful for management for DRAM
* UART
* Total: upwards of ~110 IOs available

We'd also prefer an FPGA with an open toolchain, and one with reproducible, Verilog (et al) based sources that can be generated into a bitstream just using command line tools.
 
#### Frame Size

A standard ethernet frame size is 1500 bytes. So there should be enough internal embedded memory to hold some number of frames directly in internal memory before needing to be placed in DRAM. 

Perhaps memory for Cache for the CPU?

#### DRAM

The main constraint here is total bandwidth: in order to act as a packet buffer, the FPGA needs to be able to push/pull data from the internal memory and/or DRAM at 2P*1Gbit/s minimum, where P is the port count and the doubling factor accounts for both RX & TX ("enqueue" and "dequeue") of a packet. That means for a 5-port device, we'd need a bare minimum of 10Gbps bandwidth, ignoring any lower priority besides packet buffering. 

As with all bandwidth constraints, we can "factor" that 10Gbps number into a frequency and bus width in different ways. For example:

* A bus width of 16 DDR data lanes operating at ≥312.5MHz would provide just enough raw data bandwidth:
        $$16\ \mathrm{bits}\cdot 312.5\ \mathrm{MHz}\cdot 2\ \mathrm{bits/cycle} = 10\ \mathrm{Gbps}$$

  (not accounting here for "overhead" in putting data into & taking it out of the memory)
* A bus width of 32 DDR lanes would only have to operate at half the clock rate (156.25MHz) to achieve the same data throughput, and a 64-lane-wide bus could get away with <100MHz
* A bus width of 8 SDR lanes, on the other hand, would have to operate at a brisk 1.25GHz to maintain throughput for 5 ports.

Wikipedia has a handy table ["Comparison of DDR SDRAM generations"](https://en.wikipedia.org/wiki/DDR_SDRAM#Comparison), but note that they're talking about memory _modules_ with multiple chips and implicitly assuming a bus width of 64 (and 8 bits/byte) to come up with their peak bandwidth figures. The `MT/s` column is more useful to us, since we can multiply that by a bus width to get a peak throughput in bits per second. Conveniently, any of the DDR3 or later standards seem like they would offer enough raw bandwidth for our needs with a 16-bit-wide bus. 

Attempting to account for the overhead is a challenge left up to the reader, at present. See: https://github.com/rustbox/discussion/discussions/20#discussioncomment-10400134

#### Storage

Something something SPI flash? Maybe a microSD card? We haven't looked too closely at this one yet.

#### Manufacturability

FPGAs often come in BGA format packages which is at the limit of our manufacturing capabilities. We would prefer QFN style packages or wide pitch BGA.

So far the Lattice ECP5/ECP5-5G series FPGAs seem promising: https://www.latticesemi.com/Products/FPGAandCPLD/ECP5

### Management

External serial port to interact with the system over UART
Some sort of network-accessible interface would be quite handy as well.

## Development Plan

Seth bought a BeagleV-Fire which includes a Polar Fire FPGA and an ethernet port. We'd like to experiment with the FPGA on the BeagleV to get used to those systems and also experiment with routing between multiple ethernet ports.

To make a second port we will utilize the build in SYZYGY connector which routes pins to the FPGA. We've designed a SYZYGY "pod" board which an RJ45 and PHY (GPY111) and puts RGMII over the SYZYGY connector to connect to the BeagleV FPGA. This allows us to have two ports to experiment with.

