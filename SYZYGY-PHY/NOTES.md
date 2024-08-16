# Review Notes 2024-08-16

Goal: Reviewing the design for usability with BeagleV-Fire.

requested by @dougli1sqrd:

- [x] Power regulation
    - I compared it against §3.5.1.1 "Power Supply Using Integrated Switching Regulator" and the schematic appears to match what they've described.
- [~] inductor form factor
- [x] external clock circuitry

My notes:

- [x] Mechanical: which side is the connector on vs. magjack? will it fit the BeagleV-Fire?
    - [x] standoff amount? ~5mm
    - [~] check inductor height vs. those through hole leads
- [x] SYZYGY: do we have to do anything to tickle the "host" to e.g. send us power? 
    - Nope (for the BeagleV), it's all hard-wired in
- [x] GPY111 power sequencing (& pin-strapping)
    - Looks like we're missing the power-up sequence

## Schematic

- [x] RJ-45: is it ok?
    - [x] what's C17 doing there?
    - [x] it doesn't match the GPY111 datasheet §6.9.4 RJ45 Plug
- [x] GPY111: Missing $$R_{CAL}$$ from §6.9.5 
- [~] can we use the "Fabric I2C" or nah?
    - seems like yah, but we're moving out to other header pins anyhow 
- [x] how many caps do we need?



- [~] do we need to connect the clock lines especially?

> Preferred clock input I/Os connect external clock signals to the CCCs and/or the global clock network
through low-latency paths. Use these preferred clock inputs for connecting external clocks to the
clock inputs of PLLs, DLLs, and fabric logic. Using regular I/Os as clock inputs introduces high latency
on the path.
> 
> Each preferred clock input can be used either as a single-ended clock input or be paired with an
adjacent I/O to form a differential clock input.
via PolarFire Clocking User Guide §2.4 "Preferred Clock Inputs"

So: 1) what is a "preferred clock input" and 2) is "high latency" tolerable for a 125MHz (DDR) clock?

re (2), from the datasheet it looks like we're operating at 1/2 the "maximums" listed in table 4-41 "I/O Digital Receive Double Data Rate Switching Characteristics", so we may be able to get away with it.

There's also "regions" and "stripes" (different connections in the clock distribution circuitry), so we probably also want to aim for co-locating 

re (1): it looks like the pins named `_CLKIN_` are preferred clock inputs. On the BeagleV-Fire schematic, the pins that are mapped to `B0_HSIOxx` are:

- `B0_HSIO8P`: `HSIO8PB0/CCC_NE_CLKIN_N_10/CCC_NE_PLL0_OUT0`
- `B0_HSIO12P`: `HSIO12PB0/CLKIN_N_9/CCC_NE_CLKIN_N_9`
- `B0_HSIO18P`: `HSIO18PB0/CLKIN_N_7`
- `B0_HSIO30P`: `HSIO30PB0/CLKIN_N_3/CCC_NW_CLKIN_N_3`
- `B0_HSIO34P`: `HSIO34PB0/CCC_NW_CLKIN_N_1`

None of which go to the SYZYGY connector; `B0_HSIO12P` does go to the 3.3V to 1.8V (sic: 1V8) level translator though, which brings up:

- [x] what voltage level(s) do we need to talk?



- [ ] somewhere I saw a list of recommendations for "generic DDR signalling" in one of the microchip sheets...


### Signal Timing

- [ ] minimum delay of 1/1.2ns between clock edge and data edge (depending on direction)

that'd be roughly 

#### FPGA->PHY ("RX")



#### PHY->FPGA ("TX")

## Layout

PHY-> (mag jack) traces are length tuned within a pair (intra-pair), but not between pairs (inter-pair)

shortest: 8.5699mm
longest: 14.3362mm






No length tuning on RGMII signals?


## References

- https://openbeagle.org/beaglev-fire/beaglev-fire/-/blob/main/BeagleV-Fire_sch.pdf?ref_type=heads
- RGMII Interface Timing Budgets: https://www.ti.com/lit/an/snla243/snla243.pdf?ts=1723111000887
- https://embeddedhardwaredesign.com/what-is-rgmii-reduced-gigabit-media-independent-interface/
- https://syzygyfpga.io/wp-content/uploads/2023/09/Syzygy-Specification-V1p1p1.pdf
- https://docs.opalkelly.com/resources/syzygy-design-guide/design-checklist/
- https://ww1.microchip.com/downloads/aemDocuments/documents/FPGA/ProductDocuments/UserGuides/Microchip_PolarFire_FPGA_and_PolarFire_SoC_FPGA_User_IO_User_Guide_VC.pdf
- https://ww1.microchip.com/downloads/aemDocuments/documents/FPGA/ProductDocuments/UserGuides/Microchip_PolarFire_FPGA_and_PolarFire_SoC_FPGA_Clocking_Resources_User_Guide_VB.pdf

Not as useful:

- https://ww1.microchip.com/downloads/aemDocuments/documents/FPGA/ProductDocuments/UserGuides/microsemi_polarfire_fpga_1g_ethernet_solutions_user_guide_ug0687_v5.pdf


