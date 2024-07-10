# GSW141 Switch Chip Breakout

## Different Arrangements of the GSW141 Breakout PCB?

### RJ45 Jacks

* Internal Magnetics | External Magnetics
* LEDs | No LEDs
* 1000 (or higher) Base-T
    * I think 1G capable means that there are 4 twisted pairs
* "Tab Up" | "Tab Down"
    * This means the tab on the cable. When tab is down, the cat-5 pins all in a row point "up",
and vise versa when tab is up.
    * The GSW141 chip routes traces best with "tab down"
    * "Tab up" + on the reverse side of the board is equivalent to "tab down"
* How many ports per part



### Internal Magnetics

Use `ARJM11C7-502-KB-EW2` on mouser
* Tab up, so would have to be placed on the reverse side of the PCB
* $3.09 @ 10

4x 1 unit = $12.36


=====================================================================

Use `TMJG26945AENL` on mouser, 1x2 port
* Tab down
* $11.34 per unit

2x 1 unit = $22.68


=====================================================================

Use `TMJG46945AENL` on mouser, 1x4 port
* Tab down
* $18.84 per unit


### External Magnetics

* Mouser `1149848` 4 port RJ45 w/ LEDs, $4.34 each
    * Tab up, so would have to be placed on PCB reverse side
* Mouser `LAN2VSOD48351C2` dual port magnetics chip, ~$2 each

1 @ 4.34 + 2 @ $2 = $8.34

====================================================================

* Mouser `1149854` 2 port RJ45 w/LEDs, $2.78 each
    * Tab up, reverse PCB side
* Mouser `LAN2VSOD48351C2` dual port magnetics chip, ~$2 each

2 @ 2.78 + 2 @ 2 = $9.56


# Board TODO

[x] 4 port RJ45 jack
[x] RJ45 LEDs
[x] Management Interface
    * SPI, etc
    * Pin headers
    [x] Strapping pins?
        * `206-125ST` on mouser
        * or `DS01C-254-L-09BE`
[x] GPIO Pins, header
[x] RGMII/SGMII over edge connctor
[x] RGMII/SGMII header
    [x] Remove these headers actually
[x] Power regulation, 3.3V, 1.1V
[x] Power jack (latching) and jack
    * Mouser `T6634ST` barrel connector
    * 5.5 mm (out) x 2.1 mm (in)
    * Out 0v, inside, pos
    * Barrel Jack: `694106301002` for above
[x] Clock input
    * `ECS-400-12-33B2Q-JES-TR3` on mouser
    * Clock circuit cap calculation comes from https://blog.adafruit.com/2012/01/24/choosing-the-right-crystal-and-caps-for-your-design/
    * CL = C1 * C2 / (C1 + C2) + Cstray
    * Cstray is ~3 pF
    * If C1 = C2 = C, and CL = 12 pF for the above crystal
    * so C = 2(CL - Cstray) => C = 18 pF
[x] Power switch, latching
    * Digikey `PB400EEQR1BLK` maybe?
[x] Reset Button
    * button `TS21-34-035-BK-160-SMT-TR` on mouser
[ ] Power indicator LEDs
[x] GSW Power Startup sequence
    * RC delay from 3.3 V into 1.1V enable
[ ] Make sure trace clearance is okay
[ ] Power off?
[x] Blank R for RESREF and RCAL
[ ] Diff Pairs that are off by more than 10s of mm should be length tuned
[ ] Empty Footprint for linear regulator(s)?


