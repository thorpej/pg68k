# Phaethon 1 - a 68010-based Unix machine

The Phaethon 1 (pg68010 Mk I) is a 68010-based Unix-capable computer,
loosely inspired by the Sun 2 workstations.  It was specifically
designed with the NetBSD operating system in mind, but Linux and
older Unix SVR3 releases would be possible, as well.

The Phaethon 1 is named for the son of Helios, the Sun god, who crashed
the chariot because he didn't know what he was doing.  Why did I choose
to make a Unix machine with a 68010?  Because it gave me the opportunity
(actually, it forced me) to design an MMU capable of demand-paged virtual
memory, something I've long wanted to do.

The heart of the Phaethon 1 is the MMU, or Memory Management Unit.  All
regular (User/Supervisor Program/Data, Function Codes 1, 2, 5, and 6)
bus cycles are controlled by the MMU, and except for the earliest
bootstrap stages controlled by the firmware, the MMU must be enabled
in order for the system to function at all.  Every address generated
by the CPU for the four relevant FCs is treated as a virtual address;
physical addresses exist only on the downstream side of the MMU.  On
the upstream (CPU) side of the MMU, address lines are referred to as
VA[1..23], and on the downstream side, address lines are referred to
as A[1..27].

The overall architecture of the system is fairly simple.  The system
has a 28-bit (256MB) physical address space that is mapped by the MMU
into the 68010's 24-bit (16MB) addressing capability.  This 28 bit
physical address space is divided up into four 64MB regions, selected
by bits 26 and 27 of the address:

* 00 -- RAM (on-board and expansion)
* 01 -- ROM (system firmware)
* 10 -- on-board I/O devices
* 11 -- expansion I/O ("not quite VME")

In order to keep things simple, only the CPU can master the bus.
None of the I/O devices on the Phaethon 1 have DMA capability.

In addition to the 4 physical address regions, the Phaethon 1 has a
"control space" selected by FC 4.  This space, accessible using the
`MOVES` instruction, is where the MMU resides, in addition to some
configuration registers and a diagnostic display register.

## The RAM region

The RAM subsystem is very simple.  There is 8MB of SRAM on the system
board at physical address $0000.0000, arranged as 4 banks of 2MB each,
each bank having a 1MB SRAM chip attached to each of the 2 68010 byte
lanes (upper and lower).

Memory address decoding and strobe logic is handled by an ATF1504AS
PLCC44 CPLD programmed with [memctl010](cpld-files/memctl010.v).
This CPLD also performs this function for the ROM region, which is
described in the next section.

A[23..25] == 0 selects the on-board RAM.  For all other values of
A[23..25], expansion RAM is selected.  When expansion RAM is selected,
the [vmeglue](gal-files/vmeglue.gal) GAL sets the VMEbus Address
Modifier to a value in the range of `$11` to `$1f`, with the lower
3 bits being taken from A[23..25], thus allowing expansion RAM to
phyically reside on the nqVME bus without occupying nqVME address
space.

## The ROM region

The ROM region is comprised of a pair of 512KB flash ROM chips,
each connected to one of the 68010 byte lanes, thus providing 1MB
of system ROM.  Address decoding and strobe logic for the ROM is
handled by the same CPLD that handles those duties for the RAM
region.

A key difference in the ROM case is that all regular bus cycles
select the ROM when the MMU is disabled.  The system ROM is linked
at address $F00000, but when the MMU is off even absolute
references will work during early bootstrap.  Operating in this
way facilitates having the vector table at the correct location
when the CPU comes out of reset.  Note that this means that the
stack and all I/O devices are inaccessible by the system firmware
until it has bootstrapped virtual memory and enabled the MMU.

## The on-board I/O region

The on-board I/O region consists of the following:

* A TL16C2552 dual-UART, port B at $800.0000 ("uart0") and
port A at $800.0100 ("uart1").  This arrangement is due to
the on-board address decoding of the TL16C2552.
* A system timer (integrated into the I/O controller) at
$800.0200.
* A PCF8584 I2C controller at $800.0300.
* An ATA disk interface at $800.0400 (CS1FX-, command) and
$800.0410 (CS3FX-, control).

Note that all of on-board I/O devices can be mapped with a
single 4KB virtual page.  The on-board I/O space is also
incompletely-decoded, and will repeat every 4KB.

The [ioctl010](cpld-files/ioctl010.v) I/O controller, implemented
in an ATF1508AS TQFP100 CPLD, performs the following functions:

* Address decoding and selection logic for the on-board
peripherals.
* Bus cycle state machine for the on-board peripherals.
* Implements a 16-bit count-down timer running at 1/16th the
CPU clock frequency, intended for use as the system timer
by the operating system.
* An interrupt controller.

### System timer

The system timer is a basic 16-bit count-down timer that counts
at 1/16th the CPU clock frequency.  It is specifically designed
to be used as the "hardclock" timer that drives scheduling and
time-keeping in the Unix operating system, and the interface to
the timer was designed for maximum convenience for that purpose.

The interface to the timer is via 3 8-bit registers:

* _Timer_CSR_ - control and status register, comprised of a
read-write _Enable_ bit and a read-only _Pending_ bit.
* _Timer_LSB_ - timer count least-significant byte
* _Timer_MSB_ - timer count most-significant byte

The general flow for using the system timer is as follows:

* Determine the number of ticks required for the desired clock
rate.  This value must be in the range of 1-65535.
* Program the tick count into the _Timer_LSB_ and _Timer_MSB_
registers.  The order in which these registers are programmed
does not matter, and writing to either of them implicitly
disables the timer, so no spurious timer expirations will
occur.
* Enable the timer by setting the _Enable_ bit in the
_Timer_CSR_ register.

The timer will now interrupt at the period selected by the
programmed tick count.  When the timer expires, the programmed
value is automatically re-loaded so that any delay in servicing
the timer interrupt will not result in clock skew.  The interrupt
is level-sensitive and the interrupt handler must acknowledge the
interrupt by reading _Timer_CSR_.  Software may also poll for
timer expiration by reading the _Pending_ bit in _Timer_CSR_.  This
can be used, for example, to calibrate a divisor for a software
`delay()` function.

### Interrupt controller

The interrupt controller is a simple priority encoder that takes
the the various interrupt inputs and encodes them to a 68010
interrupt priority level 1-7.  After encoding the interrupt
priority for the CPU and placing the value into the 68010's
IPL[0..2] inputs, the interrupt controller responds to interrupt
acknowledge cycles from the CPU by assertint it's `/AVEC` output
which indicates to the 68010 that the interrupt should be
auto-vectored.  On the Phaethon 1, `/AVEC` is connected to the
CPU's `/VPA` input because auto-vectored interrupts were a key
component of the 6800 peripheral interface on the 68000 and
68010.

The interrupts for the on-board peripherals are hard-wired in
the interrupt controller for maximum convenience for a
Unix-like operating system:

* The system timer interrupts at IPL 6.
* The UARTs interrupt at IPL 5.
* The I2C controller interrupts at IPL 5.
* The ATA disk interface interrupts at IPL 3.

In addition to the on-board peripheral interrupt sources, 5
external interrupts are provided, IRQ[3..7].  These are
active-low level-sensitive inputs intended to be driven by
open-drain interrupt outputs, which facilitates a wired-OR
shared interrupt system.

In addition to these external interrupt inputs, there is a
software-controlled interrupt mechanism.  This consists of
two registers in the Phaethon 1 control space that allow
IPLs 1 and 2 to be asserted and negated by software.  This
will be discussed further in the section describing control
space.

The interrupt controller does not support vectored interrupts.
This has implications for the "not quite VME" expansion
interface, discussed in the next section.

## Expansion I/O ("not quite VME")

The expansion interface, as the name implies, is based upon
the VMEbus interface standard.  Electrically, it is compatible
with VMEbus expansion cards that use only the P1 VMEbus
connector.  However, nqVME differs from standard VMEbus in
a few important ways:

* The Phaethon 1 does not use a separate backplane; the
expansion connectors are built right into the system board,
and the role of the system controller is always played by
the Phaethon 1 system board.
* While the DIN41612 connector used is the same, the nqVME
form-factor is more akin to an ISA expansion card that you'd
put into a PC, with the interface connector facing down towards
the system board, and external connections facing towards the
back of the system board.
* nqVME does not support the standard VMEbus arbitration signals.
Only the CPU is ever allowed to master the bus.  Any standard
bus-mastering VMEbus card that attempted to request the bus would
be met with silence.
* nqVME does not support vectored interrupts.  None of the
VMEbus interrupt chain signals are present, and nqVME expansion
boards will not even see interrupt acknowledge cycles because
the nqVME bus interface is left in a high-Z state unless the
nqVME bus is explcitly selected (either for RAM expansion or
I/O) in order to reduce power consumption by the active
bus termination circuitry.

With that, however, nqVME should be perfectly adequate for the
intended use cases on the Phaethon 1: RAM expansion and a
network interface based on the RTL8019AS (NE2000-compatible)
Ethernet chip.  A "workstation board" with a keyboard and
mouse interface and a bitmapped display may also follow.

## Control space

Integral to the Phaethon 1's overall design is the system board
"control space".  Control space is a seperate address space
for low-level system control.  Access to control space is via
the `MOVES` instruction when the `SFC` and `DFC` registers are
set to `4`.

Address decoding for control space is split among several
functional blocks:

* The MMU decodes its own control space addresses.
* The I/O controller decodes control space for access to the
software interrupt registers.
* A GAL decodes addresses and provides glue logic for the
remainder of the control space registers:
  * a 2-digit 7-segment diagnostic display
  * 16 bits worth of configuration DIP switches
  * the 8-bit System Enable Register, which contains:
    * MMU enable bit
    * interrupt enable bit
    * 6 software-defined bits, 2 of which are used by the
      system firmware to control reboot behavior

The address layout of board control space is:

    xxxx.xxxx xxxx.xxxx xxxx.000x
                             ^^^
                Board Control Space selector
    
    xxxx.xxxx xxxx.xxxx 0000.0000 - 7-segment display (upper)
    xxxx.xxxx xxxx.xxxx 0000.0001 - 7-segment display (lower)
    xxxx.xxxx xxxx.xxxx 0001.000x - Configuration switches
    xxxx.xxxx xxxx.xxxx 0010.0000 - System Enable register

    xxxx.xxxx xxxx.xxxx 0100.0000 - Interrupt Set
    xxxx.xxxx xxxx.xxxx 0101.0000 - Interrupt Clear

## Memory Management Unit

The Phaethon 1 MMU is a 2-level paged type MMU with a 4KB page size,
per-page write and privilege protection, modified and referenced
tracking, and ample resources capable of fully mapping 64 virtual
memory contexts (Unix processes) concurrently.  Standard virtual memory
resource sharing techniques can be used to support a larger number of
concurrent Unix processes.

The inspiration of the design was taken from the Sun 3 MMU, and the
Phaethon 1 MMU uses similar nomenclature, although there are some
key differences between the Sun 3 MMU and this MMU, and the they
are not software-compatible.

The MMU is built around 3 SRAM chips:

* 1x ISSI IS61C3216AL-12KLI 32K x 16 12ns SRAM, used for the Segment Map
* 2x Alliance AS7C4098A-12JIN 256K x 16 12ns SRAM, used for the Page Map

Segment Map entries are 16 bits wide, and Page Map entries are 32 bits
wide, and so based on the sizes of the SRAMs, there are 32,768 total
Segment Map entries and 262,144 Page Map entries.

In addition to these SRAMs, the MMU has:

* ATF1508AS TQFP100 CPLD implementing all of the MMU logic, internal
registers, control space address decoding, SRAM control logic, and bus
cycle state machine.
* Bus transceivers that allow the Segment Map and Page Map entries to
be accessed by the CPU.
* Transparent latches to ensure the stability of the PME index while
the MMU writes back the upper 16 bits of the PME to update the Modified
and Referenced PME bits.
* Muxes to select the source of the PME index (either the latched
value during an address translation, or a virtual address from the
CPU when the CPU needs to read or write a PME).
* Muxes to select the system address output of the MMU (translated
or untranslated address).

Many standard 74xx parts were used in the MMU in order to better
demonstrate how it works.  Significant board space could have been
saved by condensing all of the transceiver and mux logic into another
ATF1508AS CPLD, and a future version of this MMU for the 68020 CPU
may do exactly that.

### Programming model

There are 3 basic parts which make up a virtual address translation:

* Context - a number in range of 0-63 that identifies a virtual memory
context (Unix process).  All supervisor access is hard-wired to use
context 0.  The context selects a range in the Segment Map.  Because
there are 64 total contexts and 32,768 Segment Map entries, a context
is comprised of 512 segments.
* Segment Map - The Segment Map divides the address space into 512
segments.  Because the total virtual address space is 16MB, 512
segments yields a segment size of 32KB.  Each 16-bit Segment Map
entry (SME) contains a Valid bit and a 15-bit value indicating the Page
Map Entry Group that maps this segment.  A Page Map Entry Group (PMEG)
is a group of 8 Page Map entries, each mapping a 4KB page.
* Page Map - The Page Map maps individual physical pages.  The Page
Map is arranged into 32,768 Page Map Entry Groups (PMEGs) of 8 Page
Map entries (PMEs).  The PMEs have the following format:

```
   31          28   26         23         20 19 16 15                0
  | V | W | K |  (r)  | R | M | s3 s2 s1 s0 | (r) | Page Frame Number |
    a   r   e     e     e   o   (software      e
    l   i   r     s     f   d      defined)    s
    i   t   n     e     e   i                  e
    d   e   e     r     r   f                  r
            l     v     e   i                  v
                  e     n   e                  e
                  d     c   d                  d
                        e
                        d
```
Conceptually, putting it in terms of the Motorola 68851 PMMU configured
as a 2-level tree, the context register is like the User Root Pointer,
the Segment Map is like an A table, and each PMEG is like a B table.

The MMU registers, Segment Map, and Page Map are accessed using the
Phaethon 1's control space:

```
                     selector bits
                          vvv
 xxxx.xxxx xxxx.xxxx xxxx.000x   non-MMU control space (ignored)
 SSSS.SSSS Sxxx.xxxx xxxx.0010   SegMap entry for Context 0
 SSSS.SSSS Sxxx.xxxx xxxx.0100   SegMap entry (relative to Context Reg)
 xxxx.xxxx xxxx.xxxx xxxx.0110   Context Register (byte)
 xxPP.PPPP PPPP.PPPP Pppp.1000   PageMap entry (upper word)
 xxPP.PPPP PPPP.PPPP Pppp.1010   PageMap entry (lower word)
   ^^^^^^^^^^^^^^^^^^^|||
            PMEG      |||
                      ^^^
               Entry within PMEG
 xxxx.xxxx xxxx.xxxx xxxx.1100   Bus Error Register (byte)
 xxxx.xxxx xxxx.xxxx xxxx.1110   (unused; reserved)
```

By placing the Segment Map index at the top of the virtual address,
we eliminiate the need for a mux for that address (because that's
where it lives naturally).  A separate address range for context 0's
Segment Map is provided because it is expectedto be a hot path in the
operating system kernel, and eliminates the need to modify the context
register to modify the kernel's address space.

The Page Map entry index, unlike the Segment Map index, does require
a mux.  This stands in contrast to the Sun 3 MMU, which indexed Page
Map entries via the normal address translation mechanism.  The Sun 3
approach is not unreasonable, but it requires a valid Segment Map entry
to access any Page Map entries, which risks uninitialized Page Map
entries being available for address translation, which I preferred
to avoid.

### Address translation

Address translation follows the following flow:

```
 +---------------+--------------------+
 | Context [5:0] | Segment [VA 23:15] |
 +---------------+--------------------+--> 15-bit SegMap index --+
                                                                 |
 +---------------------------------------------------------------+
 |
 +--> SegMap entry contains 15-bit PMEG number --+
                                                 |
 +-----------------------------------------------+
 |
 +-------------------------+
 |      PageMap index      |
 | (PMEG << 3) | VA[14:12] |
 +-------------------------+--> 18-bit PageMap index --+
                                                       |
 +-----------------------------------------------------+
 |
 +--> PageMap entry (32-bits) (lower 16 bits are PFN) -----+
                                                           |
 +---------------------------------------------------------+
 |
 +--> (PFN << 12) | VA[11:0] -> Physical address
```

### Validty and permission checking

Validity and permission checks happen in parallel with address
translation.  The upper 16 bits of the PME contain XXX more to come.
