# ERRATA

This documents the errors made on the various Phaethon 1 hardware versions.

## Rev A

### A.1 - NQVME signal swap

There is an error in the NQVME slot wiring.  Pin a7 is supposed to be D6
and pin a8 is supposed to be D7, but on Rev A, these pins are reversed due
to a cut-and-paste error in the schematic.  On Rev A boards, this will be
corrected by cutting the horizontal traces leading to pins a7 and a8 of
J701 and connecting the following bodge wires on the back side of the board:

* J701 a7 -> U702 12
* J701 a8 -> U702 11

Because all 3 slots have the error and J702 and J703 are fed from J701,
correcting J701 will fix all 3 slots.

### A.2 - Reset circuit

The reset circuit is based around a DS1813 reset controller.  The DS1813
is fed PWR_OK from the ATX-PSU to bring the system out of reset when the
power rails are stable.

Unfortunately, the PWR_OK signal is also driving an LED and is not able
to drive a voltage high enough to meet the DS1813 threshold voltage, so
the system never comes out of reset.

On Rev A boards, this will be worked-around by cutting the PWR_OK trace
on the back side of the board, below the power button near the via, and
bodging +5V from any nearby source (U404 or U402 are very convenient) to
the cathode (stripe side) of D202.  This will immediately start the power-
on-reset as soon as Vcc crosses the DS1813 threshold voltage rather than
waiting for PWR_OK, but this shouldn't cause any problems.

Rev B boards will get a more complicated fix that preserves the intended
PWR_OK functionality using N-FET/P-FET pair.
