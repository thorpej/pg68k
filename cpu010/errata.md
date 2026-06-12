# ERRATA

This documents the errors made on the various Phaethon 1 hardware versions.

## Rev A

There is an error in the NQVME slot wiring.  Pin a7 is supposed to be D6
and pin a8 is supposed to be D7, but on Rev A, these pins are reversed due
to a cut-and-paste error in the schematic.  On Rev A boards, this will be
corrected by cutting the horizontal traces leading to pins a7 and a8 of
J701 and connecting the following bodge wires on the back side of the board:

* J701 a7 -> U702 12
* J701 a8 -> U702 11

Because all 3 slots have the error and J702 and J703 are fed from J701,
correcting J701 will fix all 3 slots.
