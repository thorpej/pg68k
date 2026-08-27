# Changes between the hardware revisions

## Rev B

* Correct the schematic and PCB error described in Errata A.1.
* Correct the reset circuit issue described in Errata A.2 with the
  addition of Q801 and Q802 and their associated pull resistors.
* Change the real-time clock chip from a DS3231 to a DS3232.  The chips
  are equivalent except the DS3232 also has user NVRAM.
* The SOJ-44 package for the 61C3216 SRAM used for the MMU SegMap has
  been EOL'd.  The board has been updated for the TSOP-II-44 package.
