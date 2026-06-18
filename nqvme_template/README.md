# NQVME Expansion Boards

These expansion boards are desgined to be simple and self-identifying,
not quite plug-and-play, but pretty close.

The 16MB VME address space is divided up into 8 2MB regions.  Each
board is configured to start at one of these regions (for boards
that fit within 2MB, a 3-bit DIP switch is perfect).  That switch
can be used with a 74HCT688 8-bit comparator, but also configures
the address of a 128 byte AT24C01 I2C EEPROM.  This EEPROM contains
a binary data blob structured like so:

    // all fields big-endian
    struct nqvme_reg {
        uint32_t  offset;
	uint32_t  size;
	uint32_t  flags;
    };

    #define REG_F_REGSHIFT        0x01 // reg-shift == 1
    #define REG_F_ONLY16          0x02 // only 16-bit cycles allowed
    #define REG_F_WTCACHE         0x04 // write-through cacheable
    // all other flags reserved

    struct nqvme_exphdr {
        uint8_t   magic[12];           // pg68k,nqvme\0
        uint8_t   compat[32];          // NUL-terminated, padded string
        struct nqvme_reg regs[4];      // device regions (size==0 terminates)
        uint8_t   irqs[4];             // interrupts used (irq=0 terminates)
        uint8_t   reserved[28];        // reserved for future expansion
	uint32_t  cksum;               // 32-bit xor checksum of struct
    };

The firmware will probe for valid EEPROMs, and based on which EEPROMs
respond, will compute the physical addresses of each board and construct
device-tree entries for the OS kernel; the OS kernel need not know anything
about the EEPROM format.
