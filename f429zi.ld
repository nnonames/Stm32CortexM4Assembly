MEMORY {
    FLASH   : ORIGIN = 0x08000000, LENGTH = 2M
    CCMRAM  : ORIGIN = 0x10000000, LENGTH = 64K 
    RAM     : ORIGIN = 0x20000000, LENGTH = 192K
}

SECTIONS {
    .VectorTable : { *(.VectorTable) } >FLASH

    .text : { 
        *(.text)
        . = ALIGN(4);
    } >FLASH
    
    .data (NOLOAD) : { 
        . = . + 0x400;
        _StackEnd = .;
        *(.data)
    } >RAM
}
