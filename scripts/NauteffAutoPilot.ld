/***************************************
 * stm32 nauteff.ld
 ***************************************/
 


/* Memory layout for an STM32F446XX */
MEMORY
{
   FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
   RAM     (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

ENTRY(start)
 
STACK_SIZE = 512;
_estack = ORIGIN(RAM) + LENGTH(RAM);
_sstack = _estack - STACK_SIZE;
 
/* Output sections */
SECTIONS
{
    .text :
    {
        _stext = . ;
        *(.vector_table)     /* ISR Vector Table first */
        *(.text)             /* Program code */
        *(.text.*)
        _srodata = .;
        *(.rodata)           /* Constants */
	. = ALIGN(4);
        _erodata = .;
        _etext = .;
        _sidata = _etext;    /* Début des données initiales */
         
    } > FLASH
 
    _la_data = LOADADDR(.data);

    /* Initialized global and static variables */
    .data :
    {
        /*FILL (0xff) */
         
        _sdata = .;
        *(.data)
	. = ALIGN(4);
        _edata = .;
         
    } > RAM AT> FLASH
 
    /* Uninitialized global and static variables */
    .bss :
    {
        _sbss = .;
        __bss_start__ = .;
        *(.bss)
        /* *(.bss.*) */
        *(COMMON)
        _ebss = .;
        __bss_end__ = .;
        end = .;
    } > RAM
 
    /* Stack */
    . = _sstack;
    .stack :
    {
        . = . + STACK_SIZE;
    } > RAM

}

