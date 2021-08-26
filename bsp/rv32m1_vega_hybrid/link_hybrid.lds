RISCV_STACK_SIZE = 0x1000;
ARM_STACK_SIZE = 0x400;

/* Specify the memory areas */
MEMORY
{
  m_vector_arm          (RX)  : ORIGIN = 0x01000000, LENGTH = 0x00040000
  m_vector_riscv        (RX)  : ORIGIN = 0x000FFF00, LENGTH = 0x00000100
  m_text                (RX)  : ORIGIN = 0x00000000, LENGTH = 0x000FFF00
  m_data                (RW)  : ORIGIN = 0x20000000, LENGTH = 0x00020000 - 0x1800
}

/* Define output sections */
SECTIONS
{
  .vectors_riscv :
  {
    riscv___VECTOR_TABLE = .;
    KEEP(*(.vectors))
  } > m_vector_riscv

  .vectors_arm :
  {
    arm___VECTOR_TABLE = .;
    KEEP(*(.isr_vector))
    KEEP(*(.interrupt_vector))
  } > m_vector_arm

  /* The program code and other data goes into internal flash */
  .text :
  {
    . += (1024 * 4);         /* Uncomment when M0/ZERO_RISCY are enabled */
    . = ALIGN(4);
    KEEP(*(.startup))
    . = ALIGN(4);
    riscv___user_vector = .;
    KEEP(*(user_vectors))

    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)
    *(.glue_7t)
    *(.gnu.linkonce.t*)
    *(.eh_frame)
    *(.init)
    *(.fini)

    /* section information for finsh shell */
    . = ALIGN(4);
    arm___fsymtab_start = .;
    KEEP(*(FSymTab))
    arm___fsymtab_end = .;
    . = ALIGN(4);
    arm___vsymtab_start = .;
    KEEP(*(VSymTab))
    arm___vsymtab_end = .;
    . = ALIGN(4);

    /* section information for initial. */
    . = ALIGN(4);
    arm___rt_init_start = .;
    KEEP(*(SORT(.rti_fn*)))
    arm___rt_init_end = .;
    . = ALIGN(4);

    PROVIDE(arm___ctors_start__ = .);
    /* old GCC version uses .ctors */
    KEEP(*(SORT(.ctors.*)))
    KEEP(*(.ctors))
    /* new GCC version uses .init_array */
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    PROVIDE(arm___ctors_end__ = .);

    . = ALIGN(4);
    arm__etext = .;

    /* section information for finsh shell */
    . = ALIGN(4);
    riscv___fsymtab_start = .;
    KEEP(*(FSymTab))
    riscv___fsymtab_end = .;
    . = ALIGN(4);
    riscv___vsymtab_start = .;
    KEEP(*(VSymTab))
    riscv___vsymtab_end = .;
    . = ALIGN(4);

    /* section information for initial. */
    . = ALIGN(4);
    riscv___rt_init_start = .;
    KEEP(*(SORT(.rti_fn*)))
    riscv___rt_init_end = .;
    . = ALIGN(4);
  } > m_text

  .riscv_preinit_array :
  {
    PROVIDE_HIDDEN (riscv___preinit_array_start = .);
    KEEP (*(.riscv_preinit_array*))
    PROVIDE_HIDDEN (riscv___preinit_array_end = .);
  } > m_text

  .riscv_init_array :
  {
    PROVIDE_HIDDEN (riscv___init_array_start = .);
    KEEP (*(SORT(.riscv_init_array.*)))
    KEEP (*(.riscv_init_array*))
    PROVIDE_HIDDEN (riscv___init_array_end = .);
  } > m_text

  .riscv_fini_array :
  {
    PROVIDE_HIDDEN (riscv___fini_array_start = .);
    KEEP (*(SORT(.riscv_fini_array.*)))
    KEEP (*(.riscv_fini_array*))
    PROVIDE_HIDDEN (riscv___fini_array_end = .);
  } > m_text

  riscv___etext = .;    /* define a global symbol at end of code */
  riscv___global_pointer = .;    /* define a global symbol at end of code */

  .ARM.extab :
  {
    *(.ARM.extab*)
  } > m_text

  /* The .ARM.exidx section is used for C++ exception handling. */
  /* .ARM.exidx is sorted, so has to go in its own output section.  */
  arm___exidx_start = .;

  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)

    /* This is used by the startup in order to initialize the .data secion */
    arm__sidata = .;
  } > m_text

  arm_text_end = ORIGIN(m_text) + LENGTH(m_text);
  riscv_text_end = ORIGIN(m_text) + LENGTH(m_text);
  arm___exidx_end = .;

  __DATA_ROM = .;

  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    PROVIDE(__dtors_start__ = .);
    KEEP(*(SORT(.dtors.*)))
    KEEP(*(.dtors))
    PROVIDE(__dtors_end__ = .);

    /* This is used by the startup in order to initialize the .data secion */
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* create a global symbol at data start */
    _sdata = . ;

    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    *(.gnu.linkonce.d*)
    *(.gnu.linkonce.d*)
    *(.sdata .sdata.*)
    *(.heapsram*)            /* This is only for the pulpino official test code. */

    __noncachedata_start__ = .;   /* create a global symbol at ncache data start */
    *(NonCacheable)
    __noncachedata_end__ = .;     /* define a global symbol at ncache data end */

    KEEP(*(.jcr*))
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
    _edata = .;
  } > m_data

  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);

  .arm_stack :
  {
    . = ALIGN(8);
    arm___StackLimit = .;
    . = . + ARM_STACK_SIZE;
    . = ALIGN(4);
    arm___StackTop = .;
    arm__estack = .;
  } > m_data

  .riscv_stack :
  {
    . = ALIGN(8);
    riscv___StackLimit = .;
    . += RISCV_STACK_SIZE;
    . = ALIGN(4);
    riscv___StackTop = .;
  } > m_data

  /* Initializes stack on the end of block */
  PROVIDE(riscv___stack = riscv___StackTop);

  /* Uninitialized data section */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;

    *(.bss)
    *(.bss*)
    *(.sbss)
    *(.sbss*)

    /* Include COMMON section (generated dynamically by linker) */
    *(COMMON)

    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;

    *(.arm_bss.init)
  } > m_data

  /* End of uninitalized data segment */
  _end = .;
  PROVIDE(end = .);
  PROVIDE(riscv_end = .);
  PROVIDE(arm_end = .);

  . = ALIGN(0x100);
  __heap_start = .;

  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
   * Symbols in the DWARF debugging sections are relative to the beginning
   * of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
}
