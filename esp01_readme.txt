ESP01 1MB needs these envs to be set while building and flashing
export FLASH_SIZE=8; 
export export HOMEKIT_SPI_FLASH_BASE_ADDR=0x7a000;


export FLASH_SIZE=8; export export HOMEKIT_SPI_FLASH_BASE_ADDR=0x7a000; make -C examples/rodswitch flash
