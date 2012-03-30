AM_CFLAGS = -Wall -std=c99 -O2
AM_CPPFLAGS = -isystem /usr/include/$(CRYPTOLIB)

bin_PROGRAMS = tegrarcm
tegrarcm_SOURCES = main.c usb.c nv3p.c debug.c rcm.c aes-cmac.cpp aes-cmac.h  debug.h  nv3p.h  nv3p_status.h  rcm.h  tegra20-miniloader.h  tegra30-miniloader.h  usb.h
tegrarcm_LDADD = -lusb-1.0 -l$(CRYPTOLIB) -lpthread

