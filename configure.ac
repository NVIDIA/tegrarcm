#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.67])
AC_INIT([tegrarcm], [1.7], [amartin@nvidia.com])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([src/main.c])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
PKG_PROG_PKG_CONFIG

# Checks for libraries.
AC_CHECK_LIB([pthread],
	[pthread_create],
	[HAVE_PTHREAD=1],
	[AC_MSG_ERROR([libpthread is not installed.])])
PKG_CHECK_MODULES([LIBUSB], [libusb-1.0 >= 1.0.9])
AC_LANG(C++)
SAVED_LDFLAGS=$LDFLAGS
LDFLAGS="$LDFLAGS -lcryptopp -lpthread"
CRYPTOLIB=
AC_LINK_IFELSE(
	[AC_LANG_PROGRAM([#include <cryptopp/cryptlib.h>],
	[enum CryptoPP::ByteOrder bo = CryptoPP::LITTLE_ENDIAN_ORDER])],
	[CRYPTOLIB="cryptopp"],
	[AC_LINK_IFELSE(
		[AC_LANG_PROGRAM([#include <crypto++/cryptlib.h>],
		[enum CryptoPP::ByteOrder bo = CryptoPP::LITTLE_ENDIAN_ORDER])],
		[CRYPTOLIB="crypto++"],
		[AC_MSG_ERROR([libcrypto++/libcryptopp is not installed.])])]
)
AC_SUBST(CRYPTOLIB)
LDFLAGS=$SAVED_LDFLAGS
AC_LANG(C)

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/param.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SSIZE_T
AC_HEADER_STDBOOL
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset])

AC_CONFIG_FILES([
	Makefile
	src/Makefile
])
AC_OUTPUT
