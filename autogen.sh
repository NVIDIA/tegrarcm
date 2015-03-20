#!/bin/sh -e

srcdir=`dirname $0`
test -z "$srcdir" && srcdir=.

ORIGDIR=`pwd`
cd "$srcdir"

autoreconf --install --symlink || exit 1
cd "$ORIGDIR" || exit $?

$srcdir/configure "$@"
