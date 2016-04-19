/*
 * Copyright (c) 2011, NVIDIA CORPORATION
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <iomanip>
using std::hex;

#include <string>
using std::string;

#include <cstdlib>
using std::exit;

#define CRYPTOPP_INCLUDE_CRYPTLIB	<CRYPTOLIB_HEADER_PREFIX/cryptlib.h>
#define CRYPTOPP_INCLUDE_CMAC		<CRYPTOLIB_HEADER_PREFIX/cmac.h>
#define CRYPTOPP_INCLUDE_AES		<CRYPTOLIB_HEADER_PREFIX/aes.h>
#define CRYPTOPP_INCLUDE_HEX		<CRYPTOLIB_HEADER_PREFIX/hex.h>
#define CRYPTOPP_INCLUDE_FILTERS	<CRYPTOLIB_HEADER_PREFIX/filters.h>
#define CRYPTOPP_INCLUDE_SECBLOCK	<CRYPTOLIB_HEADER_PREFIX/secblock.h>

#include CRYPTOPP_INCLUDE_CRYPTLIB
#include CRYPTOPP_INCLUDE_CMAC
#include CRYPTOPP_INCLUDE_AES
#include CRYPTOPP_INCLUDE_HEX
#include CRYPTOPP_INCLUDE_FILTERS
#include CRYPTOPP_INCLUDE_SECBLOCK

using CryptoPP::Exception;
using CryptoPP::CMAC;
using CryptoPP::AES;
using CryptoPP::HexEncoder;
using CryptoPP::HexDecoder;
using CryptoPP::StringSink;
using CryptoPP::StringSource;
using CryptoPP::HashFilter;
using CryptoPP::HashVerificationFilter;
using CryptoPP::SecByteBlock;

extern "C" int cmac_hash(const unsigned char *msg, int len, unsigned char *cmac_buf)
{
	SecByteBlock key(NULL, AES::DEFAULT_KEYLENGTH);

	string plain((const char *)msg, len);
	string mac, encoded;

	try {
		CMAC<AES> cmac(key, key.size());

		StringSource(plain, true,
			new HashFilter(cmac,
				new StringSink(mac)
				) // HashFilter
			); // StringSource
	}
	catch(const CryptoPP::Exception& e) {
		cerr << e.what() << endl;
		return 1;
	}

	memcpy(cmac_buf, mac.data(), mac.length());

	return 0;
}
