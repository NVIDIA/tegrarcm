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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "rcm.h"
#include "aes-cmac.h"

static uint32_t rcm_get_pad_len(const struct rcm *rcm, uint32_t payload_len)
{
	uint32_t msg_len = rcm->message_size + payload_len;
	uint32_t pad_len = 0;

	// First, use padding to bump the message size up to the minimum.
	if (msg_len < RCM_MIN_MSG_LENGTH) {
		pad_len = RCM_MIN_MSG_LENGTH - msg_len;
		msg_len += pad_len;
	}

	/*
	 * Next, add any extra padding needed to bump the relevant subset
	 * of the data up to a multiple of 16 bytes.  Subtracting off the
	 * rcm_msg_t size handles the initial data that is not part of
	 * the hashing and encryption.
	 */
	pad_len += 16 - ((msg_len - rcm->message_size) & 0xf);

	return pad_len;
}

static void rcm_msg_pad(uint8_t *data, uint32_t len)
{
	if (!len)
		return;

	*data = 0x80;
	memset(data+1, 0, len-1);
}

static void rcm1_init_msg(const struct rcm *rcm, void *buf,
			  uint32_t msg_len, uint32_t opcode,
			  const void *args, uint32_t args_len,
			  uint32_t payload_len)
{
	rcm1_msg_t *msg = buf;
	uint32_t padding_len;

	padding_len = rcm_get_pad_len(rcm, payload_len);

	msg->len_insecure = sizeof(rcm1_msg_t) + payload_len + padding_len;

	memset(&msg->cmac_hash, 0x0, sizeof(msg->cmac_hash));
	memset(&msg->reserved, 0x0, sizeof(msg->reserved));

	msg->opcode = opcode;
	msg->len_secure = msg->len_insecure;
	msg->payload_len = payload_len;
	msg->rcm_version = RCM_VERSION_1;

	if (args_len)
		memcpy(msg->args, args, args_len);

	memset(msg->args + args_len, 0x0, sizeof(msg->args) - args_len);

	rcm_msg_pad(msg->padding, sizeof(msg->padding));
	rcm_msg_pad(buf + sizeof(rcm1_msg_t) + payload_len, padding_len);
}

static uint32_t rcm1_get_msg_len(const struct rcm *rcm, const void *msg)
{
	return ((const rcm1_msg_t *)msg)->len_insecure;
}

static int rcm1_sign_msg(const struct rcm *rcm, void *buf)
{
	rcm1_msg_t *msg = buf;
	uint32_t crypto_len;

	// signing does not include the len_insecure and
	// cmac_hash fields at the beginning of the message.
	crypto_len = msg->len_insecure - sizeof(msg->len_insecure) -
		     sizeof(msg->cmac_hash);
	if (crypto_len % RCM_AES_BLOCK_SIZE)
		return -EMSGSIZE;

	cmac_hash(msg->reserved, crypto_len, msg->cmac_hash);
	return 0;
}

const struct rcm rcm1 = {
	.version = RCM_VERSION_1,
	.message_size = sizeof(rcm1_msg_t),
	.init_msg = rcm1_init_msg,
	.get_msg_len = rcm1_get_msg_len,
	.sign_msg = rcm1_sign_msg,
};

static void rcm35_init_msg(const struct rcm *rcm, void *buf,
			   uint32_t msg_len, uint32_t opcode,
			   const void *args, uint32_t args_len,
			   uint32_t payload_len)
{
	rcm35_msg_t *msg = buf;
	uint32_t padding_len;

	padding_len = rcm_get_pad_len(rcm, payload_len);

	msg->len_insecure = sizeof(rcm35_msg_t) + payload_len + padding_len;

	memset(&msg->object_sig.cmac_hash, 0x0, sizeof(msg->object_sig.cmac_hash));
	memset(&msg->reserved, 0x0, sizeof(msg->reserved));

	msg->opcode = opcode;
	msg->len_secure = msg->len_insecure;
	msg->payload_len = payload_len;
	msg->rcm_version = RCM_VERSION_35;

	if (args_len)
		memcpy(msg->args, args, args_len);

	memset(msg->args + args_len, 0x0, sizeof(msg->args) - args_len);

	rcm_msg_pad(msg->padding, sizeof(msg->padding));
	rcm_msg_pad(buf + sizeof(rcm35_msg_t) + payload_len, padding_len);
}

static uint32_t rcm35_get_msg_len(const struct rcm *rcm, const void *msg)
{
	return ((const rcm35_msg_t *)msg)->len_insecure;
}

static int rcm35_sign_msg(const struct rcm *rcm, void *buf)
{
	rcm35_msg_t *msg = buf;
	uint32_t crypto_len;

	// signing does not include the len_insecure, modulus
	// and object signature at the beginning of the message
	crypto_len = msg->len_insecure - sizeof(msg->len_insecure) -
		     sizeof(msg->modulus) - sizeof(msg->object_sig);
	if (crypto_len % RCM_AES_BLOCK_SIZE)
		return -EMSGSIZE;

	cmac_hash(msg->reserved, crypto_len, msg->object_sig.cmac_hash);
	return 0;
}

const struct rcm rcm35 = {
	.version = RCM_VERSION_35,
	.message_size = sizeof(rcm35_msg_t),
	.init_msg = rcm35_init_msg,
	.get_msg_len = rcm35_get_msg_len,
	.sign_msg = rcm35_sign_msg,
};

static void rcm40_init_msg(const struct rcm *rcm, void *buf,
			   uint32_t msg_len, uint32_t opcode,
			   const void *args, uint32_t args_len,
			   uint32_t payload_len)
{
	rcm40_msg_t *msg = buf;
	uint32_t padding_len;

	padding_len = rcm_get_pad_len(rcm, payload_len);

	msg->len_insecure = sizeof(rcm40_msg_t) + payload_len + padding_len;

	memset(&msg->object_sig.cmac_hash, 0x0, sizeof(msg->object_sig.cmac_hash));
	memset(&msg->reserved, 0x0, sizeof(msg->reserved));

	msg->opcode = opcode;
	msg->len_secure = msg->len_insecure;
	msg->payload_len = payload_len;
	msg->rcm_version = RCM_VERSION_40;

	if (args_len)
		memcpy(msg->args, args, args_len);

	memset(msg->args + args_len, 0x0, sizeof(msg->args) - args_len);

	rcm_msg_pad(msg->padding, sizeof(msg->padding));
	rcm_msg_pad(buf + sizeof(rcm40_msg_t) + payload_len, padding_len);
}

static int rcm40_sign_msg(const struct rcm *rcm, void *buf)
{
	rcm40_msg_t *msg = buf;
	uint32_t crypto_len;

	// signing does not include the len_insecure, modulus
	// and object signature at the beginning of the message
	crypto_len = msg->len_insecure - sizeof(msg->len_insecure) -
		     sizeof(msg->modulus) - sizeof(msg->object_sig);
	if (crypto_len % RCM_AES_BLOCK_SIZE)
		return -EMSGSIZE;

	cmac_hash(msg->reserved, crypto_len, msg->object_sig.cmac_hash);
	return 0;
}

static uint32_t rcm40_get_msg_len(const struct rcm *rcm, const void *msg)
{
	return ((const rcm40_msg_t *)msg)->len_insecure;
}

const struct rcm rcm40 = {
	.version = RCM_VERSION_40,
	.message_size = sizeof(rcm40_msg_t),
	.init_msg = rcm40_init_msg,
	.get_msg_len = rcm40_get_msg_len,
	.sign_msg = rcm40_sign_msg,
};

static void rcm21_init_msg(const struct rcm *rcm, void *buf,
			   uint32_t msg_len, uint32_t opcode,
			   const void *args, uint32_t args_len,
			   uint32_t payload_len)
{
	rcm21_msg_t *msg = buf;
	uint32_t padding_len;

	padding_len = rcm_get_pad_len(rcm, payload_len);

	msg->len_insecure = sizeof(rcm21_msg_t) + payload_len + padding_len;
	msg->fsp.num = 0;
	memset(msg->fsp.key, 0x0, sizeof(msg->fsp.key));

	memset(&msg->object_sig.cmac_hash, 0x0, sizeof(msg->object_sig.cmac_hash));
	memset(&msg->reserved, 0x0, sizeof(msg->reserved));

	msg->opcode = opcode;
	msg->len_secure = msg->len_insecure;
	msg->payload_len = payload_len;
	msg->rcm_version = RCM_VERSION_21;

	if (args_len)
		memcpy(msg->args, args, args_len);

	memset(msg->args + args_len, 0x0, sizeof(msg->args) - args_len);

	msg->secure_debug_control = 0;
	msg->secure_prov_key_num = 0;

	rcm_msg_pad(msg->padding, sizeof(msg->padding));
	rcm_msg_pad(buf + sizeof(rcm21_msg_t) + payload_len, padding_len);
}

static uint32_t rcm21_get_msg_len(const struct rcm *rcm, const void *msg)
{
	return ((const rcm21_msg_t *)msg)->len_insecure;
}

static int rcm21_sign_msg(const struct rcm *rcm, void *buf)
{
	rcm21_msg_t *msg = buf;
	uint32_t crypto_len;

	// signing does not include the len_insecure, modulus
	// and object signature at the beginning of the message
	crypto_len = msg->len_insecure - sizeof(msg->len_insecure) -
		     sizeof(msg->fsp) - sizeof(msg->modulus) -
		     sizeof(msg->object_sig);
	if (crypto_len % RCM_AES_BLOCK_SIZE)
		return -EMSGSIZE;

	cmac_hash(msg->reserved, crypto_len, msg->object_sig.cmac_hash);
	return 0;
}

const struct rcm rcm21 = {
	.version = RCM_VERSION_21,
	.message_size = sizeof(rcm21_msg_t),
	.init_msg = rcm21_init_msg,
	.get_msg_len = rcm21_get_msg_len,
	.sign_msg = rcm21_sign_msg,
};

static uint32_t rcm_get_msg_buf_len(const struct rcm *rcm,
				    uint32_t payload_len)
{
	return rcm->message_size + payload_len +
		rcm_get_pad_len(rcm, payload_len);
}

static uint8_t *rcm_get_msg_payload(const struct rcm *rcm, uint8_t *buf)
{
	return buf + rcm->message_size;
}

int rcm_create_msg(const struct rcm *rcm, uint32_t opcode, const void *args,
		   uint32_t args_len, const void *payload,
		   uint32_t payload_len, void **buf)
{
	uint8_t *msg = NULL, *msg_payload;
	uint32_t msg_len;
	int ret = 0;

	// create message buffer
	msg_len = rcm_get_msg_buf_len(rcm, payload_len);
	msg = malloc(msg_len);
	if (!msg) {
		ret = -ENOMEM;
		goto done;
	}

	// initialize message
	rcm->init_msg(rcm, msg, msg_len, opcode, args, args_len, payload_len);

	// fill message payload
	msg_payload = rcm_get_msg_payload(rcm, msg);
	if (payload_len)
		memcpy(msg_payload, payload, payload_len);

	// sign message
	rcm->sign_msg(rcm, msg);

done:
	if (ret) {
		free(msg);
		msg = NULL;
	}

	*buf = msg;

	return ret;
}

uint32_t rcm_get_msg_len(const struct rcm *rcm, const void *msg)
{
	return rcm->get_msg_len(rcm, msg);
}
