/*
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
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

#include "soc.h"

#include "miniloader/tegra20-miniloader.h"
#include "miniloader/tegra30-miniloader.h"
#include "miniloader/tegra114-miniloader.h"
#include "miniloader/tegra124-miniloader.h"

static const struct soc tegra_socs[] = {
	{
		.name = "Tegra20",
		.chip_id = 0x20,
		.needs_mts = false,
		.rcm = &rcm1,
		.miniloader = {
			.data = miniloader_tegra20,
			.size = sizeof(miniloader_tegra20),
			.entry = TEGRA20_MINILOADER_ENTRY,
		},
	}, {
		.name = "Tegra30",
		.chip_id = 0x30,
		.needs_mts = false,
		.rcm = &rcm1,
		.miniloader = {
			.data = miniloader_tegra30,
			.size = sizeof(miniloader_tegra30),
			.entry = TEGRA30_MINILOADER_ENTRY,
		},
	}, {
		.name = "Tegra114",
		.chip_id = 0x35,
		.needs_mts = false,
		.rcm = &rcm35,
		.miniloader = {
			.data = miniloader_tegra114,
			.size = sizeof(miniloader_tegra114),
			.entry = TEGRA114_MINILOADER_ENTRY,
		},
	}, {
		.name = "Tegra124",
		.chip_id = 0x40,
		.needs_mts = false,
		.rcm = &rcm40,
		.miniloader = {
			.data = miniloader_tegra124,
			.size = sizeof(miniloader_tegra124),
			.entry = TEGRA124_MINILOADER_ENTRY,
		},
	}, {
		.name = "Tegra132",
		.chip_id = 0x13,
		.needs_mts = true,
		.rcm = &rcm40,
	}, {
		.name = "Tegra210",
		.chip_id = 0x21,
		.needs_mts = false,
		.rcm = &rcm21,
	},
};

const struct soc *soc_detect(uint16_t dev_id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tegra_socs); i++)
		if ((dev_id & 0xff) == tegra_socs[i].chip_id)
			return &tegra_socs[i];

	return NULL;
}
