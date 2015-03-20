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

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <error.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/param.h>
#include <unistd.h>
#include <getopt.h>
#include <inttypes.h>

#include "usb.h"
#include "nv3p.h"
#include "nv3p_status.h"
#include "aes-cmac.h"
#include "rcm.h"
#include "debug.h"
#include "config.h"
#include "soc.h"

static int initialize_rcm(const struct soc *soc, usb_device_t *usb);
static int initialize_miniloader(const struct soc *soc, usb_device_t *usb,
				 const char *filename, uint32_t entry);
static int initialize_preboot(const struct soc *soc, usb_device_t *usb,
			      const char *filename, uint32_t entry);
static int wait_status(nv3p_handle_t h3p);
static int send_file(nv3p_handle_t h3p, const char *filename);
static int download_binary(const struct soc *soc, uint32_t cmd,
			   usb_device_t *usb, struct binary *binary);
static void dump_platform_info(nv3p_platform_info_t *info);
static int download_bct(nv3p_handle_t h3p, char *filename);
static int download_bootloader(nv3p_handle_t h3p, char *filename,
			       uint32_t entry, uint32_t loadaddr);
static int download_mts(nv3p_handle_t h3p, char *filename, uint32_t loadaddr);
static int read_bct(nv3p_handle_t h3p, char *filename);

enum cmdline_opts {
	OPT_BCT,
	OPT_BOOTLOADER,
	OPT_LOADADDR,
	OPT_ENTRYADDR,
	OPT_HELP,
	OPT_VERSION,
	OPT_MINILOADER,
	OPT_MINIENTRY,
	OPT_MINIENTRY1,
	OPT_PREBOOT,
	OPT_PREBOOTENTRY,
	OPT_MTS,
	OPT_MTSENTRY,
	OPT_END,
};

static void print_version(char *progname)
{
	printf("%s %s\n", PACKAGE_NAME, PACKAGE_VERSION);
}

static void usage(char *progname)
{
	fprintf(stderr, "usage: %s [options] --bct=bctfile --bootloader=blfile --loadaddr=<loadaddr>\n", progname);
	fprintf(stderr, "       %s [options] --bct=bctfile readbct\n", progname);
	fprintf(stderr, "\n");
	fprintf(stderr, "Commands:\n");
	fprintf(stderr, "\treadbct\n");
	fprintf(stderr, "\t\tRead the BCT from the target device and write to bctfile\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "\tDefault operation if no command is specified will download BCT and\n");
	fprintf(stderr, "\tbootloader to device and start execution of bootloader\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Options:\n");
	fprintf(stderr, "\t--entryaddr=<entryaddr>\n");
	fprintf(stderr, "\t\tSpecify the entry point for the bootloader, if this option is\n");
	fprintf(stderr, "\t\tnot provided, it is assumed to be loadaddr\n");
	fprintf(stderr, "\t--help\n");
	fprintf(stderr, "\t\tPrint this help information\n");
	fprintf(stderr, "\t--version\n");
	fprintf(stderr, "\t\tPrint version information and exit\n");
	fprintf(stderr, "\t--miniloader=mlfile\n");
	fprintf(stderr, "\t\tRead the miniloader from file instead of using built-in\n");
	fprintf(stderr, "\t\tminiloader\n");
	fprintf(stderr, "\t--miniloader_entry=<mlentry>\n");
	fprintf(stderr, "\t\tSpecify the entry point for the miniloader\n");
	fprintf(stderr, "\t--preboot=pbfile\n");
	fprintf(stderr, "\t\tRead the preboot ucode from given file\n");
	fprintf(stderr, "\t--preboot-entry=<pbentry>\n");
	fprintf(stderr, "\t\tSpecify the entry point for the preboot ucode\n");
	fprintf(stderr, "\t--mts=mtsfile\n");
	fprintf(stderr, "\t\tRead the cpu ucode from given file\n");
	fprintf(stderr, "\t--mts-entry=<mtsentry>\n");
	fprintf(stderr, "\t\tSpecify the entry point for the cpu ucode\n");
	fprintf(stderr, "\n");
}

int main(int argc, char **argv)
{
	// discover devices
	uint64_t uid[2];
	int actual_len;
	usb_device_t *usb;
	nv3p_platform_info_t info;
	nv3p_handle_t h3p;
	int ret, ret2;
	int c;
	int option_index = 0;
	char *bctfile = NULL;
	char *blfile = NULL;
	uint32_t loadaddr = 0;
	uint32_t entryaddr = 0;
	const struct soc *soc;
	uint16_t devid;
	int do_read = 0;
	char *mlfile = NULL;
	uint32_t mlentry = 0;
	char *pbfile = NULL;
	uint32_t pbentry = 0;
	char *mtsfile = NULL;
	uint32_t mtsentry = 0;

	static struct option long_options[] = {
		[OPT_BCT]        = {"bct", 1, 0, 0},
		[OPT_BOOTLOADER] = {"bootloader", 1, 0, 0},
		[OPT_LOADADDR]   = {"loadaddr", 1, 0, 0},
		[OPT_ENTRYADDR]  = {"entryaddr", 1, 0, 0},
		[OPT_HELP]       = {"help", 0, 0, 0},
		[OPT_VERSION]    = {"version", 0, 0, 0},
		[OPT_MINILOADER] = {"miniloader", 1, 0, 0},
		[OPT_MINIENTRY]  = {"miniloader_entry", 1, 0, 0},
		[OPT_MINIENTRY1] = {"miniloader-entry", 1, 0, 0},
		[OPT_PREBOOT]    = {"preboot", 1, 0, 0},
		[OPT_PREBOOTENTRY] = {"preboot-entry", 1, 0, 0},
		[OPT_MTS]        = {"mts", 1, 0, 0},
		[OPT_MTSENTRY]   = {"mts-entry", 1, 0, 0},
		[OPT_END]        = {0, 0, 0, 0}
	};

	// parse command line args
	while (1) {
		c = getopt_long(argc, argv, "h0",
				long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 0:
			switch (option_index) {
			case OPT_BCT:
				bctfile = optarg;
				break;
			case OPT_BOOTLOADER:
				blfile = optarg;
				break;
			case OPT_LOADADDR:
				loadaddr = strtoul(optarg, NULL, 0);
				break;
			case OPT_ENTRYADDR:
				entryaddr = strtoul(optarg, NULL, 0);
				break;
			case OPT_VERSION:
				print_version(argv[0]);
				exit(0);
				break;
			case OPT_MINILOADER:
				mlfile = optarg;
				break;
			case OPT_MINIENTRY:
			case OPT_MINIENTRY1:
				mlentry = strtoul(optarg, NULL, 0);
				break;
			case OPT_PREBOOT:
				pbfile = optarg;
				break;
			case OPT_PREBOOTENTRY:
				pbentry = strtoul(optarg, NULL, 0);
				break;
			case OPT_MTS:
				mtsfile = optarg;
				break;
			case OPT_MTSENTRY:
				mtsentry = strtoul(optarg, NULL, 0);
				break;
			case OPT_HELP:
			default:
				usage(argv[0]);
				exit(EXIT_FAILURE);
			}
			break;
		case 'h':
			usage(argv[0]);
			exit(EXIT_FAILURE);
			break;
		}
	}

	while (optind < argc) {
		if (!(strcmp(argv[optind], "readbct")))
			do_read = 1;
		else {
			fprintf(stderr, "%s: Unknown command line argument: %s\n",
				argv[0], argv[optind]);
			usage(argv[0]);
			exit(EXIT_FAILURE);
		}
		optind++;
	}

	if (bctfile == NULL) {
		fprintf(stderr, "BCT file must be specified\n");
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}
	printf("bct file: %s\n", bctfile);

	if (!do_read) {
		if (blfile == NULL) {
			fprintf(stderr, "bootloader file must be specified\n");
			usage(argv[0]);
			exit(EXIT_FAILURE);
		}
		if (loadaddr == 0) {
			fprintf(stderr, "loadaddr must be specified\n");
			usage(argv[0]);
			exit(EXIT_FAILURE);
		}
		if (entryaddr == 0) {
			entryaddr = loadaddr;
		}
		printf("booloader file: %s\n", blfile);
		printf("load addr 0x%x\n", loadaddr);
		printf("entry addr 0x%x\n", entryaddr);
	}

	usb = usb_open(USB_VENID_NVIDIA, &devid);
	if (!usb)
		error(1, errno, "could not open USB device");
	printf("device id: 0x%x\n", devid);

	soc = soc_detect(devid);
	if (!soc) {
		fprintf(stderr, "unsupported SoC: %04x\n", devid);
		return 1;
	}

	printf("NVIDIA %s detected\n", soc->name);

	ret = usb_read(usb, (uint8_t *)uid, sizeof(uid), &actual_len);
	if (!ret) {
		if (actual_len == 8)
			printf("uid:  0x%016" PRIx64 "\n", uid[0]);
		else if (actual_len == 16)
			printf("uid:  0x%016" PRIx64 "%016" PRIx64 "\n",
			       uid[1], uid[0]);
		else
			error(1, errno, "USB read truncated");

		// initialize rcm
		ret2 = initialize_rcm(soc, usb);
		if (ret2)
			error(1, errno, "error initializing RCM protocol");

		// download the mts_preboot ucode
		if (soc->needs_mts) {
			ret2 = initialize_preboot(soc, usb, pbfile, pbentry);
			if (ret2)
				error(1, errno, "error initializing preboot mts");
		}

		// download the miniloader to start nv3p
		ret2 = initialize_miniloader(soc, usb, mlfile, mlentry);
		if (ret2)
			error(1, errno, "error initializing miniloader");

		// device may have re-enumerated, so reopen USB
		usb_close(usb);
		sleep(1);
		usb = usb_open(USB_VENID_NVIDIA, &devid);
		if (!usb)
			error(1, errno, "could not open USB device");
	}

	// now that miniloader is up, start nv3p protocol
	ret = nv3p_open(&h3p, usb);
	if (ret)
		error(1, errno, "3p open failed");

	// read the BCT
	if (do_read) {
		printf("reading BCT from system, writing to %s...", bctfile);
		ret = read_bct(h3p, bctfile);
		if (ret)
			error(1, errno, "error reading bct");
		printf("done!\n");
		exit(0);
	}

	// get platform info and dump it
	ret = nv3p_cmd_send(h3p, NV3P_CMD_GET_PLATFORM_INFO, (uint8_t *)&info);
	if (ret)
		error(1, errno, "retreiving platform info");
	ret = wait_status(h3p);
	if (ret)
		error(1, errno, "wait status after platform info");
	dump_platform_info(&info);

	if (info.op_mode != RCM_OP_MODE_DEVEL &&
	    info.op_mode != RCM_OP_MODE_ODM_OPEN &&
	    info.op_mode != RCM_OP_MODE_ODM_SECURE &&
	    info.op_mode != RCM_OP_MODE_PRE_PRODUCTION)
		error(1, ENODEV, "device is not in developer, open, secure, "
		      "or pre-production mode, cannot flash");

	// download the BCT
	ret = download_bct(h3p, bctfile);
	if (ret) {
		error(1, ret, "error downloading bct: %s", bctfile);
	}

	// download mts
	if (soc->needs_mts) {
		ret = download_mts(h3p, mtsfile, mtsentry);
		if (ret)
			error(1, ret, "error downloading mts: %s", mtsfile);
	}

	// download the bootloader
	ret = download_bootloader(h3p, blfile, entryaddr, loadaddr);
	if (ret)
		error(1, ret, "error downloading bootloader: %s", blfile);

	nv3p_close(h3p);
	usb_close(usb);

	return 0;
}

static int initialize_rcm(const struct soc *soc, usb_device_t *usb)
{
	int ret, msg_len, actual_len;
	uint32_t status;
	void *msg_buff;

	// create query version message
	rcm_create_msg(soc->rcm, RCM_CMD_QUERY_RCM_VERSION, NULL, 0, NULL, 0,
		       &msg_buff);

	// write query version message to device
	msg_len = rcm_get_msg_len(soc->rcm, msg_buff);
	if (msg_len == 0) {
		fprintf(stderr, "write RCM query version: unknown message length\n");
		return EINVAL;
	}
	ret = usb_write(usb, msg_buff, msg_len);
	if (ret) {
		fprintf(stderr, "write RCM query version: USB transfer failure\n");
		return ret;
	}
	free(msg_buff);
	msg_buff = NULL;

	// read response
	ret = usb_read(usb, (uint8_t *)&status, sizeof(status), &actual_len);
	if (ret) {
		fprintf(stderr, "read RCM query version: USB transfer failure\n");
		return ret;
	}
	if (actual_len < sizeof(status)) {
		fprintf(stderr, "read RCM query version: USB read truncated\n");
		return EIO;
	}
	printf("RCM version: %d.%d\n", RCM_VERSION_MAJOR(status),
	       RCM_VERSION_MINOR(status));

	return 0;
}

static int initialize_preboot(const struct soc *soc, usb_device_t *usb,
			      const char *filename, uint32_t entry)
{
	struct binary preboot;
	struct stat sb;
	int fd, ret;
	void *data;

	if (!filename)
		return -EINVAL;

	fd = open(filename, O_RDONLY, 0);
	if (fd < 0) {
		dprintf("error opening %s for reading\n", filename);
		return -errno;
	}

	ret = fstat(fd, &sb);
	if (ret) {
		dprintf("error on fstat of %s\n", filename);
		return -errno;
	}

	data = malloc(sb.st_size);
	if (!data) {
		dprintf("error allocating %zu bytes for preboot mts\n",
			sb.st_size);
		return -errno;
	}

	if (read(fd, data, sb.st_size) != sb.st_size) {
		dprintf("error reading from preboot mts file");
		return -errno;
	}

	preboot.size = sb.st_size;
	preboot.entry = entry;
	preboot.data = data;

	printf("downloading preboot mts to target at address 0x%x (%zu bytes)...\n",
	       preboot.entry, preboot.size);

	ret = download_binary(soc, RCM_CMD_DL_MTS, usb, &preboot);
	if (ret) {
		fprintf(stderr, "Error downloading preboot mts\n");
		return ret;
	}

	printf("preboot mts downloaded successfully\n");

	return 0;
}

static int initialize_miniloader(const struct soc *soc, usb_device_t *usb,
				 const char *mlfile, uint32_t mlentry)
{
	struct binary miniloader;
	void *data = NULL;
	struct stat sb;
	int fd, ret;

	// use prebuilt miniloader if not loading from a file
	if (mlfile) {
		fd = open(mlfile, O_RDONLY, 0);
		if (fd < 0) {
			dprintf("error opening %s for reading\n", mlfile);
			return errno;
		}
		ret = fstat(fd, &sb);
		if (ret) {
			dprintf("error on fstat of %s\n", mlfile);
			return ret;
		}
		miniloader.size = sb.st_size;
		data = malloc(miniloader.size);
		if (!data) {
			dprintf("error allocating %zu bytes for miniloader\n", miniloader.size);
			return errno;
		}
		if (read(fd, data, miniloader.size) != miniloader.size) {
			dprintf("error reading from miniloader file");
			return errno;
		}
		miniloader.entry = mlentry;
		miniloader.data = data;
	} else {
		/* check for a built-in miniloader */
		if (!soc->miniloader.data) {
			fprintf(stderr, "no built-in miniloader for %s\n",
				soc->name);
			return -ENOENT;
		}

		memcpy(&miniloader, &soc->miniloader, sizeof(miniloader));
	}
	printf("downloading miniloader to target at address 0x%x (%zu bytes)...\n",
		miniloader.entry, miniloader.size);
	ret = download_binary(soc, RCM_CMD_DL_MINILOADER, usb, &miniloader);
	if (ret) {
		fprintf(stderr, "Error downloading miniloader\n");
		return ret;
	}
	printf("miniloader downloaded successfully\n");

	return 0;
}

static int wait_status(nv3p_handle_t h3p)
{
	int ret;
	uint32_t cmd;
	nv3p_cmd_status_t *status_arg = 0;

	ret = nv3p_cmd_recv(h3p, &cmd, (void **)&status_arg);
	if (ret) {
		dprintf("nv3p_cmd_recv failed\n");
		goto fail;
	}
	if (cmd != NV3P_CMD_STATUS) {
		dprintf("expecting NV3P_CMD_STATUS(0x%x) got 0x%x\n",
			NV3P_CMD_STATUS, cmd);
		ret = EIO;
		goto fail;
	}
	if (status_arg->code != nv3p_status_ok) {
		dprintf("expecting nv3p_status_ok(0x%x) got 0x%x\n",
			nv3p_status_ok, status_arg->code);
		ret = EIO;
		goto fail;
	}

	nv3p_ack(h3p);

	return 0;

fail:
	if(status_arg) {
		printf("bootloader status: (code: %d) message: %s flags: %d\n",
		       status_arg->code, status_arg->msg, status_arg->flags );
	}
	return ret;
}

/*
 * send_file: send data present in file "filename" to nv3p server.
 */
static int send_file(nv3p_handle_t h3p, const char *filename)
{
	int ret;
	uint8_t *buf = 0;
	uint32_t size;
	uint64_t total;
	uint32_t bytes;
	uint64_t count;
	char *spinner = "-\\|/";
	int spin_idx = 0;
	int fd = -1;
	struct stat sb;

#define NVFLASH_DOWNLOAD_CHUNK (1024 * 64)

	printf("sending file: %s\n", filename );

	fd = open(filename, O_RDONLY, 0);
	if (fd < 0) {
		ret = errno;
		goto fail;
	}

	if (fstat(fd, &sb) < 0) {
		ret = errno;
		goto fail;
	}

	total = sb.st_size;

	buf = malloc( NVFLASH_DOWNLOAD_CHUNK );
	if (!buf) {
		ret = ENOMEM;
		goto fail;
	}

	count = 0;
	while(count != total) {
		size = (uint32_t)MIN(total - count, NVFLASH_DOWNLOAD_CHUNK);

		bytes = read(fd, buf, size);
		if (bytes < 0) {
			ret = errno;
			goto fail;
		}

		ret = nv3p_data_send(h3p, buf, bytes);
		if (ret)
			goto fail;

		count += bytes;

		printf("\r%c %" PRIu64 "/%" PRIu64" bytes sent", spinner[spin_idx],
			count, total);
		spin_idx = (spin_idx + 1) % 4;
	}
	printf("\n%s sent successfully\n", filename);

#undef NVFLASH_DOWNLOAD_CHUNK

	close(fd);
	free(buf);
	return 0;

fail:
	if (fd != -1)
		close(fd);
	if (buf)
		free(buf);
	return ret;
}


static int download_binary(const struct soc *soc, uint32_t cmd,
			   usb_device_t *usb, struct binary *binary)
{
	int ret, actual_len;
	uint32_t status;
	void *msg_buff;

	// create download message
	rcm_create_msg(soc->rcm, cmd, &binary->entry, sizeof(binary->entry),
		       binary->data, binary->size, &msg_buff);
	ret = usb_write(usb, msg_buff, rcm_get_msg_len(soc->rcm, msg_buff));
	if (ret) {
		dprintf("error sending %x command to target\n", cmd);
		goto fail;
	}
	ret = usb_read(usb, (uint8_t *)&status, sizeof(status), &actual_len);
	if (ret) {
		dprintf("error reading status from target\n");
		goto fail;
	}
	if (actual_len < sizeof(status)) {
		dprintf("short read of status\n");
		ret = EIO;
		goto fail;
	}
	if (status != 0) {
		dprintf("got bad status: %x\n", status);
		ret = EIO;
		goto fail;
	}

	ret = 0;
fail:
	free(msg_buff);
	return ret;
}

static void dump_platform_info(nv3p_platform_info_t *info)
{
	printf("Chip UID:                0x%016" PRIx64 "%016" PRIx64 "\n",
	       info->uid[1], info->uid[0]);
	printf("Chip ID:                 0x%x\n", (uint32_t)info->chip_id.id);
	printf("Chip ID Major Version:   0x%x\n", (uint32_t)info->chip_id.major);
	printf("Chip ID Minor Version:   0x%x\n", (uint32_t)info->chip_id.minor);
	printf("Chip SKU:                0x%x", info->sku);

	// Convert chip sku to chip name as per chip id
	char *chip_name = NULL;
	if (info->chip_id.id == 0x20) {
		switch (info->sku) {
		case TEGRA2_CHIP_SKU_AP20:   chip_name = "ap20";  break;
		case TEGRA2_CHIP_SKU_T25SE:  chip_name = "t25se"; break;
		case TEGRA2_CHIP_SKU_AP25:   chip_name = "ap25";  break;
		case TEGRA2_CHIP_SKU_T25:    chip_name = "t25";   break;
		case TEGRA2_CHIP_SKU_AP25E:  chip_name = "ap25e"; break;
		case TEGRA2_CHIP_SKU_T25E:   chip_name = "t25e";  break;
		case TEGRA2_CHIP_SKU_T20:
		case TEGRA2_CHIP_SKU_T20_7:
		default: chip_name = "t20"; break;
		}
	} else if (info->chip_id.id == 0x30) {
		switch (info->sku) {
		case TEGRA3_CHIP_SKU_AP30:   chip_name = "ap30";  break;
		case TEGRA3_CHIP_SKU_T30S:   chip_name = "t30s";  break;
		case TEGRA3_CHIP_SKU_T33:    chip_name = "t33";   break;
		case TEGRA3_CHIP_SKU_T30:
		default: chip_name = "t30"; break;
		}
	} else if (info->chip_id.id == 0x35) {
		switch (info->sku) {
		case TEGRA114_CHIP_SKU_T114:
		case TEGRA114_CHIP_SKU_T114_1:
		default: chip_name = "t114"; break;
		}
	} else if (info->chip_id.id == 0x40) {
		switch (info->sku) {
		case TEGRA124_CHIP_SKU_T124:
		default: chip_name = "t124"; break;
		}
	} else if (info->chip_id.id == 0x13) {
		switch (info->sku) {
		case TEGRA132_CHIP_SKU_T132:
		default: chip_name = "t132"; break;
		}
	} else {
		chip_name = "unknown";
	}
	printf(" (%s)\n", chip_name);

	printf("Boot ROM Version:        0x%x\n", info->version);
	printf("Boot Device:             0x%x", info->boot_device);
	char *boot_dev = NULL;
	switch(info->boot_device) {
	case NV3P_DEV_TYPE_NAND:            boot_dev = "NAND"; break;
	case NV3P_DEV_TYPE_EMMC:            boot_dev = "EMMC"; break;
	case NV3P_DEV_TYPE_SPI:             boot_dev = "SPI"; break;
	case NV3P_DEV_TYPE_IDE:             boot_dev = "IDE"; break;
	case NV3P_DEV_TYPE_NAND_X16:        boot_dev = "NAND x16"; break;
	case NV3P_DEV_TYPE_SNOR:            boot_dev = "SNOR"; break;
	case NV3P_DEV_TYPE_MUX_ONE_NAND:    boot_dev = "Mux One NAND"; break;
	case NV3P_DEV_TYPE_MOBILE_LBA_NAND: boot_dev = "Mobile LBA NAND"; break;
	default:                            boot_dev = "unknown"; break;
	}
	printf(" (%s)\n", boot_dev);

	printf("Operating Mode:          0x%x", info->op_mode);
	char *op_mode = NULL;
	switch(info->op_mode) {
	case RCM_OP_MODE_PRE_PRODUCTION:    op_mode = "pre-production mode"; break;
	case RCM_OP_MODE_DEVEL:             op_mode = "developer mode"; break;
	case RCM_OP_MODE_ODM_OPEN:          op_mode = "odm open mode"; break;
	case RCM_OP_MODE_ODM_SECURE:        op_mode = "odm secure mode"; break;
	default:                            op_mode = "unknown"; break;
	}
	printf(" (%s)\n", op_mode);

	printf("Device Config Strap:     0x%x\n", info->dev_conf_strap);
	printf("Device Config Fuse:      0x%x\n", info->dev_conf_fuse);
	printf("SDRAM Config Strap:      0x%x\n", info->sdram_conf_strap);
}


static int download_bct(nv3p_handle_t h3p, char *filename)
{
	int ret;
	nv3p_cmd_dl_bct_t arg;
	struct stat sb;

	ret = stat(filename, &sb);
	if (ret)
		return errno;
	arg.length = sb.st_size;

	ret = nv3p_cmd_send(h3p, NV3P_CMD_DL_BCT, (uint8_t *)&arg);
	if (ret)
		return ret;
	ret = send_file(h3p, filename);
	if (ret)
		return ret;

	ret = wait_status(h3p);
	if (ret)
		error(1, errno, "wait status after sending bct");

	return 0;
}


static int read_bct(nv3p_handle_t h3p, char *filename)
{
	int ret;
	nv3p_bct_info_t bct_info;
	uint8_t *bct_data = NULL;
	int fd = -1;
	uint8_t *buf;

	ret = nv3p_cmd_send(h3p, NV3P_CMD_GET_BCT, (uint8_t *)&bct_info);
	if (ret) {
		dprintf("error sending get bct command\n");
		goto out;
	}
	ret = wait_status(h3p);
	if (ret) {
		dprintf("error waiting for status after get bct\n");
		goto out;
	}

	bct_data = malloc(bct_info.length);
	ret = nv3p_data_recv(h3p, bct_data, bct_info.length);
	if (ret) {
		dprintf("error retreiving bct data\n");
		goto out;
	}

	fd = open(filename, O_WRONLY | O_CREAT, 0644);
	if (fd < 0) {
		dprintf("error opening %s for reading\n", filename);
		ret = errno;
		goto out;
	}

	buf = bct_data;
	while(bct_info.length > 0) {
		ssize_t written = write(fd, buf, bct_info.length);
		if (written == -1) {
			if (errno == EINTR)
				continue;
			dprintf("error writing to %s\n", filename);
			ret = errno;
			goto out;
		}
		buf += written;
		bct_info.length -= written;
	}

	ret = 0;
out:
	if (bct_data)
		free(bct_data);
	if (fd >= 0)
		close(fd);
	return ret;
}

static int download_bootloader(nv3p_handle_t h3p, char *filename,
			       uint32_t entry, uint32_t loadaddr)
{
	int ret;
	nv3p_cmd_dl_bl_t arg;
	int fd;
	struct stat sb;

	fd = open(filename, O_RDONLY, 0);
	if (fd < 0) {
		dprintf("error opening %s for reading\n", filename);
		return errno;
	}

	ret = fstat(fd, &sb);
	if (ret) {
		dprintf("error on fstat of %s\n", filename);
		return ret;
	}
	arg.length = sb.st_size;
	close(fd);

	arg.entry = entry;
	arg.address = loadaddr;

	ret = nv3p_cmd_send(h3p, NV3P_CMD_DL_BL, (uint8_t *)&arg);
	if (ret) {
		dprintf("error sending 3p bootloader download command\n");
		return ret;
	}

	ret = wait_status(h3p);
	if (ret) {
		dprintf("error waiting for status on bootloader dl\n");
		return ret;
	}

	// send the bootloader file
	ret = send_file(h3p, filename);
	if (ret) {
		dprintf("error downloading bootloader\n");
		return ret;
	}

	ret = wait_status(h3p);
	if (ret) {
		dprintf("error waiting for status on bootloader dl\n");
		return ret;
	}

	return 0;
}

static int download_mts(nv3p_handle_t h3p, char *filename, uint32_t loadaddr)
{
	nv3p_cmd_dl_mts_t arg;
	struct stat sb;
	int ret, fd;

	if (!filename)
		return -EINVAL;

	fd = open(filename, O_RDONLY, 0);
	if (fd < 0) {
		dprintf("error opening %s for reading\n", filename);
		return errno;
	}

	ret = fstat(fd, &sb);
	if (ret) {
		dprintf("error on fstat of %s\n", filename);
		return errno;
	}

	close(fd);

	arg.length = sb.st_size;
	arg.address = loadaddr;

	ret = nv3p_cmd_send(h3p, NV3P_CMD_DL_MTS, (uint8_t *)&arg);
	if (ret) {
		dprintf("error sending 3p mts download command\n");
		return ret;
	}

	// send the mts file
	ret = send_file(h3p, filename);
	if (ret) {
		dprintf("error downloading mts\n");
		return ret;
	}

	ret = wait_status(h3p);
	if (ret) {
		dprintf("error waiting for status on mts dl\n");
		return ret;
	}

	return 0;
}
