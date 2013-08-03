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

// tegra20 miniloader
#include "miniloader/tegra20-miniloader.h"

// tegra30 miniloader
#include "miniloader/tegra30-miniloader.h"

// tegra114 miniloader
#include "miniloader/tegra114-miniloader.h"

// tegra124 miniloader
#include "miniloader/tegra124-miniloader.h"

// tegra132 miniloader
#include "miniloader/tegra132-miniloader.h"

// tegra132 preboot mts
#include "miniloader/tegra132-preboot-mts.h"

// tegra132 mts
#include "miniloader/tegra132-mts.h"

static int initialize_rcm(uint16_t devid, usb_device_t *usb);
static int initialize_miniloader(uint16_t devid, usb_device_t *usb, char *mlfile, uint32_t mlentry);
static int initialize_preboot(usb_device_t *usb, char *mlfile, uint32_t mlentry);
static int wait_status(nv3p_handle_t h3p);
static int send_file(nv3p_handle_t h3p, const char *filename);
static int send_buf(nv3p_handle_t h3p, uint8_t *buf, uint64_t total);
static int download_binary(uint32_t cmd, usb_device_t *usb,
			   uint8_t *miniloader, uint32_t size, uint32_t entry);
static void dump_platform_info(nv3p_platform_info_t *info);
static int download_bct(nv3p_handle_t h3p, char *filename);
static int download_bootloader(nv3p_handle_t h3p, char *filename,
			       uint32_t entry, uint32_t loadaddr);
static int download_mts(nv3p_handle_t h3p, char *filename,
			uint32_t loadaddr, uint16_t devid);
static int read_bct(nv3p_handle_t h3p, char *filename);
static int send_odmdata(nv3p_handle_t h3p, uint32_t odmdata);

enum cmdline_opts {
	OPT_BCT,
	OPT_BOOTLOADER,
	OPT_LOADADDR,
	OPT_ENTRYADDR,
	OPT_HELP,
	OPT_VERSION,
	OPT_MINILOADER,
	OPT_MINIENTRY,
	OPT_PREBOOT,
	OPT_PREBOOTENTRY,
	OPT_MTS,
	OPT_MTSENTRY,
	OPT_ODMDATA,
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
	fprintf(stderr, "\t\tRead the preboot mts ucode from file instead of using built-in\n");
	fprintf(stderr, "\t\tpreboot mts\n");
	fprintf(stderr, "\t--preboot_entry=<pbentry>\n");
	fprintf(stderr, "\t\tSpecify the entry point for the preboot mts ucode\n");
	fprintf(stderr, "\t--mts=mtsfile\n");
	fprintf(stderr, "\t\tRead the mts ucode from file instead of using built-in\n");
	fprintf(stderr, "\t\tmts\n");
	fprintf(stderr, "\t--mts_entry=<mtsentry>\n");
	fprintf(stderr, "\t\tSpecify the entry point for the mts ucode\n");
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
	uint16_t devid;
	int do_read = 0;
	char *mlfile = NULL;
	uint32_t mlentry = 0;
	char *pbfile = NULL;
	uint32_t pbentry = 0;
	char *mtsfile = NULL;
	uint32_t mtsentry = 0;
	uint32_t odmdata = 0;

	static struct option long_options[] = {
		[OPT_BCT]        = {"bct", 1, 0, 0},
		[OPT_BOOTLOADER] = {"bootloader", 1, 0, 0},
		[OPT_LOADADDR]   = {"loadaddr", 1, 0, 0},
		[OPT_ENTRYADDR]  = {"entryaddr", 1, 0, 0},
		[OPT_HELP]       = {"help", 0, 0, 0},
		[OPT_VERSION]    = {"version", 0, 0, 0},
		[OPT_MINILOADER] = {"miniloader", 1, 0, 0},
		[OPT_MINIENTRY]  = {"miniloader_entry", 1, 0, 0},
		[OPT_PREBOOT]    = {"preboot", 1, 0, 0},
		[OPT_PREBOOTENTRY] = {"preboot_entry", 1, 0, 0},
		[OPT_MTS]        = {"mts", 1, 0, 0},
		[OPT_MTSENTRY]   = {"mts_entry", 1, 0, 0},
		[OPT_ODMDATA]    = {"odmdata", 1, 0, 0},
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
			case OPT_ODMDATA:
				odmdata = strtoul(optarg, NULL, 0);
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
		ret2 = initialize_rcm(devid, usb);
		if (ret2)
			error(1, errno, "error initializing RCM protocol");

		// download the mts ucode
		if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA132) {
			ret2 = initialize_preboot(usb, pbfile, pbentry);
			if (ret2)
				error(1, errno, "error initializing preboot mts");
		}

		// download the miniloader to start nv3p
		ret2 = initialize_miniloader(devid, usb, mlfile, mlentry);
		if (ret2)
			error(1, errno, "error initializing miniloader");

		// device may have re-enumerated, so reopen USB
		usb_close(usb);
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

	if (odmdata) {
		printf("sending odm data (0x%x) to target...\n", odmdata);
		ret = send_odmdata(h3p, odmdata);
		if (ret)
			error(1, ret, "error sending ODM data");
		printf("odm data sent successfully\n");
	}

	// get platform info and dump it
	info.skip_auto_detect = 1;
	ret = nv3p_cmd_send(h3p, NV3P_CMD_GET_PLATFORM_INFO, (uint8_t *)&info);
	if (ret)
		error(1, errno, "retreiving platform info");
	ret = wait_status(h3p);
	if (ret)
		error(1, errno, "wait status after platform info");
	dump_platform_info(&info);

	if (info.op_mode != RCM_OP_MODE_DEVEL &&
	    info.op_mode != RCM_OP_MODE_ODM_OPEN &&
	    info.op_mode != RCM_OP_MODE_PRE_PRODUCTION)
		error(1, ENODEV, "device is not in developer, open, "
		      "or pre-production mode, cannot flash");

	// download the BCT
	ret = download_bct(h3p, bctfile);
	if (ret) {
		error(1, ret, "error downloading bct: %s", bctfile);
	}

	// download mts
	ret = download_mts(h3p, mtsfile, mtsentry, devid);
	if (ret)
		error(1, ret, "error downloading mts: %s", mtsfile);


	// download the bootloader
	ret = download_bootloader(h3p, blfile, entryaddr, loadaddr);
	if (ret)
		error(1, ret, "error downloading bootloader: %s", blfile);


	nv3p_close(h3p);
	usb_close(usb);

	return 0;
}

static int initialize_rcm(uint16_t devid, usb_device_t *usb)
{
	int ret;
	uint8_t *msg_buff;
	int msg_len;
	uint32_t status;
	int actual_len;

	// initialize RCM
	if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA20 ||
	    (devid & 0xff) == USB_DEVID_NVIDIA_TEGRA30) {
		dprintf("initializing RCM version 1\n");
		ret = rcm_init(RCM_VERSION_1);
	} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA114) {
		dprintf("initializing RCM version 35\n");
		ret = rcm_init(RCM_VERSION_35);
	} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA124 ||
		   (devid & 0xff) == USB_DEVID_NVIDIA_TEGRA132) {
		dprintf("initializing RCM version 40\n");
		ret = rcm_init(RCM_VERSION_40);
	} else {
		fprintf(stderr, "unknown tegra device: 0x%x\n", devid);
		return errno;
	}
	if (ret) {
		fprintf(stderr, "RCM initialize failed\n");
		return ret;
	}

	// create query version message
	rcm_create_msg(RCM_CMD_QUERY_RCM_VERSION, NULL, 0, NULL, 0, &msg_buff);

	// write query version message to device
	msg_len = rcm_get_msg_len(msg_buff);
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

static int initialize_preboot(usb_device_t *usb, char *pbfile, uint32_t pbentry)
{
	int fd;
	struct stat sb;
	int ret;
	uint8_t *preboot;
	uint32_t pb_size;
	uint32_t pb_entry;

	// use prebuilt preboot mts if not loading from a file
	if (pbfile) {
		fd = open(pbfile, O_RDONLY, 0);
		if (fd < 0) {
			dprintf("error opening %s for reading\n", pbfile);
			return errno;
		}
		ret = fstat(fd, &sb);
		if (ret) {
			dprintf("error on fstat of %s\n", pbfile);
			return ret;
		}
		pb_size = sb.st_size;
		preboot = (uint8_t *)malloc(pb_size);
		if (!preboot) {
			dprintf("error allocating %d bytes for preboot mts\n", pb_size);
			return errno;
		}
		if (read(fd, preboot, pb_size) != pb_size) {
			dprintf("error reading from preboot mts file");
			return errno;
		}
		pb_entry = pbentry;
	} else {
		preboot = preboot_mts_tegra132;
		pb_size = sizeof(preboot_mts_tegra132);
		pb_entry = TEGRA132_PREBOOT_MTS_ENTRY;
	}
	printf("downloading preboot mts to target at address 0x%x (%d bytes)...\n",
	       pb_entry, pb_size);
	ret = download_binary(RCM_CMD_DL_MTS, usb, preboot,
			      pb_size, pb_entry);
	if (ret) {
		fprintf(stderr, "Error downloading preboot mts\n");
		return ret;
	}
	printf("preboot mts downloaded successfully\n");

	return 0;
}

static int initialize_miniloader(uint16_t devid, usb_device_t *usb, char *mlfile, uint32_t mlentry)
{
	int fd;
	struct stat sb;
	int ret;
	uint8_t *miniloader;
	uint32_t miniloader_size;
	uint32_t miniloader_entry;

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
		miniloader_size = sb.st_size;
		miniloader = (uint8_t *)malloc(miniloader_size);
		if (!miniloader) {
			dprintf("error allocating %d bytes for miniloader\n", miniloader_size);
			return errno;
		}
		if (read(fd, miniloader, miniloader_size) != miniloader_size) {
			dprintf("error reading from miniloader file");
			return errno;
		}
		miniloader_entry = mlentry;
	} else {
		if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA20) {
			miniloader = miniloader_tegra20;
			miniloader_size = sizeof(miniloader_tegra20);
			miniloader_entry = TEGRA20_MINILOADER_ENTRY;
		} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA30) {
			miniloader = miniloader_tegra30;
			miniloader_size = sizeof(miniloader_tegra30);
			miniloader_entry = TEGRA30_MINILOADER_ENTRY;
		} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA114) {
			miniloader = miniloader_tegra114;
			miniloader_size = sizeof(miniloader_tegra114);
			miniloader_entry = TEGRA114_MINILOADER_ENTRY;
		} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA124) {
			miniloader = miniloader_tegra124;
			miniloader_size = sizeof(miniloader_tegra124);
			miniloader_entry = TEGRA124_MINILOADER_ENTRY;
		} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA132) {
			miniloader = miniloader_tegra132;
			miniloader_size = sizeof(miniloader_tegra132);
			miniloader_entry = TEGRA132_MINILOADER_ENTRY;
		} else {
			fprintf(stderr, "unknown tegra device: 0x%x\n", devid);
			return ENODEV;
		}
	}
	printf("downloading miniloader to target at address 0x%x (%d bytes)...\n",
		miniloader_entry, miniloader_size);
	ret = download_binary(RCM_CMD_DL_MINILOADER, usb, miniloader,
			      miniloader_size, miniloader_entry);
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
* send_buf: send data present in buffer to nv3p server
*/
static int send_buf(nv3p_handle_t h3p, uint8_t *buf, uint64_t total)
{
	int ret = 0;
	uint32_t size;
	uint64_t count;
	char *spinner = "-\\|/";
	int spin_idx = 0;

#define NVFLASH_DOWNLOAD_CHUNK (1024 * 64)

	printf("sending data:\n");

	count = 0;
	while(count != total) {
		size = (uint32_t)MIN(total - count, NVFLASH_DOWNLOAD_CHUNK);

		ret = nv3p_data_send(h3p, buf, size);
		if (ret)
			goto fail;

		count += size;
		buf += size;

		printf("\r%c %" PRIu64 "/%" PRIu64" bytes sent", spinner[spin_idx],
		       count, total);
		spin_idx = (spin_idx + 1) % 4;
	}
	printf("\ndata sent successfully\n");

#undef NVFLASH_DOWNLOAD_CHUNK

fail:
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


static int download_binary(uint32_t cmd, usb_device_t *usb,
			   uint8_t *binary, uint32_t size, uint32_t entry)
{
	uint8_t *msg_buff;
	int ret;
	uint32_t status;
	int actual_len;

	// create download message
	rcm_create_msg(cmd,
		       (uint8_t *)&entry, sizeof(entry), binary, size,
		       &msg_buff);
	ret = usb_write(usb, msg_buff, rcm_get_msg_len(msg_buff));
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

	bct_data = (uint8_t *)malloc(bct_info.length);
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

static int send_odmdata(nv3p_handle_t h3p, uint32_t odmdata)
{
	int ret;
	nv3p_cmd_send_odmdata_t odm_info;

	odm_info.odmdata = odmdata;
	ret = nv3p_cmd_send(h3p, NV3P_CMD_SEND_ODMDATA, (uint8_t *)&odm_info);
	if (ret) {
		dprintf("error sending send odmdata command\n");
		return ret;
	}
	ret = wait_status(h3p);
	if (ret) {
		dprintf("error waiting for status after get bct\n");
		return ret;
	}

	return 0;
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

	return 0;
}

static int download_mts(nv3p_handle_t h3p, char *filename,
			uint32_t loadaddr, uint16_t devid)
{
	int ret;
	nv3p_cmd_dl_mts_t arg;
	int fd;
	struct stat sb;
	uint8_t *buf;

	if (filename) {
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
		close(fd);

		arg.length = sb.st_size;
		arg.address = loadaddr;
	} else {
		if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA132) {
			arg.length = sizeof(mts_tegra132);
			arg.address = TEGRA132_MTS_ENTRY;
			buf = mts_tegra132;
		} else {
			fprintf(stderr, "unknown tegra device: 0x%x\n", devid);
			return ENODEV;
		}

	}

	ret = nv3p_cmd_send(h3p, NV3P_CMD_DL_MTS, (uint8_t *)&arg);
	if (ret) {
		dprintf("error sending 3p mts download command\n");
		return ret;
	}

	if (filename) {
		// send the mts file
		ret = send_file(h3p, filename);
		if (ret) {
			dprintf("error downloading mts\n");
			return ret;
		}
	} else {
		ret = send_buf(h3p, buf, arg.length);
		if (ret) {
			dprintf("error downloading mts\n");
			return ret;
		}
	}

	ret = wait_status(h3p);
	if (ret) {
		dprintf("error waiting for status on mts dl\n");
		return ret;
	}

	return 0;
}
