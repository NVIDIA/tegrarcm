/*
 * Copyright (c) 2011-2016, NVIDIA CORPORATION
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
#include "rsa-pss.h"
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

static int initialize_rcm(uint16_t devid, usb_device_t *usb,
		const char *pkc_keyfile, const char *signed_msgs_file,
		bool download_signed_msgs);
static int initialize_miniloader(uint16_t devid, usb_device_t *usb,
		char *mlfile, uint32_t mlentry, const char *signed_msgs_file,
		bool download_signed_msgs);
static int wait_status(nv3p_handle_t h3p);
static int send_file(nv3p_handle_t h3p, const char *filename);
static int create_miniloader_rcm(uint8_t *miniloader, uint32_t size,
			uint32_t entry, const char *signed_msgs_file);
static int download_miniloader(usb_device_t *usb, uint8_t *miniloader,
				uint32_t size, uint32_t entry,
				bool download_signed_msgs);
static void dump_platform_info(nv3p_platform_info_t *info);
static int download_bct(nv3p_handle_t h3p, char *filename);
static int download_bootloader(nv3p_handle_t h3p, char *filename,
			uint32_t entry, uint32_t loadaddr,
			const char *pkc_keyfile, const char *signed_msgs_file);
static int read_bct(nv3p_handle_t h3p, char *filename);
static int sign_blob(const char *blob_filename, const char *pkc_keyfile,
				const char *signed_msgs_file);

static void set_platform_info(nv3p_platform_info_t *info);
static uint32_t get_op_mode(void);

static nv3p_platform_info_t *g_platform_info = NULL;
extern uint32_t usb_timeout;

enum cmdline_opts {
	OPT_BCT,
	OPT_BOOTLOADER,
	OPT_LOADADDR,
	OPT_ENTRYADDR,
	OPT_HELP,
	OPT_VERSION,
	OPT_MINILOADER,
	OPT_MINIENTRY,
	OPT_PKC,
#ifdef HAVE_USB_PORT_MATCH
	OPT_USBPORTPATH,
#endif
	OPT_SIGN_MSGS,
	OPT_SIGNED_MSGS_FILE,
	OPT_SOC,
	OPT_DOWNLOAD_SIGNED_MSGS,
	OPT_USB_TIMEOUT,
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
#ifdef HAVE_USB_PORT_MATCH
	fprintf(stderr, "\t--usb-port-path=<path>\n");
	fprintf(stderr, "\t\tSpecify the USB device to program, e.g. 3-10.4\n");
	fprintf(stderr, "\t\tSee `udevadm info /dev/bus/usb/003/042` DEVPATH\n");
	fprintf(stderr, "\t\tSee /sys/bus/usb/devices/* with matching busnum/devnum files\n");
#endif
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
	fprintf(stderr, "\t--pkc=<key.ber>\n");
	fprintf(stderr, "\t\tSpecify the key file for secured devices. The private key should be\n");
	fprintf(stderr, "\t\tin DER format\n");
	fprintf(stderr, "\t--gen-signed-msgs\n");
	fprintf(stderr, "\t\tGenerate signed messages for pkc secured devices\n");
	fprintf(stderr, "\t--signed-msgs-file=<msg_file_prefix>\n");
	fprintf(stderr, "\t\tSpecify message files prefix\n");
	fprintf(stderr, "\t--soc=<tegra soc #>\n");
	fprintf(stderr, "\t\tSpecify Tegra SoC chip model number, ie, 124.\n");
	fprintf(stderr, "\t--download-signed-msgs\n");
	fprintf(stderr, "\t\tDownload signed messages\n");
	fprintf(stderr, "\t--usb-timeout=<timeout_ms>\n");
	fprintf(stderr, "\t\tSpecify usb transfer timeout value in ms, 0 for unlimited timeout\n");
	fprintf(stderr, "\n");
}

#ifdef HAVE_USB_PORT_MATCH
static void parse_usb_port_path(char *argv0, char *path, uint8_t *match_bus,
	uint8_t *match_ports, int *match_ports_len)
{
	*match_bus = strtoul(path, &path, 10);
	if (*path != '-') {
		usage(argv0);
		exit(EXIT_FAILURE);
	}
	path++;

	*match_ports_len = 0;
	for (;;) {
		match_ports[*match_ports_len] = strtoul(path, &path, 10);
		(*match_ports_len)++;
		if (!*path)
			break;
		if (*match_ports_len >= PORT_MATCH_MAX_PORTS) {
			usage(argv0);
			exit(EXIT_FAILURE);
		}
		if (*path != '.') {
			usage(argv0);
			exit(EXIT_FAILURE);
		}
		path++;
	}
}
#endif

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
static bool is_soc_supported_for_signed_msgs(uint32_t soc, uint16_t *devid)
{
	struct {
		uint32_t soc;
		uint16_t usb_devid;
	} soc_to_devid[] = {
		{114,	USB_DEVID_NVIDIA_TEGRA114},
		{124,	USB_DEVID_NVIDIA_TEGRA124},
	};

	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(soc_to_devid); ++i) {
		if (soc_to_devid[i].soc == soc) {
			*devid = soc_to_devid[i].usb_devid;
			return true;
		}
	}

	return false;
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
	char *pkc_keyfile = NULL;
#ifdef HAVE_USB_PORT_MATCH
	bool match_port = false;
	uint8_t match_bus;
	uint8_t match_ports[PORT_MATCH_MAX_PORTS];
	int match_ports_len;
#endif
	bool sign_msgs = false;
	char *signed_msgs_file = NULL;
	uint32_t soc = 0;
	bool download_signed_msgs = false;

	static struct option long_options[] = {
		[OPT_BCT]        = {"bct", 1, 0, 0},
		[OPT_BOOTLOADER] = {"bootloader", 1, 0, 0},
		[OPT_LOADADDR]   = {"loadaddr", 1, 0, 0},
		[OPT_ENTRYADDR]  = {"entryaddr", 1, 0, 0},
		[OPT_HELP]       = {"help", 0, 0, 0},
		[OPT_VERSION]    = {"version", 0, 0, 0},
		[OPT_MINILOADER] = {"miniloader", 1, 0, 0},
		[OPT_MINIENTRY]  = {"miniloader_entry", 1, 0, 0},
		[OPT_PKC]        = {"pkc", 1, 0, 0},
#ifdef HAVE_USB_PORT_MATCH
		[OPT_USBPORTPATH]  = {"usb-port-path", 1, 0, 0},
#endif
		[OPT_SIGN_MSGS] = {"gen-signed-msgs", 0, 0, 0},
		[OPT_SIGNED_MSGS_FILE] = {"signed-msgs-file", 1, 0, 0},
		[OPT_SOC]        = {"soc", 1, 0, 0},
		[OPT_DOWNLOAD_SIGNED_MSGS] = {"download-signed-msgs", 0, 0, 0},
		[OPT_USB_TIMEOUT] = {"usb-timeout", 1, 0, 0},
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
			case OPT_PKC:
				pkc_keyfile = optarg;
				break;
#ifdef HAVE_USB_PORT_MATCH
			case OPT_USBPORTPATH:
				parse_usb_port_path(argv[0], optarg,
					&match_bus, match_ports,
					&match_ports_len);
				match_port = true;
				break;
#endif
			case OPT_SIGN_MSGS:
				sign_msgs = true;
				break;
			case OPT_SIGNED_MSGS_FILE:
				signed_msgs_file = optarg;
				break;
			case OPT_SOC:
				soc = strtoul(optarg, NULL, 0);
				break;
			case OPT_DOWNLOAD_SIGNED_MSGS:
				download_signed_msgs = true;
				break;
			case OPT_USB_TIMEOUT:
				usb_timeout = strtoul(optarg, NULL, 0);
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

	/* error check */
	if (sign_msgs == true) {
		/* must have pkc option */
		if (pkc_keyfile == NULL) {
			fprintf(stderr, "PKC key file must be specified\n");
			goto usage_exit;
		}

		/* must have signed_msgs_file option */
		if (signed_msgs_file == NULL) {
			fprintf(stderr, "signed msgs filename must be"
				" specified\n");
			goto usage_exit;
		}

		/* must have --soc option */
		if (soc == 0) {
			fprintf(stderr, "soc model number must be"
				" specified\n");
			goto usage_exit;
		}
	}

	/* error check */
	if (download_signed_msgs == true) {
		/* must have signed_msgs_file option */
		if (signed_msgs_file == NULL) {
			fprintf(stderr, "signed msgs filename must be"
					" specified\n");
			goto usage_exit;
		}

		/* can not have pkc keyfile */
		if (pkc_keyfile) {
			fprintf(stderr, "No pkc keyfile is needed\n");
			goto usage_exit;
		}

		/* can not load in unsigned ml */
		if (mlfile) {
			fprintf(stderr, "can not have option --miniloader\n");
			goto usage_exit;
		}
	}

	/*
	 * option --signed-msgs-file needs to work with option
	 *   either --gen-signed-msgs or --download-signed-msgs
	 */
	if (signed_msgs_file && (sign_msgs == false) &&
			(download_signed_msgs == false)) {
		fprintf(stderr, "missing option either --gen-signed-msgs or"
				" --download-signed-msgs\n");
		goto usage_exit;
	}

	/* specify bct file if no sign_msgs option */
	if ((bctfile == NULL) && (sign_msgs == false)) {
		fprintf(stderr, "BCT file must be specified\n");
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	if (bctfile)
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
		printf("bootloader file: %s\n", blfile);
		printf("load addr 0x%x\n", loadaddr);
		printf("entry addr 0x%x\n", entryaddr);
	}

	/* if sign_msgs, generate signed msgs, then exit */
	if (sign_msgs == true) {
		/*
		 * validate SoC value
		 *  currently only verified soc is T124
		 */
		if (!is_soc_supported_for_signed_msgs(soc, &devid)) {
			fprintf(stderr, "Unrecognized soc: %u\n", soc);
			goto usage_exit;
		}

		// init and create signed query version rcm
		if (initialize_rcm(devid, NULL, pkc_keyfile, signed_msgs_file,
					download_signed_msgs))
			error(1, errno, "error initializing RCM protocol");

		// create signed download miniloader rcm
		if (initialize_miniloader(devid, NULL, mlfile, mlentry,
				signed_msgs_file, download_signed_msgs))
			error(1, errno, "error initializing miniloader");

		// create bl signature
		sign_blob(blfile, pkc_keyfile, signed_msgs_file);

		exit(0);
	}

	/* start nv3p protocol */
	usb = usb_open(USB_VENID_NVIDIA, &devid
#ifdef HAVE_USB_PORT_MATCH
		, &match_port, &match_bus, match_ports, &match_ports_len
#endif
	);
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
		ret2 = initialize_rcm(devid, usb, pkc_keyfile,
					signed_msgs_file, download_signed_msgs);
		if (ret2)
			error(1, errno, "error initializing RCM protocol");

		// download the miniloader to start nv3p or create ml rcm file
		ret2 = initialize_miniloader(devid, usb, mlfile, mlentry,
					signed_msgs_file, download_signed_msgs);
		if (ret2)
			error(1, errno, "error initializing miniloader");

		// device may have re-enumerated, so reopen USB
		usb_close(usb);

		usb = usb_open(USB_VENID_NVIDIA, &devid
#ifdef HAVE_USB_PORT_MATCH
		, &match_port, &match_bus, match_ports, &match_ports_len
#endif
		);
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
	set_platform_info(&info);

	if (info.op_mode != RCM_OP_MODE_DEVEL &&
	    info.op_mode != RCM_OP_MODE_ODM_OPEN &&
	    info.op_mode != RCM_OP_MODE_ODM_SECURE &&
	    info.op_mode != RCM_OP_MODE_ODM_SECURE_PKC &&
	    info.op_mode != RCM_OP_MODE_PRE_PRODUCTION)
		error(1, ENODEV, "device is not in developer, open, secure, "
		      "or pre-production mode, cannot flash");

	// download the BCT
	ret = download_bct(h3p, bctfile);
	if (ret) {
		error(1, ret, "error downloading bct: %s", bctfile);
	}

	// download the bootloader
	ret = download_bootloader(h3p, blfile, entryaddr, loadaddr,
				pkc_keyfile, signed_msgs_file);
	if (ret)
		error(1, ret, "error downloading bootloader: %s", blfile);

	nv3p_close(h3p);
	usb_close(usb);
	return 0;

usage_exit:
	usage(argv[0]);
	exit(EXIT_FAILURE);
}

static int create_name_string(char *out, const char *in, const char *ext)
{
	if ((strlen(in) + strlen(ext) + 1) > PATH_MAX) {
		fprintf(stderr, "error: name length %zu bytes exceed "
				"limits for file %s\n",
			strlen(in) + strlen(ext) + 1 - PATH_MAX, in);
		return -1;
	}
	snprintf(out, PATH_MAX, "%s%s", in, ext);
	return 0;
}

static int save_to_file(const char *filename, const uint8_t *msg_buff,
			const uint32_t length)
{
	FILE *fp;

	printf("Create file %s...\n", filename);

	fp = fopen(filename, "wb");
	if (fp == NULL) {
		fprintf(stderr, "Error opening raw file %s.\n", filename);
		return -1;
	}

	fwrite(msg_buff, 1, length, fp);
	fclose(fp);

	return 0;
}

static int initialize_rcm(uint16_t devid, usb_device_t *usb,
			const char *pkc_keyfile, const char *signed_msgs_file,
			bool download_signed_msgs)
{
	int ret = 0;
	uint8_t *msg_buff;
	int msg_len;
	uint32_t status;
	int actual_len;
	char query_version_rcm_filename[PATH_MAX];

	// initialize RCM
	if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA20 ||
	    (devid & 0xff) == USB_DEVID_NVIDIA_TEGRA30) {
		dprintf("initializing RCM version 1\n");
		ret = rcm_init(RCM_VERSION_1, pkc_keyfile);
	} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA114) {
		dprintf("initializing RCM version 35\n");
		ret = rcm_init(RCM_VERSION_35, pkc_keyfile);
	} else if ((devid & 0xff) == USB_DEVID_NVIDIA_TEGRA124) {
		dprintf("initializing RCM version 40\n");
		ret = rcm_init(RCM_VERSION_40, pkc_keyfile);
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

	/* if signed_msgs_file is given */
	if (signed_msgs_file) {
		int fd;
		struct stat sb;

		ret = create_name_string(query_version_rcm_filename,
					signed_msgs_file, ".qry");
		if (ret)
			goto done;

		/* either save query version rcm blob */
		if (download_signed_msgs == false) {
			ret = save_to_file(query_version_rcm_filename, msg_buff,
					rcm_get_msg_len(msg_buff));
			goto done;
		}

		/* or download signed query version rcm blob */
		printf("download signed query version rcm from file %s\n",
			 query_version_rcm_filename);

		free(msg_buff); // release memory allocated by rcm_create_msg
		msg_buff = NULL;

		fd = open(query_version_rcm_filename, O_RDONLY, 0);
		if (fd < 0) {
			fprintf(stderr, "error opening %s\n",
				query_version_rcm_filename);
			return errno;
		}

		ret = fstat(fd, &sb);
		if (ret) {
			fprintf(stderr, "error on fstat of %s\n",
				query_version_rcm_filename);
			return ret;
		}
		msg_buff = (uint8_t *)malloc(sb.st_size);
		if (!msg_buff) {
			fprintf(stderr, "error allocating %zd bytes for query"
				" rcm\n", sb.st_size);
			return errno;
		}
		if (read(fd, msg_buff, sb.st_size) != sb.st_size) {
			fprintf(stderr,"error reading from %s\n",
				query_version_rcm_filename);
			free(msg_buff);
			return errno;
		}
	}

	// write query version message to device
	msg_len = rcm_get_msg_len(msg_buff);
	if (msg_len == 0) {
		fprintf(stderr, "write RCM query version: unknown message length\n");
		return EINVAL;
	}
	ret = usb_write(usb, msg_buff, msg_len);
	if (ret) {
		fprintf(stderr, "write RCM query version: USB transfer failure\n");
		goto done;
	}

	// read response
	ret = usb_read(usb, (uint8_t *)&status, sizeof(status), &actual_len);
	if (ret) {
		fprintf(stderr, "read RCM query version: USB transfer failure\n");
		goto done;
	}
	if (actual_len < sizeof(status)) {
		fprintf(stderr, "read RCM query version: USB read truncated\n");
		ret = EIO;
		goto done;
	}
	printf("RCM version: %d.%d\n", RCM_VERSION_MAJOR(status),
	       RCM_VERSION_MINOR(status));

done:
	if (msg_buff)
		free(msg_buff);
	return ret;
}

static int initialize_miniloader(uint16_t devid, usb_device_t *usb,
		char *mlfile, uint32_t mlentry, const char *signed_msgs_file,
		bool download_signed_msgs)
{
	int fd;
	struct stat sb;
	int ret;
	uint8_t *miniloader = NULL;
	bool ml_buffer_alloc = false;
	uint32_t miniloader_size;
	uint32_t miniloader_entry;
	char ml_rcm_filename[PATH_MAX];
	char *filename = NULL;

	// use prebuilt miniloader if not loading from a file
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
	} else {
		fprintf(stderr, "unknown tegra device: 0x%x\n", devid);
		return ENODEV;
	}

	// if loading unsigned ml from a file
	if (mlfile)
		filename = mlfile;

	// if loading signed ml rcm file
	if (signed_msgs_file && (download_signed_msgs == true)) {
		filename = ml_rcm_filename;
		ret = create_name_string(ml_rcm_filename, signed_msgs_file,
					".ml");
		if (ret)
			goto done;

		/* download signed ml rcm blob */
		printf("download signed miniloader rcm from file %s\n",
			 ml_rcm_filename);
	}

	// loading ml or signed ml rcm from a file
	if (filename) {
		fd = open(filename, O_RDONLY, 0);
		if (fd < 0) {
			dprintf("error opening %s\n", filename);
			return errno;
		}
		ret = fstat(fd, &sb);
		if (ret) {
			dprintf("error on fstat of %s\n", filename);
			return ret;
		}
		miniloader_size = sb.st_size;
		miniloader = (uint8_t *)malloc(miniloader_size);
		if (!miniloader) {
			dprintf("error allocating %d bytes for miniloader\n", miniloader_size);
			return errno;
		}
		ml_buffer_alloc = true;
		if (read(fd, miniloader, miniloader_size) != miniloader_size) {
			dprintf("error reading from miniloader file");
			return errno;
		}
	}

	// if entry is specified
	if (mlentry)
		miniloader_entry = mlentry;

	// if generating ml rcm file
	if (signed_msgs_file && (download_signed_msgs == false)) {
		ret = create_miniloader_rcm(miniloader, miniloader_size,
				miniloader_entry, signed_msgs_file);
		goto done;
	}

	printf("downloading miniloader to target at address 0x%x (%d bytes)...\n",
		miniloader_entry, miniloader_size);
	ret = download_miniloader(usb, miniloader, miniloader_size,
				  miniloader_entry, download_signed_msgs);
	if (ret) {
		fprintf(stderr, "Error downloading miniloader\n");
		return ret;
	}
	printf("miniloader downloaded successfully\n");
done:
	if ((ml_buffer_alloc == true) && miniloader)
		free(miniloader);
	return ret;
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

static int create_miniloader_rcm(uint8_t *miniloader, uint32_t size,
			uint32_t entry, const char *signed_msgs_file)
{
	uint8_t *msg_buff;
	int ret = 0;
	char ml_rcm_filename[PATH_MAX];

	// create RCM_CMD_DL_MINILOADER blob
	rcm_create_msg(RCM_CMD_DL_MINILOADER, (uint8_t *)&entry, sizeof(entry),
		       miniloader, size, &msg_buff);

	ret = create_name_string(ml_rcm_filename, signed_msgs_file, ".ml");
	if (ret)
		goto done;

	// write to binary file
	dprintf("Write miniloader rcm to %s\n", ml_rcm_filename);

	ret = save_to_file(ml_rcm_filename, msg_buff,
				rcm_get_msg_len(msg_buff));
done:
	free(msg_buff);
	return ret;
}

static int download_miniloader(usb_device_t *usb, uint8_t *miniloader,
				uint32_t size, uint32_t entry,
				bool download_signed_msgs)
{
	uint8_t *msg_buff = NULL;
	bool msg_buff_alloc = false;
	int ret;
	uint32_t status;
	int actual_len;

	// download the miniloader to the bootrom
	if (download_signed_msgs == true)	/* signed ml with rcm header */
		msg_buff = miniloader;
	else {
		rcm_create_msg(RCM_CMD_DL_MINILOADER, (uint8_t *)&entry,
				sizeof(entry), miniloader, size, &msg_buff);
		msg_buff_alloc = true;
	}

	ret = usb_write(usb, msg_buff, rcm_get_msg_len(msg_buff));
	if (ret)
		goto fail;
	ret = usb_read(usb, (uint8_t *)&status, sizeof(status), &actual_len);
	if (ret)
		goto fail;
	if (actual_len < sizeof(status)) {
		ret = EIO;
		goto fail;
	}
	if (status != 0) {
		ret = EIO;
		goto fail;
	}

	ret = 0;
fail:
	if ((msg_buff_alloc == true) && msg_buff)
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
	case RCM_OP_MODE_ODM_SECURE_PKC:    op_mode = "odm secure mode with PKC"; break;
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

static int download_bootloader(nv3p_handle_t h3p, char *filename,
			uint32_t entry, uint32_t loadaddr,
			const char *pkc_keyfile, const char *signed_msgs_file)
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

	// For fused board, the bootloader hash must be sent first
	if (get_op_mode() == RCM_OP_MODE_ODM_SECURE_PKC) {
		if (pkc_keyfile)  {
			/* sign and download with --pkc option */
			uint8_t rsa_pss_sig[RCM_RSA_SIG_SIZE];

			ret = rsa_pss_sign_file(pkc_keyfile, filename, rsa_pss_sig);
			if (ret) {
				dprintf("error signing %s with %s\n",
					filename, pkc_keyfile);
				return ret;
			}

			ret = nv3p_data_send(h3p, rsa_pss_sig, sizeof(rsa_pss_sig));

		} else if (signed_msgs_file) {
			/* download bl's signature */
			char signature_filename[PATH_MAX];

			ret = create_name_string(signature_filename,
						signed_msgs_file, ".bl");
			if (ret)
				return ret;

			// send the bootloader's signature file
			ret = send_file(h3p, signature_filename);

		} else {
			/* missing both options */
			dprintf("error: missing both pkc keyfile and"
				" bootloader's signature file\n");
			return -1;
		}

		// check transfer status
		if (ret) {
			dprintf("error sending bootloader signature\n");
			return ret;
		}
	}

	// send the bootloader file
	ret = send_file(h3p, filename);
	if (ret) {
		dprintf("error downloading bootloader\n");
		return ret;
	}

	return 0;
}

static int sign_blob(const char *blob_filename, const char *pkc_keyfile,
			const char *signed_msgs_file)
{
	int ret;
	uint8_t rsa_pss_sig[RCM_RSA_SIG_SIZE];

	char signature_filename[PATH_MAX];

	ret = rsa_pss_sign_file(pkc_keyfile, blob_filename, rsa_pss_sig);
	if (ret) {
		fprintf(stderr, "error signing %s with %s\n",
			blob_filename, pkc_keyfile);
		return ret;
	}

	/* save signature to signed_msgs_file.bl */
	ret = create_name_string(signature_filename, signed_msgs_file, ".bl");
	if (ret)
		return ret;

	ret = save_to_file(signature_filename, rsa_pss_sig,
				sizeof(rsa_pss_sig));
	return ret;
}

static void set_platform_info(nv3p_platform_info_t *info)
{
	g_platform_info = info;
}

static uint32_t get_op_mode(void)
{
	if (g_platform_info)
		return g_platform_info->op_mode;

	fprintf(stderr, "Error: No platform info has been retrieved\n");
	return 0;
}
