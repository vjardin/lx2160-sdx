/*
 * lx2160-mc — stop, reload and restart the LX2160A Management Complex
 * from Linux userland.
 *
 * Unlike the other lx2160-* tools this one is NOT read-only: it opens
 * /dev/mem O_RDWR and drives the MC through reset by design.
 */

#define _DEFAULT_SOURCE

#include <dirent.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/sockios.h>
#include <time.h>
#include <unistd.h>

#define EXIT_USAGE 2

#define MC_CCSR_BASE       0x08340000UL
#define MC_CCSR_LEN        0x1000UL

#define OFF_GCR1           0x00
#define OFF_GSR            0x08
#define OFF_MCFBALR        0x20
#define OFF_MCFBAHR        0x24
#define OFF_MCFAPR         0x28

#define GCR1_P1_STOP       (1u << 31)
#define GCR1_P2_STOP       (1u << 30)
#define GCR1_P1_DE_RST     (1u << 23)
#define GCR1_M1_DE_RST     (1u << 15)
#define GCR1_M2_DE_RST     (1u << 14)
#define GCR1_RELEASE       (GCR1_P1_DE_RST | GCR1_M1_DE_RST | GCR1_M2_DE_RST)

#define GSR_FS_MASK        0x3fffffffu
#define GSR_DELAYED_DPL    0x0000dd00u
#define GSR_BOOT_OK        0x1u

/* MCFAPR = AMQ_PL | AMQ_BMT: privileged, and bypass SMMU translation. */
#define MCFAPR_BYPASS      0x00060000u

/* Offsets inside the MC's private DRAM block. */
#define MC_DPC_OFFSET      0x00F00000UL
#define MC_DPL_OFFSET      0x00F20000UL
#define MC_LOG_OFFSET      0x01000000UL
#define MC_LOG_MAGIC       0x4d430100u

#define MC_BOOT_TIMEOUT_MS 30000

struct log_header {
	uint32_t magic_word;
	uint32_t reserved;
	uint32_t buf_start;
	uint32_t buf_length;
	uint32_t last_byte;
};
#define LOG_WRAPAROUND     0x80000000u

static int devmem_fd = -1;

static void devmem_open(void)
{
	devmem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (devmem_fd < 0)
		err(EXIT_FAILURE, "/dev/mem (are you root?)");
}

/* Map phys [addr, addr+len); returns a pointer to addr itself. */
static volatile uint8_t *map_phys(uint64_t addr, size_t len, void **base,
				  size_t *maplen)
{
	long ps = sysconf(_SC_PAGESIZE);
	uint64_t aligned = addr & ~((uint64_t)ps - 1);
	size_t off = (size_t)(addr - aligned);
	uint8_t *p;

	*maplen = len + off;
	p = mmap(NULL, *maplen, PROT_READ | PROT_WRITE, MAP_SHARED,
		 devmem_fd, (off_t)aligned);
	if (p == MAP_FAILED)
		err(EXIT_FAILURE, "mmap %#" PRIx64 " (%zu bytes)", addr, len);
	*base = p;
	return p + off;
}

static uint32_t ccsr_rd(unsigned off)
{
	void *base;
	size_t maplen;
	volatile uint8_t *p = map_phys(MC_CCSR_BASE, MC_CCSR_LEN, &base, &maplen);
	uint32_t v = *(volatile uint32_t *)(p + off);

	munmap(base, maplen);
	return v;
}

static void ccsr_wr(unsigned off, uint32_t val)
{
	void *base;
	size_t maplen;
	volatile uint8_t *p = map_phys(MC_CCSR_BASE, MC_CCSR_LEN, &base, &maplen);

	*(volatile uint32_t *)(p + off) = val;
	__sync_synchronize();
	munmap(base, maplen);
}

/* Device-memory-safe copies: aligned 64-bit where possible, else bytes. */
static void copy_to_phys(volatile uint8_t *dst, const uint8_t *src, size_t n)
{
	size_t i = 0;

	if ((((uintptr_t)dst) & 7) == 0)
		for (; i + 8 <= n; i += 8) {
			uint64_t v;

			memcpy(&v, src + i, 8);	/* src is normal RAM */
			*(volatile uint64_t *)(dst + i) = v;
		}
	for (; i < n; i++)
		dst[i] = src[i];
}

static void copy_from_phys(uint8_t *dst, volatile const uint8_t *src, size_t n)
{
	size_t i = 0;

	if ((((uintptr_t)src) & 7) == 0)
		for (; i + 8 <= n; i += 8) {
			uint64_t v = *(volatile const uint64_t *)(src + i);

			memcpy(dst + i, &v, 8);
		}
	for (; i < n; i++)
		dst[i] = src[i];
}

/* Where the MC's private DRAM block lives, straight out of MCFBALR/AHR. */
static void mc_block(uint64_t *base, uint64_t *size)
{
	uint32_t lo = ccsr_rd(OFF_MCFBALR);
	uint32_t hi = ccsr_rd(OFF_MCFBAHR);

	*base = ((uint64_t)hi << 32) | (lo & ~0x3fu);
	/* U-Boot writes (num_256mb_blocks - 1) in the low bits. */
	*size = (uint64_t)((lo & 0x3f) + 1) * (256UL << 20);
}

static void load_file(const char *path, uint64_t addr)
{
	FILE *f = fopen(path, "rb");
	long len;
	uint8_t *buf;
	void *base;
	size_t maplen;
	volatile uint8_t *p;

	if (!f)
		err(EXIT_FAILURE, "%s", path);
	if (fseek(f, 0, SEEK_END) != 0 || (len = ftell(f)) < 0)
		err(EXIT_FAILURE, "%s: seek", path);
	rewind(f);
	buf = malloc((size_t)len);
	if (!buf)
		err(EXIT_FAILURE, "malloc");
	if (fread(buf, 1, (size_t)len, f) != (size_t)len)
		errx(EXIT_FAILURE, "%s: short read", path);
	fclose(f);

	p = map_phys(addr, (size_t)len, &base, &maplen);
	copy_to_phys(p, buf, (size_t)len);
	munmap(base, maplen);
	free(buf);
	printf("  loaded %-24s %8ld bytes -> %#" PRIx64 "\n", path, len, addr);
}

static void wipe(uint64_t addr, uint64_t len)
{
	const uint64_t slice = 64UL << 20;

	for (uint64_t done = 0; done < len; done += slice) {
		uint64_t n = (len - done < slice) ? len - done : slice;
		void *base;
		size_t maplen;
		volatile uint8_t *p = map_phys(addr + done, (size_t)n, &base, &maplen);
		uint64_t i = 0;

		for (; i + 8 <= n; i += 8)
			*(volatile uint64_t *)(p + i) = 0;
		for (; i < n; i++)
			p[i] = 0;
		munmap(base, maplen);
	}
	printf("  wiped %#" PRIx64 " bytes at %#" PRIx64 "\n", len, addr);
}

static int wait_fs1(const char *what)
{
	for (int ms = 0; ms < MC_BOOT_TIMEOUT_MS; ms += 50) {
		uint32_t gsr = ccsr_rd(OFF_GSR);

		if ((gsr & GSR_FS_MASK) == GSR_BOOT_OK) {
			printf("  %s: OK after %d ms (GSR=%#010x)\n", what, ms, gsr);
			return 0;
		}
		usleep(50000);
	}
	warnx("%s: FAILED, GSR=%#010x (U-Boot only accepts FS==0x1)", what, ccsr_rd(OFF_GSR));
	return -1;
}

static void cmd_status(void)
{
	uint32_t gcr1 = ccsr_rd(OFF_GCR1);
	uint32_t gsr = ccsr_rd(OFF_GSR);
	uint32_t apr = ccsr_rd(OFF_MCFAPR);
	uint64_t base, size;

	mc_block(&base, &size);

	printf("MC CCSR   %#010lx\n", MC_CCSR_BASE);
	printf("  GCR1    %#010x  [%s%s%s%s%s]\n", gcr1,
	       (gcr1 & GCR1_P1_STOP) ? "P1_STOP " : "",
	       (gcr1 & GCR1_P2_STOP) ? "P2_STOP " : "",
	       (gcr1 & GCR1_P1_DE_RST) ? "P1_DE_RST " : "",
	       (gcr1 & GCR1_M1_DE_RST) ? "M1_DE_RST " : "",
	       (gcr1 & GCR1_M2_DE_RST) ? "M2_DE_RST" : "");
	printf("  GSR     %#010x  FS=%#x %s\n", gsr, gsr & GSR_FS_MASK,
	       (gsr & GSR_FS_MASK) == GSR_BOOT_OK ? "(booted)" : "(not ready)");
	printf("  MCFAPR  %#010x  %s\n", apr,
	       (apr & MCFAPR_BYPASS) == MCFAPR_BYPASS ?
	       "PL|BMT - firmware fetch bypasses the SMMU" : "translated");
	printf("\nMC private DRAM\n");
	printf("  base    %#012" PRIx64 "\n", base);
	printf("  size    %" PRIu64 " MB\n", size >> 20);
	printf("  DPC     %#012" PRIx64 "\n", base + MC_DPC_OFFSET);
	printf("  DPL     %#012" PRIx64 "\n", base + MC_DPL_OFFSET);
	printf("  log     %#012" PRIx64 "\n", base + MC_LOG_OFFSET);
}

static void cmd_log(void)
{
	uint64_t base, size;
	struct log_header hdr;
	void *mbase;
	size_t maplen;
	volatile uint8_t *p;
	uint32_t valid;
	uint8_t *buf;
	uint64_t logaddr;

	mc_block(&base, &size);
	p = map_phys(base + MC_LOG_OFFSET, sizeof(hdr), &mbase, &maplen);
	copy_from_phys((uint8_t *)&hdr, p, sizeof(hdr));
	munmap(mbase, maplen);

	if (hdr.magic_word != MC_LOG_MAGIC)
		errx(EXIT_FAILURE,
		     "bad log magic %#010x (expected %#010x) - MC never booted?",
		     hdr.magic_word, MC_LOG_MAGIC);

	valid = hdr.last_byte & ~LOG_WRAPAROUND;
	if (valid > hdr.buf_length)
		valid = hdr.buf_length;
	logaddr = base + hdr.buf_start;

	buf = malloc(valid + 1);
	if (!buf)
		err(EXIT_FAILURE, "malloc");
	p = map_phys(logaddr, valid, &mbase, &maplen);
	copy_from_phys(buf, p, valid);
	munmap(mbase, maplen);

	/* The ring is zero-padded; print it as text. */
	for (uint32_t i = 0; i < valid; i++)
		if (buf[i] == '\0')
			buf[i] = '\n';
	buf[valid] = '\0';
	fputs((char *)buf, stdout);
	free(buf);
}

#define FSL_MC_DEVICES "/sys/bus/fsl-mc/devices"
#define DPAA2_ETH_UNBIND "/sys/bus/fsl-mc/drivers/fsl_dpaa2_eth/unbind"

static int sysfs_write(const char *path, const char *val)
{
	int fd = open(path, O_WRONLY);
	ssize_t n;

	if (fd < 0)
		return -1;
	n = write(fd, val, strlen(val));
	close(fd);
	return n > 0 ? 0 : -1;
}

/*
 * Bring every DPAA2 netdev administratively down.
 *
 * This has to happen before the DPIOs are torn down:
 * the MC's global_init probes QBMan software portal 0 on the next boot,
 * and if the datapath was still live when Linux let go,
 * SWP 0 does not answer and the MC gives up with "SWP 0 is not responding" / MC error status 0x19.
 */
static void links_down(void)
{
	DIR *d = opendir("/sys/class/net");
	struct dirent *e;
	int fd;
	int n = 0;

	if (!d)
		return;
	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		closedir(d);
		return;
	}
	while ((e = readdir(d))) {
		char link[PATH_MAX], path[PATH_MAX];
		struct ifreq ifr;
		ssize_t r;

		if (e->d_name[0] == '.' || !strcmp(e->d_name, "lo"))
			continue;
		snprintf(path, sizeof(path), "/sys/class/net/%.63s/device",
			 e->d_name);
		r = readlink(path, link, sizeof(link) - 1);
		if (r < 0)
			continue;
		link[r] = '\0';
		if (!strstr(link, "dpni."))	/* only DPAA2 interfaces */
			continue;

		memset(&ifr, 0, sizeof(ifr));
		snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%.15s",
			 e->d_name);
		if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0)
			continue;
		if (!(ifr.ifr_flags & IFF_UP))
			continue;
		ifr.ifr_flags &= ~IFF_UP;
		if (ioctl(fd, SIOCSIFFLAGS, &ifr) == 0)
			n++;
	}
	close(fd);
	closedir(d);
	printf("  brought %d DPAA2 netdev(s) down\n", n);
}

static int count_fsl_mc(void)
{
	DIR *d = opendir(FSL_MC_DEVICES);
	struct dirent *e;
	int n = 0;

	if (!d)
		return -1;
	while ((e = readdir(d)))
		if (e->d_name[0] != '.')
			n++;
	closedir(d);
	return n;
}

static int count_dpni(void)
{
	DIR *d = opendir(FSL_MC_DEVICES);
	struct dirent *e;
	int n = 0;

	if (!d)
		return -1;
	while ((e = readdir(d)))
		if (!strncmp(e->d_name, "dpni.", 5))
			n++;
	closedir(d);
	return n;
}

/*
 * Delete every DPNI through the running MC.
 *
 * XXX hacky !
 * It frees the CTLU. It lives in WRIOP hardware, not in MC DRAM, and
 * the MC's own CTLU init never clears them.
 * So without this a restarted MC fails every dpni_init with
 * "dpkg_profile ... was created earlier" and the DPL apply returns 0x3f.
 */
static int cmd_release(void)
{
	DIR *d;
	struct dirent *e;
	char names[64][32];
	int n = 0, before, after;

	before = count_dpni();
	printf("releasing %d DPNI(s) through the live MC\n", before);

	links_down();
	sleep(2);

	d = opendir(FSL_MC_DEVICES);
	if (!d)
		err(EXIT_FAILURE, FSL_MC_DEVICES);
	while ((e = readdir(d)) && n < (int)(sizeof(names) / sizeof(names[0])))
		if (!strncmp(e->d_name, "dpni.", 5) &&
		    strlen(e->d_name) < sizeof(names[0]))
			snprintf(names[n++], sizeof(names[0]), "%.31s",
				 e->d_name);
	closedir(d);

	for (int i = 0; i < n; i++)
		sysfs_write(DPAA2_ETH_UNBIND, names[i]);
	sleep(2);

	for (int i = 0; i < n; i++) {
		char cmd[96];

		snprintf(cmd, sizeof(cmd),
			 "restool dpni destroy %.31s >/dev/null 2>&1",
			 names[i]);
		int rc = system(cmd);	/* status is unreliable, see above */

		(void)rc;
	}
	sleep(3);

	after = count_dpni();
	printf("  DPNIs: %d -> %d\n", before, after);
	if (after != 0) {
		warnx("%d DPNI(s) survived; the restart will fail on CTLU "
		      "profile collisions", after);
		return -1;
	}
	return 0;
}

static void cmd_stop(void)
{
	printf("stopping MC (GCR1 = 0)\n");
	ccsr_wr(OFF_GCR1, 0);
	usleep(200000);
	printf("  GCR1 now %#010x\n", ccsr_rd(OFF_GCR1));
}

static int cmd_start(void)
{
	uint64_t base, size;

	mc_block(&base, &size);
	printf("starting MC at %#012" PRIx64 "\n", base);
	ccsr_wr(OFF_MCFAPR, MCFAPR_BYPASS);
	ccsr_wr(OFF_GSR, GSR_DELAYED_DPL);
	ccsr_wr(OFF_GCR1, GCR1_RELEASE);
	return wait_fs1("MC boot");
}

static int cmd_restart(const char *fw, const char *dpc, const char *dpl,
		       bool do_wipe, bool do_release)
{
	uint64_t base, size;
	uint32_t lo, hi;

	if (!fw || !dpc)
		errx(EXIT_USAGE, "restart needs at least --fw and --dpc");

	if (do_release) {
		if (cmd_release() != 0)
			return -1;
		/* Now let go of the root container itself. */
		if (sysfs_write("/sys/bus/fsl-mc/drivers/fsl_mc_dprc/unbind",
				"dprc.1") != 0)
			warnx("could not unbind dprc.1 (already unbound?)");
		/*
		 * Give the DPIOs time to hand their QBMan software portals
		 * back.
		 */
		sleep(8);
		printf("  fsl-mc devices left: %d\n", count_fsl_mc());
	}

	mc_block(&base, &size);
	lo = ccsr_rd(OFF_MCFBALR);
	hi = ccsr_rd(OFF_MCFBAHR);
	printf("MC block %#012" PRIx64 " (%" PRIu64 " MB)\n", base, size >> 20);

	cmd_stop();

	if (do_wipe) {
		/*
		 * Mandatory. GCR1=0 resets the cores but not their memory, so
		 * a second instance otherwise collides with the first one's
		 * bookkeeping ("dpkg_profile = 3 ... was created earlier")
		 * and every DPNI fails to initialise.
		 */
		printf("wiping the carve-out\n");
		wipe(base, size);
	}

	printf("loading blobs\n");
	load_file(fw, base);
	load_file(dpc, base + MC_DPC_OFFSET);
	if (dpl)
		load_file(dpl, base + MC_DPL_OFFSET);

	/* Restore the block description the wipe did not touch. */
	ccsr_wr(OFF_MCFBALR, lo);
	ccsr_wr(OFF_MCFBAHR, hi);

	if (cmd_start() != 0)
		return -1;

	if (dpl) {
		printf("deploying the DPL (GSR = 0)\n");
		ccsr_wr(OFF_GSR, 0);
		if (wait_fs1("DPL apply") != 0)
			return -1;
	}

	printf("\nMC restarted. Linux must now re-attach, then the netdev numbering has to be put back (the rebind enumerates by\n"
	       "descending DPNI id, so ethN may not matche dpni.N anymore):\n"
	       "  echo dprc.1 > /sys/bus/fsl-mc/drivers/fsl_mc_dprc/bind\n"
	       "  # /etc/ethnames.sh ## do your script\n");
	return 0;
}

static void usage(FILE *fp)
{
	fprintf(fp,
		"usage: lx2160-mc <command> [options]\n"
		"\n"
		"commands:\n"
		"  status                 CCSR registers + decoded carve-out\n"
		"  log                    dump the MC's DDR log ring\n"
		"  release                destroy every DPNI via the live MC\n"
		"  stop                   GCR1 = 0 (hold the cores in reset)\n"
		"  start                  release the cores, wait for FS==1\n"
		"  restart                the whole sequence\n"
		"\n"
		"restart options:\n"
		"  --fw <file>            raw MC firmware (dumpimage -T flat_dt -p 0 mc.itb)\n"
		"  --dpc <file>           compiled DPC dtb\n"
		"  --dpl <file>           compiled DPL dtb (optional)\n"
		"  --no-release           do not free the DPNIs / unbind dprc.1 first\n"
		"  --no-wipe              skip the carve-out wipe\n"
		"\n"
		"Without --no-release, 'restart' frees the DPNIs (which releases their\n"
		"CTLU profiles in WRIOP, it is mandatory) and it unbinds dprc.1 for you.\n"
		"\n"
		"Grab the DPC the board is really running, MAC fixups included:\n"
		"  base=$(lx2160-mc status | awk '/^  base/{print $2}')\n");
}

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "fw",      required_argument, NULL, 'f' },
		{ "dpc",     required_argument, NULL, 'c' },
		{ "dpl",     required_argument, NULL, 'l' },
		{ "no-wipe", no_argument,       NULL, 'W' },
		{ "no-release", no_argument,    NULL, 'R' },
		{ "help",    no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	const char *fw = NULL, *dpc = NULL, *dpl = NULL;
	bool do_wipe = true, do_release = true;
	const char *cmd;
	int c, rc = 0;

	if (argc < 2) {
		usage(stderr);
		return EXIT_USAGE;
	}
	cmd = argv[1];
	if (!strcmp(cmd, "-h") || !strcmp(cmd, "--help")) {
		usage(stdout);
		return 0;
	}

	optind = 2;
	while ((c = getopt_long(argc, argv, "f:c:l:WRh", opts, NULL)) != -1) {
		switch (c) {
		case 'f': fw = optarg; break;
		case 'c': dpc = optarg; break;
		case 'l': dpl = optarg; break;
		case 'W': do_wipe = false; break;
		case 'R': do_release = false; break;
		case 'h': usage(stdout); return 0;
		default:  usage(stderr); return EXIT_USAGE;
		}
	}

	devmem_open();

	if (!strcmp(cmd, "status"))
		cmd_status();
	else if (!strcmp(cmd, "log"))
		cmd_log();
	else if (!strcmp(cmd, "stop"))
		cmd_stop();
	else if (!strcmp(cmd, "start"))
		rc = cmd_start();
	else if (!strcmp(cmd, "release"))
		rc = cmd_release();
	else if (!strcmp(cmd, "restart"))
		rc = cmd_restart(fw, dpc, dpl, do_wipe, do_release);
	else {
		warnx("unknown command '%s'", cmd);
		usage(stderr);
		return EXIT_USAGE;
	}

	close(devmem_fd);
	return rc == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
