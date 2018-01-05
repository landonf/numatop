/*
 * Copyright (c) 2013, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Intel Corporation nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/cpuset.h>
#include <sys/sysctl.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <kvm.h>
#include <limits.h>
#include <math.h>
#include <paths.h>

#define boolean_t vm_boolean_t
#include <sys/user.h>
#undef boolean_t

#include "../../include/types.h"
#include "../../include/util.h"
#include "../../include/os/os_util.h"

uint64_t g_clkofsec;
double g_nsofclk;
unsigned int g_pqos_moni_id;

static kvm_t *kd;
static pthread_mutex_t kd_mutex = PTHREAD_MUTEX_INITIALIZER;

int
os_init(void)
{
	char errbuf[_POSIX2_LINE_MAX];

	kd = kvm_openfiles(NULL, _PATH_DEVNULL, NULL, O_RDONLY, errbuf);
	if (kd == NULL) {
		debug_print(NULL, 2, "kvm_openfiles() failed: %s\n",
		    errbuf);
		errno = ENXIO;
		return -1;
	}

	return 0;
}

void
os_fini(void)
{
	if (kd != NULL) {
		kvm_close(kd);
		kd = NULL;
	}
}

boolean_t
os_authorized(void)
{
	return (B_TRUE);
}

int
os_numatop_lock(boolean_t *locked)
{
	/* Not supported on FreeBSD */
	return (0);
}

void
os_numatop_unlock(void)
{
	/* Not supported on FreeBSD */
}

/*
 * Retrieve a malloc()'d array of all process pids
 */
int
os_proc_enum(pid_t **pids, int *num)
{
	struct kinfo_proc *procs;
	pid_t *parray = NULL;
	int ret;
	int nprocs;

	ret = -1;

	pthread_mutex_lock(&kd_mutex);
	procs = kvm_getprocs(kd, KERN_PROC_PROC, 0, &nprocs);
	if (procs == NULL) {
		debug_print(NULL, 2, "os_proc_enum failed: %s\n",
		    kvm_geterr(kd));
		goto L_EXIT;
	}

	parray = calloc(nprocs, sizeof(*parray));
	if (parray == NULL) {
		goto L_EXIT;
	}

	*num = nprocs;
	*pids = parray;

	for (int i = 0; i < nprocs; i++) {
		/* Skip zombie processes */
		if (procs[i].ki_stat == SZOMB) {
			(*num)--;
			continue;
		}

		parray[i] = procs[i].ki_pid;
	}

	ret = 0;

L_EXIT:
	pthread_mutex_unlock(&kd_mutex);

	if (ret != 0) {
		free(parray);
	}

	return (ret);
}

int
os_psinfo_get(pid_t pid, void *info)
{
	/* Not supported on FreeBSD */
	return (0);
}

/*
 * Retrieve the process's executable name.
 */
int
os_pname_get(pid_t pid, char *buf, int size)
{
	struct kinfo_proc *proc;
	int ret;
	int nprocs;

	ret = -1;

	pthread_mutex_unlock(&kd_mutex);
	proc = kvm_getprocs(kd, KERN_PROC_PID, pid, &nprocs);
	if (proc == NULL) {
		debug_print(NULL, 2, "os_pname_get failed: %s (pid = %jd)\n",
		    kvm_geterr(kd), (intmax_t)pid);
		goto L_EXIT;
	}

	if (nprocs != 1) {
		debug_print(NULL, 2, "os_pname_get failed: kvm_getprocs() "
		    "returned %d procs\n",
		    nprocs);
		goto L_EXIT;
	}

	strlcpy(buf, proc->ki_comm, size);

L_EXIT:
	pthread_mutex_unlock(&kd_mutex);
	return (ret);
}

/*
 * Retrieve the lwpid in process.
 */
int
os_proc_lwp_enum(pid_t pid, int **lwps, int *num)
{
	struct kinfo_proc *procs;
	int *parray = NULL;
	int ret;
	int nprocs;

	ret = -1;

	pthread_mutex_unlock(&kd_mutex);
	procs = kvm_getprocs(kd, KERN_PROC_PID | KERN_PROC_INC_THREAD, pid,
	    &nprocs);
	if (procs == NULL) {
		debug_print(NULL, 2, "os_proc_lwp_enum failed: %s "
		    "(pid = %jd)\n", kvm_geterr(kd), (intmax_t)pid);
		goto L_EXIT;
	}

	parray = calloc(nprocs, sizeof(*parray));
	if (parray == NULL) {
		goto L_EXIT;
	}

	for (int i = 0; i < nprocs; i++) {
		parray[i] = procs[i].ki_tid;
	}

	*lwps = parray;
	*num = nprocs;
	ret = 0;

L_EXIT:
	pthread_mutex_unlock(&kd_mutex);

	if (ret != 0) {
		free(parray);
	}

	return (ret);
}

/*
 * Check if the specified pid/lwpid can be found in '/proc'.
 */
boolean_t
os_proc_lwp_valid(pid_t pid, int lwpid)
{
	struct kinfo_proc *procs;
	boolean_t ret;
	int nprocs;

	ret = B_FALSE;

	pthread_mutex_unlock(&kd_mutex);
	procs = kvm_getprocs(kd, KERN_PROC_PID | KERN_PROC_INC_THREAD, pid,
	    &nprocs);
	if (procs == NULL) {
		debug_print(NULL, 2, "os_proc_lwp_valid failed: %s "
		    "(pid = %jd, lwpid=%d)\n", kvm_geterr(kd), (intmax_t)pid,
		    lwpid);
		goto L_EXIT;
	}

	for (int i = 0; i < nprocs; i++) {
		if (procs[i].ki_pid != pid)
			continue;

		if (procs[i].ki_tid != lwpid)
			continue;

		ret = B_TRUE;
		break;
	}

L_EXIT:
	pthread_mutex_unlock(&kd_mutex);
	return (ret);
}

/*
 * Bind current thread to a cpu or unbind current thread
 * from a cpu.
 */
int
processor_bind(int cpu)
{
#ifdef FBSD_TODO
	cpu_set_t cs;

	CPU_ZERO (&cs);
	CPU_SET (cpu, &cs);

	if (sched_setaffinity(0, sizeof (cs), &cs) < 0) {
		debug_print(NULL, 2, "Fail to bind to CPU%d\n", cpu);
    		return (-1);
    	}

	return (0);
#else
	return (ENOSYS);
#endif
}

int
processor_unbind(void)
{
#ifdef FBSD_TODO
	cpu_set_t cs;
	int i;

	CPU_ZERO (&cs);
	for (i = 0; i < g_ncpus; i++) {
		CPU_SET (i, &cs);
	}

	if (sched_setaffinity(0, sizeof (cs), &cs) < 0) {
		debug_print(NULL, 2, "Fail to unbind from CPU\n");
		return (-1);
	}

	return (0);
#else
	return (ENOSYS);
#endif
}

static int
calibrate_cpuinfo(double *nsofclk, uint64_t *clkofsec)
{
	char unit[11] = {0};
	double freq = 0.0;

	if (arch__cpuinfo_freq(&freq, unit)) {
		return -1;
	}

	if (fabsl(freq) < 1.0E-6) {
		return (-1);
	}

	*clkofsec = freq;
	*nsofclk = (double)NS_SEC / *clkofsec;

	debug_print(NULL, 2, "calibrate_cpuinfo: nsofclk = %.4f, "
	    "clkofsec = %lu\n", *nsofclk, *clkofsec);

	return (0);
}

/*
 * On all recent Intel CPUs, the TSC frequency is always
 * the highest p-state. So get that frequency from sysfs.
 * e.g. 2262000
 */
static int
calibrate_cpufreq(double *nsofclk, uint64_t *clkofsec)
{
#ifdef FBSD_TODO
	int fd, i;
	char buf[32];
	uint64_t freq;

	if ((fd = open(CPU0_CPUFREQ_PATH, O_RDONLY)) < 0) {
		return (-1);
	}

	if ((i = read(fd, buf, sizeof (buf) - 1)) <= 0) {
		close(fd);
		return (-1);
	}

	close(fd);
	buf[i] = 0;
	if ((freq = atoll(buf)) == 0) {
		return (-1);
	}

	*clkofsec = freq * KHZ;
	*nsofclk = (double)NS_SEC / *clkofsec;

	debug_print(NULL, 2, "calibrate_cpufreq: nsofclk = %.4f, "
	    "clkofsec = %lu\n", *nsofclk, *clkofsec);

	return (0);
#else
	return (ENOSYS);
#endif
}

/*
 * Measure how many TSC cycles in a second and how many
 * nanoseconds in a TSC cycle.
 */
static void
calibrate_by_tsc(double *nsofclk, uint64_t *clkofsec)
{
	uint64_t start_ms, end_ms, diff_ms;
	uint64_t start_tsc, end_tsc;
	int i;

	for (i = 0; i < g_ncpus; i++) {
		/*
		 * Bind current thread to cpuN to ensure the
		 * thread can not be migrated to another cpu
		 * while the rdtsc runs.
		 */
		if (processor_bind(i) == 0) {
			break;
		}
	}

	if (i == g_ncpus) {
		return;
	}

	/*
	 * Make sure the start_ms is at the beginning of
	 * one millisecond.
	 */
	end_ms = current_ms(&g_tvbase);
	while ((start_ms = current_ms(&g_tvbase)) == end_ms) {}

	start_tsc = rdtsc();
	while ((end_ms = current_ms(&g_tvbase)) < (start_ms + 100)) {}
	end_tsc = rdtsc();

	diff_ms = end_ms - start_ms;
	*nsofclk = (double)(diff_ms * NS_MS) /
	    (double)(end_tsc - start_tsc);

	*clkofsec = (uint64_t)((double)NS_SEC / *nsofclk);

	/*
	 * Unbind current thread from cpu once the measurement completed.
	 */
	processor_unbind();

	debug_print(NULL, 2, "calibrate_by_tsc: nsofclk = %.4f, "
	    "clkofsec = %lu\n", *nsofclk, *clkofsec);
}

/*
 * calibrate_by_tsc() is the last method used by os_calibrate()
 * to calculate cpu frequency if cpu freq is not available by both
 * procfs and sysfs.
 *
 * On intel, calibrate_by_tsc() uses TSC register which gets updated
 * in sync of processor clock and thus cpu freq can be calculated
 * programmatically using this register.
 *
 * OTOH, PowerPC does not have analogue to TSC. There is a register
 * called TB (Time Base) but it's get updated at constant freq and
 * thus we can't find cpu frequency using TB register. But for
 * powerpc, cpu frequency is always gets exposed via either procfs
 * or sysfs and thus there is no point for depending on any other
 * method for powerpc.
 */
void
os_calibrate(double *nsofclk, uint64_t *clkofsec)
{
	if (calibrate_cpuinfo(nsofclk, clkofsec) == 0) {
		return;
	}
	
	if (calibrate_cpufreq(nsofclk, clkofsec) == 0) {
		return;	
	}

	calibrate_by_tsc(nsofclk, clkofsec);
}

boolean_t
os_node_enum(int *node_arr, int arr_size, int *num)
{
	size_t len;
	int ndomains;

	len = sizeof(ndomains);
	if (sysctlbyname("vm.ndomains", &ndomains, &len, NULL, 0) < 0) {
		debug_print(NULL, 2, "sysctlbyname(vm.ndomains) failed (errno "
		    "= %d)\n");
		return B_FALSE;
	}

	if (arr_size < ndomains)
		return B_FALSE;

	/* FBSD_XXX: assumes that domain IDs are not sparse, and are allocated
	 * in monotonically increasing order starting with 0 */
	*num = ndomains;
	for (int i = 0; i < ndomains; i++)
		node_arr[i] = i;

	return B_TRUE;
}

boolean_t
os_cpu_enum(int nid, int *cpu_arr, int arr_size, int *num)
{
	cpuset_t mask;
	int cpu, ncpu;
	int ret;

	ret = cpuset_getaffinity(CPU_LEVEL_WHICH, CPU_WHICH_DOMAIN, nid,
	    sizeof(mask), &mask);
	if (ret < 0) {
		debug_print(NULL, 2, "cpuset_getaffinity() failed (errno = "
		    "%d)\n");
		return B_FALSE;
	}

	ncpu = CPU_COUNT(&mask);
	if (arr_size < ncpu)
		return B_FALSE;

	*num = ncpu;
	for (int i = 0; i < ncpu; i++) {
		if ((cpu = CPU_FFS(&mask)) == 0) {
			stderr_print("invalid cpu count\n");
			return B_FALSE;
		}

		cpu--;
		CPU_CLR(cpu, &mask);
		cpu_arr[i] = cpu;
	}

	return B_TRUE;
}

int
os_online_ncpus(void)
{
	cpuset_t mask;
	int ret;

	ret = cpuset_getaffinity(CPU_LEVEL_WHICH, CPU_WHICH_PID, -1,
	    sizeof(mask), &mask);
	if (ret < 0) {
		debug_print(NULL, 2, "cpuset_getaffinity() failed (errno = "
		    "%d)\n");
		return -1;
	}

	return (CPU_COUNT(&mask));
}

boolean_t
os_meminfo(int nid, node_meminfo_t *info)
{
	char *buf;
	const char *line;
	uintmax_t start, end;
	size_t len;
	int domain, seg;
	boolean_t ret = B_FALSE;

	buf = NULL;
	line = NULL;
	len = 0;
	seg = -1;
	domain = -1;
	start = UINTMAX_MAX;
	end = 0;

	memset(info, 0, sizeof(*info));

	while (1) {
		if (sysctlbyname("vm.phys_segs", buf, &len, NULL, 0) == 0) {
			if (buf != NULL)
				break;
			else
				errno = ENOMEM;
		}

		if (errno != ENOMEM)
			goto L_EXIT;

		if (buf != NULL)
			free(buf);

		if ((buf = malloc(len)) == NULL)
			goto L_EXIT;
	}

	// FBSD_XXX: parsing node memory information is fiddily, and the kernel
	// doesn't provide any of the stats we need here.
	//
	// We need to expand vmmeter, vm.vmtotal, et al to track track and
	// expose per-domain statistics.
	line = buf;
	while (1) {
		int nr;

		while (isspace_l(*line, NULL))
			line++;

		if (*line == '\0' || strstr(line, "SEGMENT") == line) {
			/* Hit EOF or the next segment record; record any stats
			 * from the previously parsed segment */
			if (seg != -1) {
				if (domain == -1 || start == UINTMAX_MAX ||
				    end == 0)
				{
					debug_print(NULL, 2, "missing required "
					    "attributes for seg %d\n", seg);
					goto L_EXIT;
				}

				if (domain == nid) {
					info->mem_total += end - start;
					// TODO: additional statistics unavailable
				}

				seg = -1;
				start = UINTMAX_MAX;
				end = 0x0;
				domain = -1;
			}

			/* Finished parsing? */
			if (*line == '\0')
				break;

			/* Start parsing next segment */
			if (sscanf(line, "SEGMENT %d:%n", &seg, &nr) != 1)
				goto L_EXIT;
		} else if (strstr(line, "start:") == line) {
			if (sscanf(line, "start: %jx:%n", &start, &nr) != 1)
				goto L_EXIT;
		} else if (strstr(line, "end:") == line) {
			if (sscanf(line, "end: %jx:%n", &end, &nr) != 1)
				goto L_EXIT;
		} else if (strstr(line, "domain:") == line) {
			if (sscanf(line, "domain: %d:%n", &domain, &nr) != 1)
				goto L_EXIT;
		} else {
			/* Skip unrecognized line */
			nr = 0;
			while (line[nr] != '\0' && line[nr] != '\n')
				nr++;
		}

		line += nr;
	}

	ret = B_TRUE;
	
L_EXIT:
	if (ret != B_TRUE && line != NULL)
		debug_print(NULL, 2, "parse error at '%.16s...'\n", line);

	if (buf != NULL)
		free(buf);

	return ret;
}

int
os_sysfs_uncore_qpi_init(qpi_info_t *qpi, int num)
{
	return (0);
}

int
os_sysfs_uncore_upi_init(qpi_info_t *qpi, int num)
{
	return (0);
}

int
os_sysfs_uncore_imc_init(imc_info_t *imc, int num)
{
	return (0);
}

boolean_t os_cmt_init(void)
{
	return B_FALSE;
}

void os_cmt_fini(void)
{
	// FBSD_TODO
	g_pqos_moni_id = 0;
}

int os_sysfs_cmt_task_set(int pid, int lwpid, struct _perf_pqos *pqos)
{
	return -1;
}

int os_sysfs_cmt_task_value(struct _perf_pqos *pqos, int nid)
{
	return -1;
}
