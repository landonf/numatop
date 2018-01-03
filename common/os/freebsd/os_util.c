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

#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include "../../include/types.h"
#include "../../include/util.h"
#include "../../include/os/os_util.h"

uint64_t g_clkofsec;
double g_nsofclk;
unsigned int g_pqos_moni_id;

boolean_t
os_authorized(void)
{
	return (B_TRUE);
}

int
os_numatop_lock(boolean_t *locked)
{
	/* Not supported on Linux */
	return (0);
}

void
os_numatop_unlock(void)
{
	/* Not supported on Linux */
}

int
os_procfs_psinfo_get(pid_t pid, void *info)
{
	/* Not supported on Linux */
	return (0);
}

/*
 * Retrieve the process's executable name from '/proc'
 */
int
os_procfs_pname_get(pid_t pid, char *buf, int size)
{
	return (ENOSYS);
}

/*
 * Retrieve the lwpid in process from '/proc'.
 */
int
os_procfs_lwp_enum(pid_t pid, int **lwps, int *num)
{
	return (ENOSYS);
}

/*
 * Check if the specified pid/lwpid can be found in '/proc'.
 */
boolean_t
os_procfs_lwp_valid(pid_t pid, int lwpid)
{
	/* Not supported on Linux */
	return (B_TRUE);
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
os_sysfs_node_enum(int *node_arr, int arr_size, int *num)
{
	return (false);
}

boolean_t
os_sysfs_cpu_enum(int nid, int *cpu_arr, int arr_size, int *num)
{
	return (false);
}

int
os_sysfs_online_ncpus(void)
{
#ifdef FBSD_TODO
	int cpu_arr[NCPUS_MAX], num;
	char path[PATH_MAX];

	if (sysconf(_SC_NPROCESSORS_CONF) > NCPUS_MAX) {
		return (-1);
	}

	snprintf(path, PATH_MAX, "/sys/devices/system/cpu/online");
	if (!file_int_extract(path, cpu_arr, NCPUS_MAX, &num)) {
		return (-1);
	}

	return (num);
#else
	return (0);
#endif
}

boolean_t
os_sysfs_meminfo(int nid, node_meminfo_t *info)
{
	return (false);
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
	return (false);
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
