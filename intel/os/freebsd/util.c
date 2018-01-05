/*
 * Copyright (c) 2013, Intel Corporation
 * Copyright (c) 2017, IBM Corporation
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
#include <sys/sysctl.h>

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../../../common/include/util.h"
#include "../../../common/include/os/os_util.h"

#define KERNEL_ADDR_START	0xffffffff80000000

/*
 * Check the cpu name in proc info. Intel CPUs always have @ x.y
 * Ghz and that is the TSC frequency.
 */
int
arch__cpuinfo_freq(double *freq, char *unit)
{
	char *model;
	size_t len;
	int ret;

	model = NULL;
	len = 0;
	ret = -1;

	/* Fetch the CPU model string */
	while (1) {
		int mib[2] = { CTL_HW, HW_MODEL };

		if (sysctl(mib, nitems(mib), model, &len, NULL, 0) == 0) {
			if (model != NULL)
				break;
			else
				errno = ENOMEM;
		}

		if (errno != ENOMEM)
			goto L_EXIT;

		if (model != NULL)
			free(model);

		if ((model = malloc(len)) == NULL)
			goto L_EXIT;
	}

	if (sscanf(model + strcspn(model, "@") + 1, "%lf%10s", freq,
	    unit) == 2) {
		if (strcasecmp(unit, "GHz") == 0) {
			*freq *= GHZ;
		} else if (strcasecmp(unit, "Mhz") == 0) {
			*freq *= MHZ;
		} else {
			goto L_EXIT;
		}
	}

	debug_print(NULL, 2, "arch__cpuinfo_freq: freq = %.4f, unit = %s\n",
	    *freq, unit);
	ret = 0;

L_EXIT:
	free(model);
	return (ret);
}

int
is_userspace(uint64_t ip)
{
	return ip < KERNEL_ADDR_START;
}
