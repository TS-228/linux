/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SMP hooks shared between arch/arm/mach-realtek and the RTD119x SoC
 * drivers.
 */

#ifndef __SOC_REALTEK_RTD119X_SMP_H
#define __SOC_REALTEK_RTD119X_SMP_H

struct smp_operations;
extern const struct smp_operations rtd1195_smp_ops;

void rtd119x_cpu_die(unsigned int cpu);

#endif /* __SOC_REALTEK_RTD119X_SMP_H */
