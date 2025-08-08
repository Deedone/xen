==========================================================
Design Proposal: Add SMMUv3 Stage-1 Support for XEN Guests
==========================================================

:Author:     Milan Djokic <milan_djokic@epam.com>
:Date:       2026-02-13
:Status:     Draft

Introduction
============

The SMMUv3 supports two stages of translation. Each stage of translation 
can be
independently enabled. An incoming address is logically translated from 
VA to
IPA in stage 1, then the IPA is input to stage 2 which translates the IPA to
the output PA. Stage 1 translation support is required to provide 
isolation between different
devices within OS. XEN already supports Stage 2 translation but there is no
support for Stage 1 translation.
This design proposal outlines the introduction of Stage-1 SMMUv3 support 
in Xen for ARM guests.

Motivation
==========

ARM systems utilizing SMMUv3 require stage-1 address translation to 
ensure secure DMA and
guest managed I/O memory mappings.
With stage-1 enabled, guest manages IOVA to IPA mappings through its own 
IOMMU driver.

This feature enables:

- Stage-1 translation for the guest domain
- Device passthrough with per-device I/O address space

Design Overview
===============

These changes provide emulated SMMUv3 support:

- **SMMUv3 Stage-1 Translation**: stage-1 and nested translation support
  in SMMUv3 driver.
- **vIOMMU Abstraction**: Virtual IOMMU framework for guest stage-1
  handling.
- **Register/Command Emulation**: SMMUv3 register emulation and command
  queue handling.
- **Device Tree Extensions**: Adds `iommus` and virtual SMMUv3 nodes to
  device trees for dom0 and dom0less scenarios.
- **Runtime Configuration**: Introduces a `viommu` boot parameter for
  dynamic enablement.

A single vIOMMU device is exposed to the guest and mapped to one or more
physical IOMMUs through a Xen-managed translation layer.
The vIOMMU feature provides a generic framework together with a backend
implementation specific to the target IOMMU type. The backend is responsible
for implementing the hardware-specific data structures and command handling
logic (currently only SMMUv3 is supported).

This modular design allows the stage-1 support to be reused
for other IOMMU architectures in the future.

vIOMMU architecture
===================

Responsibilities:

Guest:
 - Configures stage-1 via vIOMMU commands.
 - Handles stage-1 faults received from Xen.

Xen:
 - Emulates the IOMMU interface (registers, commands, events).
 - Provides vSID->pSID mappings.
 - Programs stage-1/stage-2 configuration in the physical IOMMU.
 - Propagate stage-1 faults to guest.

vIOMMU commands and faults are transmitted between guest and Xen via
command and event queues (one command/event queue created per guest).

vIOMMU command Flow:

::

    Guest:
        smmu_cmd(vSID, IOVA -> IPA)

    Xen:
        trap MMIO read/write
        translate vSID->pSID
        store stage-1 state
        program pIOMMU for (pSID, IPA -> PA)

All hardware programming of the physical IOMMU is performed exclusively by Xen.

vIOMMU Stage-1 fault handling flow:

::

    Xen:
        receives stage-1 fault
        triggers vIOMMU callback
        injects virtual fault

    Guest:
        receives and handles fault

vSID Mapping Layer
------------------

Each guest-visible Stream ID (vSID) is mapped by Xen to a physical Stream ID
(pSID). The mapping is maintained per-domain. The allocation policy guarantees
vSID uniqueness within a domain while allowing reuse of pSIDs for different
pIOMMUs.

* Platform devices receive individually allocated vSIDs.
* PCI devices receive a contiguous vSID range derived from RID space.


Supported Device Model
======================

Currently, the vIOMMU framework supports only devices described via the
Device Tree (DT) model. This includes platform devices and basic PCI
devices support instantiated through the vPCI DT node. ACPI-described
devices are not supported.

Guest assigned platform devices are mapped via `iommus` property:

::

    <&pIOMMU pSID> -> <&vIOMMU vSID>

PCI devices use RID-based mapping via the root complex `iommu-map`:

::

    <RID-base &viommu vSID-base length>

PCI Topology Assumptions and Constraints:

- RID space must be contiguous
- Pre-defined continuous pSID space (0-0x1000)
- No runtime PCI reconfiguration
- Single root complex assumed
- Mapping is fixed at guest DT construction

Constraints for PCI devices will be addressed as part of the future work on
this feature.

Security Considerations
=======================

Stage-1 translation provides isolation between guest devices by
enforcing a per-device I/O address space, preventing unauthorized DMA.
With the introduction of emulated IOMMU, additional protection
mechanisms are required to minimize security risks.

1. Observation:
---------------
Support for Stage-1 translation in SMMUv3 introduces new data structures 
(`s1_cfg` alongside `s2_cfg`)
and logic to write both Stage-1 and Stage-2 entries in the Stream Table 
Entry (STE), including an `abort`
field to handle partial configuration states.

**Risk:**
Without proper handling, a partially applied configuration
might leave guest DMA mappings in an inconsistent state, potentially
enabling unauthorized access or causing cross-domain interference.

**Mitigation:** *(Handled by design)*
This feature introduces logic that writes both `s1_cfg` and `s2_cfg` to
STE and manages the `abort` field - only considering
configuration if fully attached. This ensures  incomplete or invalid
device configurations are safely ignored by the hypervisor.

2. Observation:
---------------
Guests can now invalidate Stage-1 caches; invalidation needs forwarding
to SMMUv3 hardware to maintain coherence.

**Risk:**
Failing to propagate cache invalidation could allow stale mappings,
enabling access to old mappings and possibly
data leakage or misrouting between devices assigned to the same guest.

**Mitigation:**
The guest must issue appropriate invalidation commands whenever
its stage-1 I/O mappings are modified to ensure that translation caches
remain coherent.

3. Observation:
---------------
Introducing optional per-guest enabled features (`viommu` argument in xl 
guest config) means some guests
may opt-out.

**Risk:**
Guests without vIOMMU enabled (stage-2 only) could potentially dominate
access to the physical command and event queues, since they bypass the
emulation layer and processing is faster comparing to vIOMMU-enabled guests.

**Mitigation:**
Audit the impact of emulation overhead effect on IOMMU processing fairness
in a multi-guest environment.
Consider enabling/disabling stage-1 on a system level, instead of per-domain.

4. Observation:
---------------
Guests have the ability to issue Stage-1 IOMMU commands like cache 
invalidation, stream table entries
configuration, etc. An adversarial guest may issue a high volume of 
commands in rapid succession.

**Risk:**
Excessive commands requests can cause high hypervisor CPU consumption 
and disrupt scheduling,
leading to degraded system responsiveness and potential 
denial-of-service scenarios.

**Mitigation:**

- Implement vIOMMU commands execution restart and continuation support:

  - Introduce processing budget with only a limited amount of commands
    handled per invocation.
  - If additional commands remain pending after the budget is exhausted,
    defer further processing and resume it asynchronously, e.g. via a
    per-domain tasklet.

- Batch multiple commands of same type to reduce emulation overhead:

  - Inspect the command queue and group commands that can be processed
    together (e.g. multiple successive invalidation requests or STE
    updates for the same SID).
  - Execute the entire batch in one go, reducing repeated accesses to
    guest memory and emulation overhead per command.
  - This reduces CPU time spent in the vIOMMU command processing loop.
    The optimization is applicable only when consecutive commands of the
    same type operate on the same SID/context.

5. Observation:
---------------
Some guest commands issued towards vIOMMU are propagated to pIOMMU 
command queue (e.g. TLB invalidate).

**Risk:**
Excessive commands requests from abusive guest can cause flooding of 
physical IOMMU command queue,
leading to degraded pIOMMU responsiveness on commands issued from other 
guests.

**Mitigation:**

- Batch commands that are propagated to the pIOMMU command queue and
  implement batch execution pause/continuation.
  Rely on the same mechanisms as in the previous observation
  (command continuation and batching of pIOMMU-related commands of the same
  type and context).
- If possible, implement domain penalization by adding a per-domain budget
  for vIOMMU/pIOMMU usage:

  - Apply per-domain dynamic budgeting of allowed IOMMU commands to
    execute per invocation, reducing the budget for guests with
    excessive command requests over a longer period of time
  - Combine with command continuation mechanism

6. Observation:
---------------
The vIOMMU feature includes an event queue used to forward IOMMU events
to the guest (e.g. translation faults, invalid Stream IDs, permission errors).
A malicious guest may misconfigure its IOMMU state or intentionally trigger
faults at a high rate.

**Risk:**
Occurrence of IOMMU events with high frequency can cause Xen to flood the
event queue and disrupt scheduling with
high hypervisor CPU load for events handling.

**Mitigation:**

- Implement fail-safe state by disabling events forwarding when faults 
  are occurred with high frequency and
  not processed by guest:

  - Introduce a per-domain pending event counter.
  - Stop forwarding events to the guest once the number of unprocessed
    events reaches a predefined threshold.

- Consider disabling the emulated event queue for untrusted guests.
- Note that this risk is more general and may also apply to stage-2-only
  guests. This section addresses mitigations in the emulated IOMMU layer
  only. Mitigation of physical event queue flooding should also be considered
  in the target pIOMMU driver.

Performance Impact
==================

With iommu stage-1 and nested translation inclusion, performance 
overhead is introduced comparing to existing,
stage-2 only usage in Xen. Once mappings are established, translations 
should not introduce significant overhead.
Emulated paths may introduce moderate overhead, primarily affecting 
device initialization and event/command handling.
Testing is performed on Renesas R-Car platform.
Performance is mostly impacted by emulated vIOMMU operations, results 
shown in the following table.

+-------------------------------+---------------------------------+
| vIOMMU Operation              | Execution time in guest         |
+===============================+=================================+
| Reg read                      | median: 645ns, worst-case: 2us  |
+-------------------------------+---------------------------------+
| Reg write                     | median: 630ns, worst-case: 1us  |
+-------------------------------+---------------------------------+
| Invalidate TLB                | median: 2us, worst-case: 10us   |
+-------------------------------+---------------------------------+
| Invalidate STE                | median: 5us worst_case: 100us   |
+-------------------------------+---------------------------------+

With vIOMMU exposed to guest, guest OS has to initialize IOMMU device
and configure stage-1 mappings for the devices
attached to it.
Following table shows initialization stages which impact stage-1 enabled 
guest boot time and compares it with
stage-1 disabled guest.

NOTE: Device probe execution time varies depending on device complexity.
A USB host controller was selected as the test device in this case.

+---------------------+-----------------------+------------------------+
| Stage               | Stage-1 Enabled Guest | Stage-1 Disabled Guest |
+=====================+=======================+========================+
| IOMMU Init          | ~10ms                 | /                      |
+---------------------+-----------------------+------------------------+
| Dev Attach / Mapping| ~100ms                | ~90ms                  |
+---------------------+-----------------------+------------------------+

For devices configured with dynamic DMA mappings, DMA allocate/map/unmap 
operations performance is
also impacted on stage-1 enabled guests.
Dynamic DMA mapping operation trigger emulated IOMMU functions like mmio 
write/read and TLB invalidations.

+---------------+---------------------------+--------------------------+
| DMA Op        | Stage-1 Enabled Guest     | Stage-1 Disabled Guest   |
+===============+===========================+==========================+
| dma_alloc     | median: 20us, worst: 5ms  | median: 8us, worst: 60us |
+---------------+---------------------------+--------------------------+
| dma_free      | median: 500us, worst: 10ms| median: 6us, worst: 30us |
+---------------+---------------------------+--------------------------+
| dma_map       | median: 12us, worst: 60us | median: 3us, worst: 20us |
+---------------+---------------------------+--------------------------+
| dma_unmap     | median: 400us, worst: 5ms | median: 5us, worst: 20us |
+---------------+---------------------------+--------------------------+

Testing
=======

- QEMU-based ARM system tests for Stage-1 translation.
- Actual hardware validation to ensure compatibility with real SMMUv3 
implementations.
- Unit/Functional tests validating correct translations (not implemented).

Migration and Compatibility
===========================

This optional feature defaults to disabled (`viommu=""`) for backward 
compatibility.

Future improvements
===================

- Implement the proposed mitigations to address security risks that are 
  not covered by the current design
  (events batching, commands execution continuation)
- PCI support
- Support for other IOMMU HW (Renesas, RISC-V, etc.)

References
==========

- Original feature implemented by Rahul Singh:
  
https://patchwork.kernel.org/project/xen-devel/cover/cover.1669888522.git.rahul.singh@arm.com/ 

- SMMUv3 architecture documentation
- Existing vIOMMU code patterns (KVM, QEMU)