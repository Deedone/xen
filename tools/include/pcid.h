/*
    Common definitions for Xen PCI client-server protocol.
    Copyright (C) 2021 EPAM Systems Inc.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PCID_H
#define PCID_H

#define PCID_SRV_NAME           "pcid"
#define PCID_XS_TOKEN           "pcid-token"

#define PCI_RECEIVE_BUFFER_SIZE 4096
#define PCI_MAX_SIZE_RX_BUF     MB(1)

/*
 *******************************************************************************
 * Common request and response structures used be the pcid remote protocol are
 * described below.
 *******************************************************************************
 * Request:
 * +-------------+--------------+----------------------------------------------+
 * | Field       | Type         | Comment                                      |
 * +-------------+--------------+----------------------------------------------+
 * | cmd         | string       | String identifying the command               |
 * +-------------+--------------+----------------------------------------------+
 *
 * Response:
 * +-------------+--------------+----------------------------------------------+
 * | Field       | Type         | Comment                                      |
 * +-------------+--------------+----------------------------------------------+
 * | resp        | string       | Command string as in the request             |
 * +-------------+--------------+----------------------------------------------+
 * | error       | string       | "okay", "failed"                               |
 * +-------------+--------------+----------------------------------------------+
 * | error_desc  | string       | Optional error description string            |
 * +-------------+--------------+----------------------------------------------+
 *
 * Notes.
 * 1. Every request and response must contain the above mandatory structures.
 * 2. In case if a bad packet or an unknown command received by the server side
 * a valid reply with the corresponding error code must be sent to the client.
 *
 * Requests and responses, which require SBDF as part of their payload, must
 * use the following convention for encoding SBDF value:
 *
 * pci_device object:
 * +-------------+--------------+----------------------------------------------+
 * | Field       | Type         | Comment                                      |
 * +-------------+--------------+----------------------------------------------+
 * | sbdf        | string       | SBDF string in form SSSS:BB:DD.F             |
 * +-------------+--------------+----------------------------------------------+
 */

#define PCID_MSG_FIELD_CMD      "cmd"

#define PCID_MSG_FIELD_RESP     "resp"
#define PCID_MSG_FIELD_ERR      "error"
#define PCID_MSG_FIELD_ERR_DESC "error_desc"

/* pci_device object fields. */
#define PCID_MSG_FIELD_SBDF     "sbdf"

#define PCID_MSG_ERR_OK         "okay"
#define PCID_MSG_ERR_FAILED     "failed"
#define PCID_MSG_ERR_NA         "NA"

#define PCID_SBDF_FMT           "%04x:%02x:%02x.%01x"

/*
 *******************************************************************************
 * List assignable devices
 *
 * This command lists PCI devices that can be passed through to a guest domain.
 *
 * Request (see other mandatory fields above):
 *  - "cmd" field of the request must be set to "list_assignable".
 *
 * Response (see other mandatory fields above):
 *  - "resp" field of the response must be set to "list_assignable".
 * Command specific response data:
 * +-------------+--------------+----------------------------------------------+
 * | devices     | list         | List of of pci_device objects                |
 * +-------------+--------------+----------------------------------------------+
 */
#define PCID_CMD_LIST_ASSIGNABLE        "list_assignable"
#define PCID_MSG_FIELD_DEVICES          "devices"

/*
 *******************************************************************************
 * Make device assignable
 *
 * This command makes given device assignable by ensuring that OS
 * will not try to access it.
 *
 * Request (see other mandatory fields above):
 *  - "cmd" field of the request must be set to "make_assignable".
 *  - "sbdf" SBDF of the device in format defined by PCID_SBDF_FMT.
 *  - "rebind" = true if daemon needs to save original driver name,
 *    so device later can be rebound back.
 *
 * Response (see other mandatory fields above):
 *  - "resp" field of the response must be set to "make_assignable".
 */
#define PCID_CMD_MAKE_ASSIGNABLE        "make_assignable"
#define PCID_MSG_FIELD_REBIND           "rebind"

/*
 *******************************************************************************
 * Revert device from assignable state
 *
 * This command reverts effect of "make_assignable" command. Basically,
 * now device can be used by OS again.
 *
 * Request (see other mandatory fields above):
 *  - "cmd" field of the request must be set to "revert_assignable".
 *  - "sbdf" SBDF of the device in format defined by PCID_SBDF_FMT.
 *  - "rebind" = true if daemon needs to rebind device back to it's
 *    original driver, which name was saved by "make_assignable" command
 *
 * Response (see other mandatory fields above):
 *  - "resp" field of the response must be set to "revert_assignable".
 */
#define PCID_CMD_REVERT_ASSIGNABLE      "revert_assignable"

/*
 *******************************************************************************
 * Check is device assigned
 *
 * This command checks device is assigned
 *
 * Request (see other mandatory fields above):
 *  - "cmd" field of the request must be set to "is_device_assigned".
 *  - "sbdf" SBDF of the device in format defined by PCID_SBDF_FMT.
 *
 * Response (see other mandatory fields above):
 *  - "resp" field of the response must be set to "is_device_assigned".
 * Command specific response data:
 * +-------------+--------------+----------------------------------------------+
 * | result      | bool         | true if device assigned                      |
 * +-------------+--------------+----------------------------------------------+
 */
#define PCID_CMD_IS_ASSIGNED            "is_device_assigned"
#define PCID_MSG_FIELD_RESULT           "result"

/*
 *******************************************************************************
 * Get device resources
 *
 * This command returns resource list of device
 *
 * Request (see other mandatory fields above):
 *  - "cmd" field of the request must be set to "resource_list".
 *  - "sbdf" SBDF of the device in format defined by PCID_SBDF_FMT.
 *
 * Response (see other mandatory fields above):
 *  - "resp" field of the response must be set to "resource_list".
 * Command specific response data:
 * +-------------+--------------+----------------------------------------------+
 * | resources   | map          | key 'iomem' - list of memory regions         |
 * |             |              | key 'irqs' - list of irqs                    |
 * +-------------+--------------+----------------------------------------------+
 */
#define PCID_CMD_RESOURCE_LIST          "resource_list"
/* Arguments */
#define PCID_MSG_FIELD_DOMID            "domid"
/* Result */
#define PCID_MSG_FIELD_RESOURCES        "resources"
#define PCID_RESULT_KEY_IOMEM           "iomem"
#define PCID_RESULT_KEY_IRQS            "irqs"

/*
 *******************************************************************************
 * Write BDF values to the pciback sysfs path
 *
 * This command resets PCI device
 *
 * Request (see other mandatory fields above):
 *  - "cmd" field of the request must be set to "write_bdf".
 *  - "sbdf" SBDF of the device in format defined by PCID_SBDF_FMT.
 *  - "name" name of sysfs file of pciback driver
 *
 * Response (see other mandatory fields above):
 *  - "resp" field of the response must be set to "write_bdf".
 */
#define PCID_CMD_WRITE_BDF               "write_bdf"
#define PCID_MSG_FIELD_NAME              "name"

/*
 *******************************************************************************
 * Reset PCI device
 *
 * This command resets PCI device
 *
 * Request (see other mandatory fields above):
 *  - "cmd" field of the request must be set to "reset_device".
 *  - "sbdf" SBDF of the device in format defined by PCID_SBDF_FMT.
 *
 * Response (see other mandatory fields above):
 *  - "resp" field of the response must be set to "reset_device".
 */
#define PCID_CMD_RESET_DEVICE            "reset_device"

int libxl_pcid_process(libxl_ctx *ctx);

#endif /* PCID_H */

/*
 * Local variables:
 *  mode: C
 *  c-file-style: "linux"
 *  indent-tabs-mode: t
 *  c-basic-offset: 8
 *  tab-width: 8
 * End:
 */