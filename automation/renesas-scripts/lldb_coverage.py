import lldb
import sys
import os
import struct
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)-8s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

log = logging.getLogger(__name__)

executed_addresses = set()

def parse_drcov_v2(input_path):
    modules = {}
    bb_count = 0

    with open(input_path, 'rb') as f:
        for line in f:
            if line.startswith(b"Module Table:"):
                break

        for line in f:
            if line.startswith(b"BB Table:"):
                bb_count = int(line.split()[2])
                break

            if line.startswith(b"Columns:"):
                continue

            parts = line.decode('utf-8').split(',')
            if len(parts) >= 3:
                mod_id = int(parts[0])
                mod_base = int(parts[1], 16)

                modules[mod_id] = mod_base

        log.info(f"Parsed {len(modules)} modules. Reading {bb_count} basic blocks...")

        bb_data = f.read(bb_count * 8)

        # struct.iter_unpack('<IHH') reads the data as an array of 8-byte structs:
        # '<'  = Little Endian
        # 'I'  = uint32 (start offset)
        # 'H'  = uint16 (block size)
        # 'H'  = uint16 (module id)
        for bb_start, bb_size, mod_id in struct.iter_unpack('<IHH', bb_data):
            abs_addr = modules[mod_id] + bb_start

            for instr_addr in range(abs_addr, abs_addr + bb_size, 4):
                log.debug(f"Add executed instr_pc: 0x{instr_addr:x}")
                executed_addresses.add(instr_addr)

coverage_results = {}

def add_coverage_line(file_spec, line, is_hit):
    if not file_spec.IsValid() or line == 0:
        return

    directory = file_spec.GetDirectory()
    filename = file_spec.GetFilename()

    if directory:
        full_path = os.path.normpath(os.path.join(directory, filename))
    else:
        full_path = filename

    if full_path not in coverage_results:
        coverage_results[full_path] = {}

    if line not in coverage_results[full_path]:
        coverage_results[full_path][line] = False

    if is_hit:
        coverage_results[full_path][line] = True

def process_inlines(target, start_addr, is_hit):
    """
    Traverses the DWARF inline stack for a given address and records
    execution hits for all parent call site lines.
    """
    addr = target.ResolveFileAddress(start_addr)
    if not addr.IsValid():
        return

    block = addr.GetBlock()
    if not block.IsValid():
        return

    inlined_block = block.GetContainingInlinedBlock()

    while inlined_block.IsValid():
        file_spec = inlined_block.GetInlinedCallSiteFile()
        line = inlined_block.GetInlinedCallSiteLine()

        add_coverage_line(file_spec, line, is_hit)

        parent_block = inlined_block.GetParent()
        if parent_block.IsValid():
            inlined_block = parent_block.GetContainingInlinedBlock()
        else:
            break

def generate_lcov_report(elf_file, lcov_report):
    debugger = lldb.SBDebugger.Create()

    target = debugger.CreateTarget(elf_file)

    if not target.IsValid():
        log.error(f"Error: Failed to find target file: {elf_file}.")
        os._exit(1)

    module = target.GetModuleAtIndex(0)
    num_compile_units = module.GetNumCompileUnits()

    for i in range(num_compile_units):
        compile_unit = module.GetCompileUnitAtIndex(i)

        for j in range(compile_unit.GetNumLineEntries()):
            line_entry = compile_unit.GetLineEntryAtIndex(j)
            line_num = line_entry.GetLine()

            if (line_num == 0):
                continue

            start_addr = line_entry.GetStartAddress().GetFileAddress()
            end_addr = line_entry.GetEndAddress().GetFileAddress()

            if start_addr == 0 or start_addr == 0xffffffffffffffff or start_addr == end_addr:
                continue

            file_spec = line_entry.GetFileSpec()

            is_hit = False
            for instr_pc in range(start_addr, end_addr, 4):
                if instr_pc in executed_addresses:
                    log.debug(f"Select line {file_spec.GetFilename()}:{line_num} executed by instr_pc: 0x{instr_pc:x}")
                    is_hit = True
                    break

            add_coverage_line(file_spec, line_num, is_hit)

            process_inlines(target, start_addr, is_hit)

    log.info(f"Writing report to {lcov_report}...")

    with open(lcov_report, 'w') as out_file:
        for file_name, lines in coverage_results.items():
            out_file.write(f"SF:{file_name}\n")

            for line_num, is_hit in sorted(lines.items()):
                out_file.write(f"DA:{line_num},{1 if is_hit else 0}\n")

            out_file.write("end_of_record\n")

    log.info("Done")

drcov_file_path = os.environ.get("COV_INPUT", "drcov.trace")
parse_drcov_v2(drcov_file_path)

elf_path = os.environ.get("ELF", "xen-syms")
lcov_report = os.environ.get("LCOV_OUT", "coverage.info")

generate_lcov_report(elf_path, lcov_report)
