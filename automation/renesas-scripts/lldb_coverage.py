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
            directory = file_spec.GetDirectory()
            filename = file_spec.GetFilename()

            if directory:
                file_name = os.path.normpath(os.path.join(directory, filename))
            else:
                file_name = filename

            if file_name not in coverage_results:
                coverage_results[file_name] = {}
            if line_num not in coverage_results[file_name]:
                coverage_results[file_name][line_num] = {"ranges": []}

            column = line_entry.GetColumn()

            range_was_hit = False
            for instr_pc in range(start_addr, end_addr, 4):
                if instr_pc in executed_addresses:
                    log.debug(f"Select line {file_name}:{line_num} executed by instr_pc: 0x{instr_pc:x}")
                    range_was_hit = True
                    break

            coverage_results[file_name][line_num]["ranges"].append({
                "hit": 1 if range_was_hit else 0,
                "col": column,
                "start": start_addr,
                "end": end_addr
            })

    log.info(f"Writing report to {lcov_report}...")

    with open(lcov_report, 'w') as out_file:
        for file_name, lines in coverage_results.items():
            out_file.write(f"SF:{file_name}\n")

            for line_num, stats in sorted(lines.items()):
                ranges = stats["ranges"]
                total = len(ranges)

                if total == 0:
                    continue

                total_line_hits = sum(r["hit"] for r in ranges)

                out_file.write(f"DA:{line_num},{1 if total_line_hits > 0 else 0}\n")

                if total == 1:
                    continue

                for hit_id, r in enumerate(ranges):
                    col_str = f"col_{r['col']}"
                    hover_text = f"{col_str}_[0x{r['start']:x}-0x{r['end']:x})"

                    out_file.write(f"BRDA:{line_num},0,{hover_text},{r['hit']}\n")

            out_file.write("end_of_record\n")

    log.info("Done")

drcov_file_path = os.environ.get("COV_INPUT", "drcov.trace")
parse_drcov_v2(drcov_file_path)

elf_path = os.environ.get("XEN_ELF", "xen-syms")
lcov_report = os.environ.get("LCOV_OUT", "coverage.info")

generate_lcov_report(elf_path, lcov_report)
