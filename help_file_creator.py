Import("env")
import re
import os

# Width of the command column in the output file
COMMAND_WIDTH = 20

COMMENT_PATTERN     = re.compile(r'^\s*//\s*(.*)')
EXTENDED_PATTERN    = re.compile(r'^\s+//\s*(.*)')
INSTRUCTION_PATTERN = re.compile(r'\{\s*"(\w+)"[^/]*//\s*(.*)')


def _scan_file(src_path, outfile, converted_count):
    """Scan one .cpp file for command blocks and write matching lines to outfile.
    Returns the updated converted_count."""
    start_flag = False
    try:
        with open(src_path, 'r', encoding='utf-8', errors='replace') as infile:
            for line in infile:
                if line.strip() == "// Start of command block":
                    start_flag = True
                    continue

                if not start_flag:
                    continue

                if line.strip() == "// End of table marker":
                    break

                # Indented comment — extended description for the previous command
                extended_match = EXTENDED_PATTERN.search(line)
                if extended_match:
                    comment = extended_match.group(1).strip()
                    outfile.write(f"{' ':<{COMMAND_WIDTH}}{comment}\n")
                    converted_count += 1
                    continue

                # Top-level comment — section header or free text
                comment_match = COMMENT_PATTERN.match(line)
                if comment_match:
                    outfile.write(comment_match.group(1).strip() + "\n")
                    continue

                # Command table entry:  {"CMDNAME", ..., // description}
                instruction_match = INSTRUCTION_PATTERN.search(line)
                if instruction_match:
                    command     = "  " + instruction_match.group(1).strip()
                    description = instruction_match.group(2).strip()
                    outfile.write(f"{command:<{COMMAND_WIDTH}}{description}\n")
                    converted_count += 1

    except Exception as e:
        print(f"help_file_creator: error processing {src_path}: {type(e).__name__}: {e}")

    return converted_count


def generate_help_file(source, target, env):  # source/target unused; called at config time
    src_dir     = env.subst("$PROJECT_SRC_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    output_path = os.path.join(project_dir, "MIPScommands.txt")

    # Collect all .cpp files recursively
    cpp_files = []
    for root, dirs, files in os.walk(src_dir):
        # Skip any hidden or build subdirectories
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        for fname in files:
            if fname.endswith('.cpp'):
                cpp_files.append(os.path.join(root, fname))

    # Sort alphabetically, but always put Serial.cpp first — it holds the
    # primary command table and should appear at the top of the output.
    def sort_key(path):
        name = os.path.basename(path)
        return (0 if name == "Serial.cpp" else 1, name.lower())

    cpp_files.sort(key=sort_key)

    total_converted = 0
    try:
        with open(output_path, 'w', encoding='utf-8') as outfile:
            for cpp_file in cpp_files:
                total_converted = _scan_file(cpp_file, outfile, total_converted)
    except IOError as e:
        print(f"help_file_creator: could not write {output_path}: {e}")
        return

    print("")
    print("=" * 50)
    print(f"  Help file:     MIPScommands.txt")
    print(f"  Commands:      {total_converted}")
    print(f"  Files scanned: {len(cpp_files)}")
    print("=" * 50)
    print("")


generate_help_file(None, None, env)
