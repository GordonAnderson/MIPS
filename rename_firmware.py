Import("env")
import re
import os
import shutil

# Feature flags defined in Variants.h and the letter each appends to the version
FLAGS = [
    ("FAIMSFBcode",   "b"),
    ("FAIMScode",     "f"),
    ("HOFAIMcode",    "h"),
    ("HVPScode",      "v"),
    ("DMSDMSMB",      "d"),
    ("DCBanalog",     "a"),
    ("DCBcurrent",    "c"),
    ("DCBswitchCode", "s"),
    ("HVPSinterface", "i"),
]


def rename_firmware(source, target, env):
    include_dir = env.subst("$PROJECT_INCLUDE_DIR")
    src_dir     = env.subst("$PROJECT_SRC_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    variants_h  = os.path.join(include_dir, "Variants.h")
    mips_cpp    = os.path.join(src_dir, "MIPS.cpp")

    # Pull base version number from MIPS.cpp  e.g. "Version 1.263"
    base_version = "Version_unknown"
    try:
        with open(mips_cpp) as f:
            for line in f:
                m = re.search(r'char\s+Version\[\].*?"(Version\s+[\d.]+)"', line)
                if m:
                    base_version = m.group(1).replace(" ", "")
                    break
    except Exception as e:
        print(f"rename_firmware: could not read MIPS.cpp: {e}")

    # Determine which feature flags are enabled
    suffix = ""
    try:
        with open(variants_h) as f:
            content = f.read()
        for name, letter in FLAGS:
            if re.search(r'#define\s+' + name + r'\s+true\b', content):
                suffix += letter
    except Exception as e:
        print(f"rename_firmware: could not read Variants.h: {e}")

    built_bin = str(target[0])
    out_dir   = os.path.join(project_dir, "firmware")
    out_name  = f"MIPS_{base_version}{suffix}.bin"
    out_path  = os.path.join(out_dir, out_name)

    try:
        os.makedirs(out_dir, exist_ok=True)
        shutil.copy(built_bin, out_path)
        print("")
        print("=" * 50)
        print(f"  Firmware copied to: firmware/{out_name}")
        print("=" * 50)
        print("")
    except Exception as e:
        print(f"rename_firmware: could not copy firmware: {e}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", rename_firmware)
