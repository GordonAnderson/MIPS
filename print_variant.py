Import("env")
import re, os

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

include_dir = env.subst("$PROJECT_INCLUDE_DIR")
src_dir     = env.subst("$PROJECT_SRC_DIR")
variants_h  = os.path.join(include_dir, "Variants.h")
mips_cpp    = os.path.join(src_dir, "MIPS.cpp")

# Pull base version number from MIPS.cpp  e.g. "Version 1.262"
base_version = "Version ?"
try:
    with open(mips_cpp) as f:
        for line in f:
            m = re.search(r'char\s+Version\[\].*?"(Version\s+[\d.]+)"', line)
            if m:
                base_version = m.group(1)
                break
except Exception as e:
    print("print_variant.py: could not read MIPS.cpp: " + str(e))

# Determine which feature flags are enabled
enabled = []
try:
    with open(variants_h) as f:
        content = f.read()
    for name, letter in FLAGS:
        if re.search(r'#define\s+' + name + r'\s+true\b', content):
            enabled.append((name, letter))
except Exception as e:
    print("print_variant.py: could not read Variants.h: " + str(e))

suffix  = "".join(l for _, l in enabled)
modules = ", ".join(n for n, _ in enabled) if enabled else "none"

print("")
print("=" * 50)
print("  MIPS variant:  " + base_version + suffix)
print("  Modules:       " + modules)
print("=" * 50)
print("")
