"""Generate src/unicode_c_ranges.inc: the code point ranges in Unicode general category C*
(as Python unicodedata reports them), used to clean STEP names as Python import_step does.

    python scripts/gen_unicode_c_ranges.py src/unicode_c_ranges.inc
"""
import sys, unicodedata
ranges = []
start = None
for cp in range(0x110000):
    is_c = unicodedata.category(chr(cp))[0] == "C"
    if is_c and start is None:
        start = cp
    elif not is_c and start is not None:
        ranges.append((start, cp - 1)); start = None
if start is not None:
    ranges.append((start, 0x10FFFF))
out = ["// Generated from Python %s unicodedata (Unicode %s): code point ranges whose" % (sys.version.split()[0], unicodedata.unidata_version),
       "// general category starts with 'C' (Cc, Cf, Cs, Co, Cn). See import_step's get_name.",
       "// clang-format off"]
line = ""
for a, b in ranges:
    item = "{0x%X, 0x%X}," % (a, b)
    if len(line) + len(item) + 1 > 100:
        out.append(line.rstrip()); line = ""
    line += item + " "
if line: out.append(line.rstrip())
out.append("// clang-format on")
open(sys.argv[1], "w").write("\n".join(out) + "\n")
print(len(ranges))
