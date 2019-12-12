#!/usr/bin/env python3
# Patches a firmware binary to hold the right checksum
# Checksum is a 32bit filed at offset 0x1C, whereas
# firmware size is stored at 0x20 (little endian, words)

import sys, struct

fwbin = open(sys.argv[1], "rb").read()

# Ensure the firmware is word size aligned
print("Firmware size", len(fwbin))
while len(fwbin) % 4 != 0:
	fwbin += b"\x00"
print("Firmware size after padding", len(fwbin))

# Patch 0x1C with zero, 0x20 with the FW size too
sizestr = struct.pack("<I", len(fwbin) // 4)
fwbin = fwbin[:0x1C] + b"\x00\x00\x00\x00" + sizestr + fwbin[0x24:]

# Calculate the checksum, whole file with padding
xorv = 0
for i in range(0, len(fwbin), 4):
	xorv ^= struct.unpack("<I", fwbin[i:i+4])[0]

# Pack everything
xorv = struct.pack("<I", xorv)
fwbin = fwbin[:0x1C] + xorv + fwbin[0x20:]

# Overwrite firmware file
open(sys.argv[1], "wb").write(fwbin)

