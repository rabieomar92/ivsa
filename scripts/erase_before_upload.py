"""Force a full-chip erase as part of every normal serial firmware upload."""

from SCons.Script import COMMAND_LINE_TARGETS

Import("env")


# A filesystem-only upload must never erase the application image. Normal
# PlatformIO firmware uploads use the `upload` target and esptool's write_flash
# command, where --erase-all must appear after the subcommand.
if not ({"uploadfs", "uploadfsota"} & set(COMMAND_LINE_TARGETS)):
    uploader_flags = list(env.get("UPLOADERFLAGS", []))
    if "write_flash" in uploader_flags and "--erase-all" not in uploader_flags:
        write_flash_index = uploader_flags.index("write_flash")
        uploader_flags.insert(write_flash_index + 1, "--erase-all")
        env.Replace(UPLOADERFLAGS=uploader_flags)
        print("[upload] Full 4 MB flash erase is enabled before programming.")
