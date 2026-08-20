#!/usr/bin/env python3
"""
Remove the AP "Live readings" feature from src/main.cpp.

Removes, in order:
  1. the four /live and /api/live-reading route registrations
  2. handleLiveReadings()      (~18 KB page + 1 Hz polling JS)
  3. handleLiveReadingApi()    (JSON endpoint the page polled every second)
  4. both forward declarations
  5. the nav tab markup on the configuration page
  6. the live-only CSS block (~2.5 KB of every page's <style>)

Deliberately NOT removed: the LatestLiveReading cache and the assignments to it
inside publishData(). Those are a handful of struct field writes with no network
or HTTP cost, and leaving them keeps publishData() byte-identical to the version
known to acquire correctly. They are dead code and can be stripped later.

Every removal is asserted, so a partial edit fails loudly instead of silently
producing a broken file.
"""
import re
import sys
from pathlib import Path

p = Path("src/main.cpp")
text = p.read_text(encoding="utf-8")
orig_len = len(text)
done = []


def cut(label, needle, expect=1):
    """Remove exact substring `needle`, expecting `expect` occurrences."""
    global text
    n = text.count(needle)
    if n != expect:
        sys.exit(f"FAIL [{label}]: expected {expect} occurrence(s), found {n}")
    text = text.replace(needle, "", expect)
    done.append(f"{label} ({expect}x)")


def cut_function(label, signature):
    """Remove a whole top-level function definition starting at `signature`."""
    global text
    lines = text.split("\n")
    starts = [i for i, l in enumerate(lines) if l == signature]
    if len(starts) != 1:
        sys.exit(f"FAIL [{label}]: found {len(starts)} definitions of {signature!r}")
    i = starts[0]
    end = None
    for j in range(i, len(lines)):
        if lines[j] == "}":
            end = j
            break
    if end is None:
        sys.exit(f"FAIL [{label}]: no closing brace for {signature!r}")
    removed = end - i + 1
    # also swallow one trailing blank line if present
    if end + 1 < len(lines) and lines[end + 1] == "":
        end += 1
    del lines[i:end + 1]
    text = "\n".join(lines)
    done.append(f"{label} ({removed} lines)")


# 1. routes ------------------------------------------------------------------
cut("route /live",              '    server.on("/live", HTTP_GET, handleLiveReadings);\n')
cut("route /live/",             '    server.on("/live/", HTTP_GET, handleLiveReadings);\n')
cut("route /api/live-reading",  '    server.on("/api/live-reading", HTTP_GET, handleLiveReadingApi);\n')
cut("route /api/live-reading/", '    server.on("/api/live-reading/", HTTP_GET, handleLiveReadingApi);\n')

# 2/3. handler definitions ---------------------------------------------------
cut_function("handleLiveReadings()",   "void handleLiveReadings() {")
cut_function("handleLiveReadingApi()", "void handleLiveReadingApi() {")

# 4. forward declarations ----------------------------------------------------
cut("forward decl handleLiveReadings",   "void handleLiveReadings();\n")
cut("forward decl handleLiveReadingApi", "void handleLiveReadingApi();\n")

# 5. nav tabs on the configuration page --------------------------------------
nav = ("  html += \"<nav class='tabs' aria-label='Setup pages'>"
       "<a class='tab active' href='/config' aria-current='page'>Configuration</a>"
       "<a class='tab' href='/live'>Live readings</a></nav>\";\n")
cut("configuration page nav tabs", nav)

# 6. live-only CSS -----------------------------------------------------------
lines = text.split("\n")
css = [i for i, l in enumerate(lines) if l.lstrip().startswith('".page{padding:18px 20px}.live-lead')]
if len(css) != 1:
    sys.exit(f"FAIL [live CSS]: found {len(css)} candidate lines")
css_bytes = len(lines[css[0]])
del lines[css[0]]
text = "\n".join(lines)
done.append(f"live-only CSS block ({css_bytes} bytes off every page)")

p.write_text(text, encoding="utf-8")

print(f"src/main.cpp: {orig_len} -> {len(text)} bytes  ({orig_len - len(text)} removed)\n")
for i, d in enumerate(done, 1):
    print(f"  {i:2d}. {d}")

# --- guards -----------------------------------------------------------------
leftovers = [n for n in ("handleLiveReadings", "handleLiveReadingApi",
                         "/api/live-reading", "href='/live'") if n in text]
if leftovers:
    sys.exit(f"\nFAIL: live-tab references still present: {leftovers}")
print("\nGuard OK: no live-tab routes, handlers or links remain.")

if "latestLiveReading" not in text:
    sys.exit("FAIL: publishData() cache writes were removed - that was not intended.")
print("Guard OK: publishData() cache writes left untouched (dead but harmless).")
