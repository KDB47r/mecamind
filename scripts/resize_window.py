#!/usr/bin/env python3
"""Resize an X11 window matched by name regex."""
import re
import sys

from Xlib import display


def wm_name(win):
    try:
        name = win.get_wm_name()
        if isinstance(name, bytes):
            name = name.decode("utf-8", "replace")
        return name or ""
    except Exception:
        return ""


def iter_windows(win, depth=0):
    yield win
    if depth > 6:
        return
    try:
        for child in win.query_tree().children:
            yield from iter_windows(child, depth + 1)
    except Exception:
        return


pattern, w, h = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
d = display.Display(":0")
root = d.screen().root
best = None
for win in iter_windows(root):
    name = wm_name(win)
    if name and re.search(pattern, name, re.I):
        try:
            geo = win.get_geometry()
        except Exception:
            continue
        if geo.width < 200:
            continue
        if best is None or geo.width * geo.height > best[1].width * best[1].height:
            best = (win, geo, name)
if best is None:
    print("NO_MATCH", file=sys.stderr)
    sys.exit(1)
win, geo, name = best
win.configure(width=w, height=h)
d.sync()
print(f"RESIZED '{name}' -> {w}x{h}")
