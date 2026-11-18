"""Inspect X11 ancestry in an isolated process without blocking the Qt event loop."""

import argparse
import ctypes
import ctypes.util
import json
import time
from collections.abc import Iterator
from contextlib import contextmanager


class _WindowAttributes(ctypes.Structure):
    """XSetWindowAttributes layout from Xlib; Bool and gravities are C integers."""

    _fields_ = [
        ("background_pixmap", ctypes.c_ulong),
        ("background_pixel", ctypes.c_ulong),
        ("border_pixmap", ctypes.c_ulong),
        ("border_pixel", ctypes.c_ulong),
        ("bit_gravity", ctypes.c_int),
        ("win_gravity", ctypes.c_int),
        ("backing_store", ctypes.c_int),
        ("backing_planes", ctypes.c_ulong),
        ("backing_pixel", ctypes.c_ulong),
        ("save_under", ctypes.c_int),
        ("event_mask", ctypes.c_long),
        ("do_not_propagate_mask", ctypes.c_long),
        ("override_redirect", ctypes.c_int),
        ("colormap", ctypes.c_ulong),
        ("cursor", ctypes.c_ulong),
    ]


@contextmanager
def _display_connection() -> Iterator[tuple[ctypes.CDLL, int]]:
    library = ctypes.CDLL(ctypes.util.find_library("X11") or "libX11.so.6")
    library.XOpenDisplay.argtypes = [ctypes.c_char_p]
    library.XOpenDisplay.restype = ctypes.c_void_p
    library.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = library.XOpenDisplay(None)
    if not display:
        raise RuntimeError("X11 display is unavailable")
    try:
        yield library, display
    finally:
        library.XCloseDisplay(display)


def window_ancestry(window_id: int) -> list[int]:
    """Return native ancestors, releasing every X11 allocation and display connection."""
    with _display_connection() as (library, display):
        return _read_ancestry(library, display, window_id)


def _read_ancestry(library: ctypes.CDLL, display: int, window_id: int) -> list[int]:
    window = ctypes.c_ulong
    window_pointer = ctypes.POINTER(window)
    library.XQueryTree.argtypes = [
        ctypes.c_void_p,
        window,
        window_pointer,
        window_pointer,
        ctypes.POINTER(window_pointer),
        ctypes.POINTER(ctypes.c_uint),
    ]
    library.XQueryTree.restype = ctypes.c_int
    library.XFree.argtypes = [ctypes.c_void_p]
    ancestors = []
    current = window_id
    while current and current not in ancestors and len(ancestors) < 32:
        ancestors.append(current)
        root, parent = window(), window()
        children = window_pointer()
        count = ctypes.c_uint()
        succeeded = library.XQueryTree(
            display,
            current,
            ctypes.byref(root),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(count),
        )
        if children:
            library.XFree(children)
        if not succeeded:
            raise RuntimeError("Native window ancestry is unavailable")
        current = parent.value
    return ancestors


def detach_window(window_id: int) -> None:
    """Hide and detach an independent view before its Qt host is destroyed."""
    with _display_connection() as (library, display):
        library.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        library.XDefaultRootWindow.restype = ctypes.c_ulong
        library.XReparentWindow.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_int,
        ]
        library.XUnmapWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        library.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        root = library.XDefaultRootWindow(display)
        library.XUnmapWindow(display, window_id)
        library.XReparentWindow(display, window_id, root, 0, 0)
        library.XSync(display, 0)


def attach_window(window_id: int, parent_id: int) -> None:
    """Withdraw a managed top-level window before embedding it as an unmanaged child.

    Desktop window managers can otherwise reclaim a foreign Qt window after reparenting.
    This runs in a separate bounded process, never on the presentation event loop.
    """
    with _display_connection() as (library, display):
        library.XDefaultScreen.argtypes = [ctypes.c_void_p]
        library.XDefaultScreen.restype = ctypes.c_int
        library.XWithdrawWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int]
        library.XChangeWindowAttributes.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.POINTER(_WindowAttributes),
        ]
        library.XReparentWindow.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_int,
        ]
        library.XMapWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        library.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        library.XWithdrawWindow(display, window_id, library.XDefaultScreen(display))
        library.XSync(display, 0)
        deadline = time.monotonic() + 1
        while len(_read_ancestry(library, display, window_id)) > 2:
            if time.monotonic() >= deadline:
                raise TimeoutError("Window manager did not release the independent viewer")
            time.sleep(0.02)
        attributes = _WindowAttributes(override_redirect=1)
        library.XChangeWindowAttributes(display, window_id, 1 << 9, ctypes.byref(attributes))
        library.XReparentWindow(display, window_id, parent_id, 0, 0)
        library.XMapWindow(display, window_id)
        library.XSync(display, 0)
        if parent_id not in _read_ancestry(library, display, window_id):
            raise RuntimeError("Viewer attachment was not observed after reparenting")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("window_id", type=int)
    parser.add_argument("host_id", type=int, nargs="?")
    operation = parser.add_mutually_exclusive_group()
    operation.add_argument("--detach", action="store_true")
    operation.add_argument("--attach", action="store_true")
    arguments = parser.parse_args()
    if arguments.detach:
        detach_window(arguments.window_id)
        return
    if arguments.host_id is None:
        parser.error("A host ID is required for ancestry verification")
    if arguments.attach:
        attach_window(arguments.window_id, arguments.host_id)
        return
    ancestors = window_ancestry(arguments.window_id)
    print(json.dumps({"attached": arguments.host_id in ancestors, "ancestors": ancestors}))


if __name__ == "__main__":
    main()
