#!/usr/bin/env python3
"""Run local tests/simulation with kernel-enforced network denial on Linux.

No robot SDK is imported. The seccomp filter also applies to native libraries
and survives fork/exec into worker Python environments. Only Unix-domain socket
creation is allowed, for local IPC. This is network isolation, not a filesystem
or serial/USB-device sandbox. Failure to install the filter aborts the command.
"""
from __future__ import annotations

import ctypes
import errno
import os
from pathlib import Path
import socket
import sys


def block_network():
    if sys.platform != "linux":
        raise RuntimeError("Network-isolated execution requires Linux and libseccomp")
    lib = ctypes.CDLL("libseccomp.so.2", use_errno=True)

    class Comparison(ctypes.Structure):
        _fields_ = [("arg", ctypes.c_uint), ("op", ctypes.c_uint),
                    ("datum_a", ctypes.c_uint64), ("datum_b", ctypes.c_uint64)]

    lib.seccomp_init.argtypes = [ctypes.c_uint32]
    lib.seccomp_init.restype = ctypes.c_void_p
    lib.seccomp_syscall_resolve_name.argtypes = [ctypes.c_char_p]
    lib.seccomp_syscall_resolve_name.restype = ctypes.c_int
    lib.seccomp_rule_add_array.argtypes = [ctypes.c_void_p, ctypes.c_uint32,
                                         ctypes.c_int, ctypes.c_uint,
                                         ctypes.POINTER(Comparison)]
    lib.seccomp_rule_add_array.restype = ctypes.c_int
    lib.seccomp_load.argtypes = [ctypes.c_void_p]
    lib.seccomp_load.restype = ctypes.c_int
    lib.seccomp_release.argtypes = [ctypes.c_void_p]
    lib.seccomp_release.restype = None
    context = lib.seccomp_init(0x7FFF0000)  # SCMP_ACT_ALLOW
    if not context:
        raise RuntimeError("Could not allocate seccomp filter")
    try:
        # SCMP_CMP_NE = 1. Reject all network families, not just IPv4/IPv6.
        nonlocal_socket = Comparison(0, 1, socket.AF_UNIX, 0)
        for name, comparison in ((b"socket", nonlocal_socket),
                                 (b"socketpair", nonlocal_socket),
                                 (b"io_uring_setup", None)):
            number = lib.seccomp_syscall_resolve_name(name)
            if number < 0:
                raise RuntimeError(f"Cannot resolve syscall {name!r}")
            result = lib.seccomp_rule_add_array(
                context, 0x00050000 | errno.EPERM, number,
                int(comparison is not None),
                ctypes.byref(comparison) if comparison is not None else None,
            )
            if result != 0:
                raise RuntimeError(f"Could not block {name!r}: {result}")
        result = lib.seccomp_load(context)
        if result != 0:
            raise RuntimeError(f"Could not install network filter: {result}")
    finally:
        lib.seccomp_release(context)

    # An inherited connected network socket must not bypass socket creation.
    for entry in Path("/proc/self/fd").iterdir():
        try:
            duplicate = os.dup(int(entry.name))
        except OSError:
            continue
        try:
            sock = socket.socket(fileno=duplicate)
        except OSError:
            os.close(duplicate)
            continue
        with sock:
            if sock.family != socket.AF_UNIX:
                os.close(int(entry.name))

    # Check denial without resolving, connecting to, or contacting any address.
    for family in (socket.AF_INET, socket.AF_INET6):
        for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
            try:
                unexpected = socket.socket(family, kind)
            except OSError as exc:
                if exc.errno != errno.EPERM:
                    raise RuntimeError("Network denial check failed") from exc
            else:
                unexpected.close()
                raise RuntimeError("Network filter did not block socket creation")


def main():
    command = sys.argv[1:]
    if command[:1] == ["--"]:
        command.pop(0)
    if not command:
        raise SystemExit("Usage: run_network_isolated.py -- COMMAND [ARG ...]")
    block_network()
    print("Network isolation active: non-Unix sockets blocked in this process and its descendants.",
          file=sys.stderr, flush=True)
    os.execvpe(command[0], command, os.environ)


if __name__ == "__main__":
    main()
