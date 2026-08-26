#!/usr/bin/env python3
"""Fail fast when headless Ogre2 cannot obtain OpenGL 3.3 or newer.

This deliberately uses EGL rather than GLX: Gazebo's headless Ogre2 renderer
does not need an X server, and a GLX-only check can give the wrong answer on a
remote simulation host.  It creates a tiny surfaceless/pbuffer OpenGL context
with the same user and GPU-device access that will run Gazebo.
"""

from __future__ import annotations

import ctypes
import os
import re
import sys
from typing import Final


EGL_NONE: Final = 0x3038
EGL_SURFACE_TYPE: Final = 0x3033
EGL_PBUFFER_BIT: Final = 0x0001
EGL_RENDERABLE_TYPE: Final = 0x3040
EGL_OPENGL_BIT: Final = 0x0008
EGL_WIDTH: Final = 0x3057
EGL_HEIGHT: Final = 0x3056
EGL_OPENGL_API: Final = 0x30A2
EGL_VENDOR: Final = 0x3053
GL_VERSION: Final = 0x1F02
GL_RENDERER: Final = 0x1F01
MINIMUM_GL: Final = (3, 3)


def _decode(value: bytes | None) -> str:
    return value.decode("utf-8", errors="replace") if value else "unavailable"


def _version_from_string(value: str) -> tuple[int, int] | None:
    match = re.search(r"(\d+)\.(\d+)", value)
    return (int(match.group(1)), int(match.group(2))) if match else None


def _at_least(version: tuple[int, int] | None, minimum: tuple[int, int] = MINIMUM_GL) -> bool:
    return version is not None and version >= minimum


def _egl_error(egl: ctypes.CDLL) -> str:
    egl.eglGetError.restype = ctypes.c_uint
    return f"0x{egl.eglGetError():04x}"


def main() -> int:
    # Mesa understands this platform without an X/Wayland display. Leave an
    # explicit administrator choice alone for a different EGL implementation.
    os.environ.setdefault("EGL_PLATFORM", "surfaceless")
    try:
        egl = ctypes.CDLL("libEGL.so.1")
        gl = ctypes.CDLL("libGL.so.1")
    except OSError as error:
        print(f"renderer preflight failed: cannot load EGL/OpenGL: {error}", file=sys.stderr)
        return 2

    egl.eglGetDisplay.argtypes = [ctypes.c_void_p]
    egl.eglGetDisplay.restype = ctypes.c_void_p
    egl.eglInitialize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
    egl.eglInitialize.restype = ctypes.c_uint
    egl.eglChooseConfig.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_void_p),
        ctypes.c_int, ctypes.POINTER(ctypes.c_int),
    ]
    egl.eglChooseConfig.restype = ctypes.c_uint
    egl.eglBindAPI.argtypes = [ctypes.c_uint]
    egl.eglBindAPI.restype = ctypes.c_uint
    egl.eglCreatePbufferSurface.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    egl.eglCreatePbufferSurface.restype = ctypes.c_void_p
    egl.eglCreateContext.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    egl.eglCreateContext.restype = ctypes.c_void_p
    egl.eglMakeCurrent.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    egl.eglMakeCurrent.restype = ctypes.c_uint
    egl.eglQueryString.argtypes = [ctypes.c_void_p, ctypes.c_int]
    egl.eglQueryString.restype = ctypes.c_char_p
    egl.eglDestroyContext.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    egl.eglDestroySurface.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    egl.eglTerminate.argtypes = [ctypes.c_void_p]
    gl.glGetString.argtypes = [ctypes.c_uint]
    gl.glGetString.restype = ctypes.c_char_p

    display = egl.eglGetDisplay(None)
    if not display:
        print(f"renderer preflight failed: eglGetDisplay returned no display ({_egl_error(egl)})", file=sys.stderr)
        return 2

    surface = context = None
    try:
        major = ctypes.c_int()
        minor = ctypes.c_int()
        if not egl.eglInitialize(display, ctypes.byref(major), ctypes.byref(minor)):
            print(f"renderer preflight failed: eglInitialize failed ({_egl_error(egl)})", file=sys.stderr)
            return 2

        attributes = (ctypes.c_int * 5)(
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
            EGL_NONE,
        )
        config = ctypes.c_void_p()
        count = ctypes.c_int()
        if not egl.eglChooseConfig(display, attributes, ctypes.byref(config), 1, ctypes.byref(count)) or count.value < 1:
            print(f"renderer preflight failed: no EGL OpenGL pbuffer config ({_egl_error(egl)})", file=sys.stderr)
            return 2
        if not egl.eglBindAPI(EGL_OPENGL_API):
            print(f"renderer preflight failed: EGL cannot bind OpenGL ({_egl_error(egl)})", file=sys.stderr)
            return 2

        surface_attributes = (ctypes.c_int * 5)(EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE)
        surface = egl.eglCreatePbufferSurface(display, config, surface_attributes)
        context_attributes = (ctypes.c_int * 1)(EGL_NONE)
        context = egl.eglCreateContext(display, config, None, context_attributes)
        if not surface or not context or not egl.eglMakeCurrent(display, surface, surface, context):
            print(f"renderer preflight failed: cannot create a headless OpenGL context ({_egl_error(egl)})", file=sys.stderr)
            return 2

        version_text = _decode(gl.glGetString(GL_VERSION))
        renderer = _decode(gl.glGetString(GL_RENDERER))
        version = _version_from_string(version_text)
        print(f"EGL {major.value}.{minor.value}; OpenGL {version_text}; renderer: {renderer}")
        if not _at_least(version):
            print(
                "renderer preflight failed: Ogre2 requires OpenGL "
                f"{MINIMUM_GL[0]}.{MINIMUM_GL[1]} or newer, got {version_text}",
                file=sys.stderr,
            )
            return 1
        print("renderer preflight passed")
        return 0
    finally:
        if display:
            egl.eglMakeCurrent(display, None, None, None)
        if context:
            egl.eglDestroyContext(display, context)
        if surface:
            egl.eglDestroySurface(display, surface)
        if display:
            egl.eglTerminate(display)


if __name__ == "__main__":
    raise SystemExit(main())
