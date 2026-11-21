@echo off
rem Build skillarm with MSVC + Ninja (Visual Studio 2022).
set VS=C:\Program Files\Microsoft Visual Studio\2022\Community
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
set CMAKE="%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if not exist build\build.ninja %CMAKE% -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM="%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
%CMAKE% --build build
