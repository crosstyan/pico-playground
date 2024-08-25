# Raspberry Pi Pico Playground

## Build

set `PICO_SDK_PATH` environment variable or pass it in `-DPICO_SDK_PATH=` argument

```bash
git clone https://github.com/raspberrypi/pico-sdk --recursive
```

and you should have `arm-none-eabi-gcc` in your PATH

and build [`raspberrypi/picotool`](https://github.com/raspberrypi/picotool) (or download it from [`crosstyan/picotool-prebuilt`](https://github.com/crosstyan/picotool-prebuilt) if in Windows) and add it to your PATH

See also [Creating a new C/C++ Raspberry Pi Pico Project on Windows](https://vanhunteradams.com/Pico/Setup/NewProjectWindows.html)

```powershell
"C:\Program Files\CMake\bin\cmake.EXE" -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DPICO_SDK_PATH=~/pico-sdk -DPICOTOOL_PATH=C:/tools/picotool -GNinja -DCMAKE_C_COMPILER=C:/tools/arm-gnu-toolchain-13.2/bin/arm-none-eabi-gcc.exe -DCMAKE_CXX_COMPILER=C:/tools/arm-gnu-toolchain-13.2/bin/arm-none-eabi-g++.exe --no-warn-unused-cli -SC:/Users/cross/Desktop/code/heart-pico -Bc:/Users/cross/Desktop/code/heart-pico/build
```

## Upload

```powershell
# if USB STDIO is enabled, otherwise have to press BOOTSEL button
picotool reboot -f -u
cp .\build\heart-pi.uf2 G:/
# or
picotool load .\build\heart-pi.uf2
```

See [raspberrypi/pico-examples](https://github.com/raspberrypi/pico-examples) for more examples

## Monitor

Install [Msys2](https://www.msys2.org/) and [tio](https://github.com/tio/tio). 

```bash
pacman -S tio
tio --list-devices
# like COM11
tio -b 115200 /dev/ttyS11
```

- [Serial Monitor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor) for Visual Studio Code
- [Serial Port Monitor](https://plugins.jetbrains.com/plugin/8031-serial-port-monitor) for JetBrains IDEs

