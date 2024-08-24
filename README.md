set `PICO_SDK_PATH` environment variable or pass it in `-DPICO_SDK_PATH=` argument

```bash
git clone https://github.com/raspberrypi/pico-sdk --recursive
```

and you should have `arm-none-eabi-gcc` in your PATH

and build [`raspberrypi/picotool`](https://github.com/raspberrypi/picotool) and add it to your PATH

See also [Creating a new C/C++ Raspberry Pi Pico Project on Windows](https://vanhunteradams.com/Pico/Setup/NewProjectWindows.html)

```powershell
"C:\Program Files\CMake\bin\cmake.EXE" -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DPICO_SDK_PATH=~/pico-sdk -DPICOTOOL_PATH=C:/tools/picotool -GNinja -DCMAKE_C_COMPILER=C:/tools/arm-gnu-toolchain-13.2/bin/arm-none-eabi-gcc.exe -DCMAKE_CXX_COMPILER=C:/tools/arm-gnu-toolchain-13.2/bin/arm-none-eabi-g++.exe --no-warn-unused-cli -SC:/Users/cross/Desktop/code/heart-pico -Bc:/Users/cross/Desktop/code/heart-pico/build
```
