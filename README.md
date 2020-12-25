#infinity-firmware
Firmware for Infinity ESCs
##Setup
Clone repostiory. 
```
git clone https://github.com/raphaelchang/infinity-firmware.git
```
##Compiling
You'll need to install the following gcc arm cross compiler.
```
sudo apt-get install gcc-arm-none-eabi
```
To build the files. Navigate to the root of the repository and run the following:
```
make
```
##Programming
You'll need to install dfu-util v0.9 or higher. 
####Mac OS
On Mac, the homebrew version is adequate. 
```
brew install dfu-util
```
####Ubuntu
On Ubuntu 14.04, dfu-util must be installed from source. See instructions here: http://dfu-util.sourceforge.net/build.html.

####Uploading code

1. Plug in Infinity to computer via USB.
2. Enable bootloader mode by reseting the board while the bootloader button is held down.
3. Ensure that no other boards are connected that are also in bootloader mode.
4. Run the following to build and upload the code.
```
make upload
```
