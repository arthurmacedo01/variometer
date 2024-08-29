# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/arthur/Documents/Projects/esp32/esp-idf/components/bootloader/subproject"
  "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader"
  "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader-prefix"
  "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader-prefix/tmp"
  "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader-prefix/src/bootloader-stamp"
  "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader-prefix/src"
  "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/arthur/Documents/Projects/esp32/variometer_ble/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
