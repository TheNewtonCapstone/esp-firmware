# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/galileo/esp/esp-idf/components/bootloader/subproject"
  "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader"
  "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader-prefix"
  "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader-prefix/tmp"
  "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader-prefix/src/bootloader-stamp"
  "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader-prefix/src"
  "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/galileo/workspaces/newton-esp-firmware/receiver/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
