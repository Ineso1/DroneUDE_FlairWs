# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/nessy/flair/flair-src/lib")
  file(MAKE_DIRECTORY "/home/nessy/flair/flair-src/lib")
endif()
file(MAKE_DIRECTORY
  "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs"
  "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs-prefix"
  "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs-prefix/tmp"
  "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs-prefix/src/FlairLibs-stamp"
  "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs-prefix/src"
  "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs-prefix/src/FlairLibs-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs-prefix/src/FlairLibs-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/nessy/Documents/Hds_Repo/DroneUDE_FlairWs/build/FlairLibs-prefix/src/FlairLibs-stamp${cfgdir}") # cfgdir has leading slash
endif()
