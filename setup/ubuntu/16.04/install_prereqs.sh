#!/bin/bash
#
# Prerequisite set-up script for Drake with Bazel on Ubuntu 16.04.

set -euo pipefail

die () {
    echo "$@" 1>&2
    exit 1
}

me="The Drake prerequisite set-up script"

[[ $EUID -eq 0 ]] || die "$me must run as root. Please use sudo."

apt update
apt install --no-install-recommends lsb-release wget

[[ "$(lsb_release -sc)" == "xenial" ]] || die "$me only supports Ubuntu 16.04."

# Install Clang 3.9
while true; do
  echo "The Ubuntu 16.04 distribution includes Clang 3.8 by default."
  echo "To install Clang 3.9 it is necessary to add a Personal Package Archive (PPA)."
  echo "This script will add the repository
    'deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-3.9 main'"
  read -p "Do you want to continue? [Y/n] " yn
  case $yn in
    [Yy]*)
      apt install --no-install-recommends software-properties-common
      wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | apt-key add -
      # In this form, add-apt-repository is only truly idempotent when -s is
      # added, since it otherwise duplicates the commented deb-src line.
      add-apt-repository -s -y "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-3.9 main"
      apt update
      apt install --no-install-recommends clang-3.9 clang-format-3.9 lldb-3.9
      break
      ;;
    [Nn]*) break ;;
    *) echo "Please answer yes or no." ;;
  esac
done

# Install the APT dependencies.
apt update -y
apt install --no-install-recommends $(tr '\n' ' ' <<EOF

bash-completion
binutils
coinor-libipopt-dev
doxygen
g++
g++-5
g++-5-multilib
gcc
gcc-5
gcc-5-multilib
gdb
git
graphviz
libblas-dev
libboost-dev
libexpat1-dev
libfreetype6
libglib2.0-dev
libglu1-mesa-dev
libhdf5-10
libjpeg8
libjsoncpp1
liblapack-dev
liblz4-dev
libnetcdf-c++4
libnetcdf11
libogg0
libpng-dev
libqt5multimedia5
libqt5opengl5
libqt5x11extras5
libtheora0
libtiff5
libtinyxml-dev
libtool
libxml2
libxt6
mesa-common-dev
openjdk-8-jdk
patchutils
pkg-config
python-dev
python-gtk2
python-lxml
python-numpy
python-pygame
python-scipy
python-sphinx
python-yaml
valgrind
zip
zlib1g-dev

EOF
    )

# Install Bazel.
wget -O /tmp/bazel_0.5.2-linux-x86_64.deb https://github.com/bazelbuild/bazel/releases/download/0.5.2/bazel_0.5.2-linux-x86_64.deb
if echo "b14c8773dab078d3422fe4082f3ab4d9e14f02313c3b3eb4b5b40c44ce29ed59 /tmp/bazel_0.5.2-linux-x86_64.deb" | sha256sum -c -; then
  dpkg -i /tmp/bazel_0.5.2-linux-x86_64.deb
else
  die "The Bazel deb does not have the expected SHA256.  Not installing Bazel."
fi

rm /tmp/bazel_0.5.2-linux-x86_64.deb

# Remove deb that we used to generate and install, but no longer need.
if [ -L /usr/lib/ccache/bazel ]; then
  apt purge ccache-bazel-wrapper
fi
