#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
pushd "${SCRIPT_DIR}" > /dev/null
echo "Working Path: ${SCRIPT_DIR}"

WORKSPACE_ROOT=""
PACKAGE_LINK=""

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION

WORKSPACE_ROOT="${SCRIPT_DIR}/.ws_${ROS_VERSION,,}"
PACKAGE_LINK="${WORKSPACE_ROOT}/src/livox_ros_driver2"

# prepare isolated workspace inside the repository
rm -rf "${WORKSPACE_ROOT}"
mkdir -p "${WORKSPACE_ROOT}/src"

# avoid recursive symlink leftovers if a previous build failed
if [ -L "${PACKAGE_LINK}" ] || [ -e "${PACKAGE_LINK}" ]; then
    rm -rf "${PACKAGE_LINK}"
fi
ln -s "${SCRIPT_DIR}" "${PACKAGE_LINK}"

# exit

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# build
pushd "${WORKSPACE_ROOT}" > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    catkin_make -DROS_EDITION=${VERSION_ROS1}
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null
