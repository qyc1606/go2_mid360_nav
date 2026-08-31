#!/usr/bin/env bash

set -u

# ============================================================
# Complete RealSense cleanup script
#
# Target:
#   Jetson AGX Orin
#   Ubuntu 20.04
#   ROS Noetic
#
# It removes:
#   1. librealsense / realsense ROS Debian packages
#   2. /usr/local source-installed librealsense
#   3. realsense-ros source code
#   4. librealsense source tree
#   5. RealSense udev rules
#   6. RealSense related ROS logs
#   7. Optional catkin build/devel/install directories
#
# It DOES NOT remove:
#   - ROS Noetic
#   - ddynamic_reconfigure
#   - Livox
#   - FAST-LIO
#   - Unitree SDK2
# ============================================================

HOME_DIR="${HOME}"
PROJECT_DIR="${HOME_DIR}/go2_mid360_nav"
CATKIN_WS="${PROJECT_DIR}/catkin_ws"
CATKIN_SRC="${CATKIN_WS}/src"
THIRD_PARTY="${PROJECT_DIR}/third_party"

echo
echo "============================================================"
echo " RealSense COMPLETE CLEANUP"
echo "============================================================"
echo
echo "Project directory:"
echo "  ${PROJECT_DIR}"
echo
echo "This script will remove ALL librealsense / realsense-ros"
echo "installations that it can find."
echo
echo "It will NOT remove:"
echo "  - ROS Noetic"
echo "  - ros-noetic-ddynamic-reconfigure"
echo "  - Livox"
echo "  - FAST-LIO"
echo "  - Unitree SDK2"
echo

read -r -p "Type YES to continue: " ANSWER

if [[ "${ANSWER}" != "YES" ]]; then
    echo
    echo "Cancelled."
    exit 0
fi


# ------------------------------------------------------------
# 1. Stop RealSense ROS processes
# ------------------------------------------------------------

echo
echo "[1/10] Stopping RealSense related processes..."

pkill -f realsense2_camera 2>/dev/null || true
pkill -f realsense-viewer 2>/dev/null || true
pkill -f rs-enumerate-devices 2>/dev/null || true

# Do not indiscriminately kill all nodelets because other ROS
# packages may currently use nodelets.

sleep 1


# ------------------------------------------------------------
# 2. Remove apt / ROS RealSense packages
# ------------------------------------------------------------

echo
echo "[2/10] Searching installed RealSense Debian packages..."

REAL_SENSE_PACKAGES=$(
    dpkg-query -W -f='${binary:Package}\n' 2>/dev/null \
    | grep -Ei '(^librealsense|^realsense|^ros-noetic-.*realsense)' \
    || true
)

if [[ -n "${REAL_SENSE_PACKAGES}" ]]; then

    echo
    echo "Packages to purge:"
    echo "${REAL_SENSE_PACKAGES}"
    echo

    # shellcheck disable=SC2086
    sudo apt purge -y ${REAL_SENSE_PACKAGES}

else

    echo "No RealSense Debian packages found."

fi

sudo apt autoremove -y


# ------------------------------------------------------------
# 3. Remove /usr/local source-installed librealsense libraries
# ------------------------------------------------------------

echo
echo "[3/10] Removing /usr/local librealsense libraries..."

sudo rm -f /usr/local/lib/librealsense2.so
sudo rm -f /usr/local/lib/librealsense2.so.*
sudo rm -f /usr/local/lib/librealsense2-gl.so
sudo rm -f /usr/local/lib/librealsense2-gl.so.*

# Some installations use lib64.
sudo rm -f /usr/local/lib64/librealsense2.so 2>/dev/null || true
sudo rm -f /usr/local/lib64/librealsense2.so.* 2>/dev/null || true
sudo rm -f /usr/local/lib64/librealsense2-gl.so 2>/dev/null || true
sudo rm -f /usr/local/lib64/librealsense2-gl.so.* 2>/dev/null || true


# ------------------------------------------------------------
# 4. Remove librealsense headers / cmake / pkgconfig
# ------------------------------------------------------------

echo
echo "[4/10] Removing librealsense headers and build metadata..."

sudo rm -rf /usr/local/include/librealsense2
sudo rm -rf /usr/local/include/librealsense2-gl

sudo rm -rf /usr/local/lib/cmake/realsense2
sudo rm -rf /usr/local/lib64/cmake/realsense2 2>/dev/null || true

sudo rm -f /usr/local/lib/pkgconfig/realsense2.pc
sudo rm -f /usr/local/lib64/pkgconfig/realsense2.pc 2>/dev/null || true

# Older / alternate names if present.
sudo rm -rf /usr/local/share/realsense2 2>/dev/null || true


# ------------------------------------------------------------
# 5. Remove source-installed RealSense tools
# ------------------------------------------------------------

echo
echo "[5/10] Removing source-installed RealSense executables..."

REAL_SENSE_BINARIES=(
    rs-align
    rs-align-advanced
    rs-callback
    rs-capture
    rs-color
    rs-convert
    rs-data-collect
    rs-depth
    rs-depth-quality
    rs-distance
    rs-enumerate-devices
    rs-fw-logger
    rs-fw-update
    rs-gl
    rs-hdr
    rs-hello-realsense
    rs-imu
    rs-measure
    rs-multicam
    rs-pointcloud
    rs-post-processing
    rs-record
    rs-record-playback
    rs-rosbag-inspector
    rs-save-to-disk
    rs-sensor-control
    rs-software-device
    rs-terminal
    rs-tracking-and-mapping
    rs-tracking
    realsense-viewer
)

for BIN in "${REAL_SENSE_BINARIES[@]}"; do
    sudo rm -f "/usr/local/bin/${BIN}"
done

# Remove any remaining rs-* files under /usr/local/bin.
sudo find /usr/local/bin \
    -maxdepth 1 \
    -type f \
    -name 'rs-*' \
    -delete 2>/dev/null || true


# ------------------------------------------------------------
# 6. Remove udev rules
# ------------------------------------------------------------

echo
echo "[6/10] Removing RealSense udev rules..."

sudo rm -f /etc/udev/rules.d/99-realsense-libusb.rules

# In case rules were copied under system location.
sudo rm -f /lib/udev/rules.d/99-realsense-libusb.rules 2>/dev/null || true
sudo rm -f /usr/lib/udev/rules.d/99-realsense-libusb.rules 2>/dev/null || true

sudo udevadm control --reload-rules
sudo udevadm trigger


# ------------------------------------------------------------
# 7. Remove ROS wrapper source packages
# ------------------------------------------------------------

echo
echo "[7/10] Removing realsense-ros source packages..."

rm -rf "${CATKIN_SRC}/realsense-ros"
rm -rf "${CATKIN_SRC}/realsense_ros"
rm -rf "${CATKIN_SRC}/realsense2_camera"
rm -rf "${CATKIN_SRC}/realsense2_description"

# Handle directories with version suffixes.
find "${CATKIN_SRC}" \
    -maxdepth 1 \
    -type d \
    \( -iname 'realsense-ros*' \
       -o -iname 'realsense2_camera*' \
       -o -iname 'realsense2_description*' \) \
    -print \
    -exec rm -rf {} + 2>/dev/null || true


# ------------------------------------------------------------
# 8. Remove librealsense source tree
# ------------------------------------------------------------

echo
echo "[8/10] Removing librealsense source tree..."

rm -rf "${THIRD_PARTY}/librealsense"
rm -rf "${THIRD_PARTY}/librealsense2"

find "${THIRD_PARTY}" \
    -maxdepth 1 \
    -type d \
    -iname 'librealsense*' \
    -print \
    -exec rm -rf {} + 2>/dev/null || true


# ------------------------------------------------------------
# 9. Remove RealSense ROS logs
# ------------------------------------------------------------

echo
echo "[9/10] Cleaning old RealSense ROS logs..."

if [[ -d "${HOME_DIR}/.ros/log" ]]; then

    find "${HOME_DIR}/.ros/log" \
        -type f \
        \( -iname '*realsense*' \
           -o -iname '*camera_manager*' \) \
        -delete 2>/dev/null || true

fi


# ------------------------------------------------------------
# 10. Refresh dynamic linker
# ------------------------------------------------------------

echo
echo "[10/10] Refreshing dynamic linker cache..."

sudo ldconfig


# ------------------------------------------------------------
# Optional catkin clean
# ------------------------------------------------------------

echo
echo "============================================================"
echo " Catkin build cache"
echo "============================================================"
echo
echo "Old catkin build/devel files may still contain a"
echo "librealsense2_camera.so linked against the old SDK."
echo
echo "For a truly clean reinstallation, cleaning them is recommended."
echo
echo "This will NOT remove anything under catkin_ws/src."
echo

read -r -p "Remove catkin_ws/build, devel and install? [y/N]: " CLEAN_CATKIN

if [[ "${CLEAN_CATKIN}" =~ ^[Yy]$ ]]; then

    echo
    echo "Removing catkin build products..."

    rm -rf "${CATKIN_WS}/build"
    rm -rf "${CATKIN_WS}/devel"
    rm -rf "${CATKIN_WS}/install"

else

    echo
    echo "Catkin build products kept."

fi


# ------------------------------------------------------------
# Verification
# ------------------------------------------------------------

echo
echo
echo "============================================================"
echo " Verification"
echo "============================================================"
echo

echo "[A] Debian packages:"
dpkg -l | grep -Ei "realsense|librealsense" || echo "  OK: none"

echo
echo "[B] ldconfig:"
ldconfig -p | grep librealsense || echo "  OK: none"

echo
echo "[C] pkg-config:"
if pkg-config --exists realsense2 2>/dev/null; then
    echo "  WARNING: realsense2 still exists:"
    pkg-config --modversion realsense2
    pkg-config --variable=libdir realsense2
else
    echo "  OK: realsense2 not found"
fi

echo
echo "[D] /usr/local residual files:"
sudo find /usr/local \
    -maxdepth 5 \
    -iname '*realsense*' \
    -print 2>/dev/null \
    | head -100

echo
echo "[E] catkin source residual files:"
find "${CATKIN_SRC}" \
    -maxdepth 3 \
    -iname '*realsense*' \
    -print 2>/dev/null

echo
echo "[F] third_party residual files:"
find "${THIRD_PARTY}" \
    -maxdepth 2 \
    -iname '*realsense*' \
    -print 2>/dev/null

echo
echo "============================================================"
echo " Cleanup finished."
echo "============================================================"
echo
echo "Expected final state:"
echo
echo "  pkg-config --modversion realsense2"
echo "      -> Package realsense2 was not found"
echo
echo "  ldconfig -p | grep librealsense"
echo "      -> no output"
echo
echo "  dpkg -l | grep -Ei 'realsense|librealsense'"
echo "      -> no output"
echo
echo "Do NOT reinstall anything until the verification above is clean."
echo


