#!/bin/bash
# CycloneDDS localhost config (fixes macOS discovery issues)
export CYCLONEDDS_URI="file://${PIXI_PROJECT_ROOT}/cyclonedds.xml"
export ROS_LOCALHOST_ONLY=1

# Conditionally source the colcon overlay if it has been built
if [ -f "${PIXI_PROJECT_ROOT}/install/setup.sh" ]; then
    source "${PIXI_PROJECT_ROOT}/install/setup.sh"
fi
