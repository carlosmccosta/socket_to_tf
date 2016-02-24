#!/usr/bin/env sh

ros_version=${1:-"$(rosversion -d)"}
install_args=${2:-"-y --allow-unauthenticated"}


echo "####################################################################################################"
echo "##### Checking and installing dependencies for socket_to_tf ros package"
echo "####################################################################################################"

sudo apt-get update
sudo apt-get upgrade ${install_args}
sudo apt-get dist-upgrade ${install_args}


# required system dependencies
sudo apt-get install git ${install_args}
sudo apt-get install libzmq3-dev ${install_args}


# required ros packages
sudo apt-get install ros-${ros_version}-cpp-common ${install_args}
sudo apt-get install ros-${ros_version}-geometry-msgs ${install_args}
sudo apt-get install ros-${ros_version}-rospp ${install_args}
sudo apt-get install ros-${ros_version}-rosconsole ${install_args}
sudo apt-get install ros-${ros_version}-rostime ${install_args}
sudo apt-get install ros-${ros_version}-tf2 ${install_args}
sudo apt-get install ros-${ros_version}-tf2-ros ${install_args}


sudo apt-get upgrade ${install_args}
sudo apt-get dist-upgrade ${install_args}

echo "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Installation of socket_to_tf dependencies finished"
echo "----------------------------------------------------------------------------------------------------"
