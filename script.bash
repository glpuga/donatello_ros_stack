#!/bin/sh
# InOrbit agent installation and update script.

# TODO(adamantivm) Error checking and handling.
# TODO(adamantivm) Delete older versions.
# TODO(adamantivm) Trap and cleanup.

INORBIT_AGENT_VARIANT="main"

INORBIT_KEY="Ypww3tXe_TKZz7bt"
# NOTE: We need to use a different variable name than what gets stored in the
# configuration, otherwise it will be overwritten when loading the currently
# stored values.
MY_INORBIT_URL="https://control.inorbit.ai/"
INSTALLER_VERSION="4.21.1"
INSTALLER_AGENT_VERSION="4.21.1"

# Log level configuration for inorbit_agent.log
INORBIT_LOG_LEVEL_DEFAULT="info"

main() {
  set -u
  set -e

  if [ "${INORBIT_KEY}" != "update" ]; then
    display_initial_greeting
  fi

  set +u
  set_agent_install_path
  set -u

  INORBIT_DIST="${INORBIT_HOME}/dist"
  INORBIT_BACKUP="${INORBIT_HOME}/.backup"
  INORBIT_ENV="${INORBIT_HOME}/local/agent.env.sh"
  INORBIT_INSTALLER_LOG="${INORBIT_HOME}/local/install.log"

  SUPPORTED_ROS1_DISTROS="kinetic melodic noetic"
  SUPPORTED_ROS2_DISTROS="foxy humble iron jazzy"
  SUPPORTED_ROS_DISTROS="${SUPPORTED_ROS1_DISTROS} ${SUPPORTED_ROS2_DISTROS}"

  mkdir -p "${INORBIT_HOME}/local"

  cat > ${INORBIT_INSTALLER_LOG} <<EOM
InOrbit installer log $(date)
INSTALLER_VERSION: ${INSTALLER_VERSION}
INSTALLER_INORBIT_AGENT_VARIANT: ${INORBIT_AGENT_VARIANT}
INSTALLER_AGENT_VERSION: ${INSTALLER_AGENT_VERSION}
INORBIT_HOME: ${INORBIT_HOME}
EOM

  detect_environment

  if [ "${INORBIT_KEY}" != "update" ]; then
    install_system_dependencies
  fi
  create_backup
  install_agent
  install_local_dependencies
  clean_backup
  if [ "${INORBIT_KEY}" != "update" ]; then
    check_autostart
    end_message
  fi
}

display_initial_greeting() {
  echo
  echo "---------------------------------------------------------------------------------"
  echo "Welcome to the InOrbit Agent installer"
  echo "---------------------------------------------------------------------------------"
  echo
  echo "This agent will keep your robot connected to InOrbit using low bandwidth"
  echo "heartbeats."
  echo
  echo "This script does not require root privileges to run. You will be notified if"
  echo "authentication is needed to perform an action such as installing a package."
  echo
  echo "For more information, please visit https://www.inorbit.ai/faq or contact our"
  echo "support team at support@inorbit.ai"
  echo
  echo "Press ENTER to continue or CTRL+C to cancel"
  read -r input </dev/tty
  echo "---------------------------------------------------------------------------------"
}

# Configure InOrbit agent install path by detecting INORBIT_PATH environment variable
set_agent_install_path() {
  if [ -n "${INORBIT_PATH}" ] && [ -d ${INORBIT_PATH} ]
  then
    INORBIT_HOME="${INORBIT_PATH}/.inorbit"
    echo ""
    echo "---------------------------------------------------------------------------------"
    echo "INORBIT_PATH preference detected. The agent will be installed on the path:"
    echo "${INORBIT_PATH}"
    echo "---------------------------------------------------------------------------------"
    echo ""
  fi

  if [ -z ${INORBIT_PATH+set} ]
  then
    INORBIT_PATH="$HOME"
  fi

  if [ ! -w "$INORBIT_PATH" ]
  then
    echo ""
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo "User '$USER' cannot write on directory '${INORBIT_PATH}'"
    echo ""
    echo "Please update directory permissions or set a valid INORBIT_PATH and"
    echo "try again by running the initial curl command."
    echo ""
    echo "The installation will terminate now"
    echo ""
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo ""
    exit 3
  fi

  INORBIT_HOME="${INORBIT_PATH}/.inorbit"

  echo ""
  echo "InOrbit's agent home directory: ${INORBIT_HOME}"
  echo ""

}

# Detect the environment we are running on and make some choices based on that
# Currently supported environments:
#  - Ubuntu 24.04 Noble with ROS 2 Jazzy (experimental)
#  - Ubuntu 22.04 Jammy with ROS 2 Iron
#  - Ubuntu 22.04 Jammy with ROS 2 Humble
#  - Ubuntu 20.04 Focal with ROS 2 Foxy
#  - Ubuntu 20.04 Focal with ROS Noetic
#  - Ubuntu 18.04 Bionic with ROS Melodic
#  - Ubuntu 16.04 Xenial with ROS Kinetic
detect_environment() {
  # First check Ubuntu distribution
  # Current user ID: InOrbit will run under this user.
  USER=$(id -nu)

  # TODO(adamantivm) Test this on non-Ubuntu / non-LTS distros and update if necessary
  UBUNTU_CODE=$(lsb_release -cs)

  # Support for environments with Ubuntu 14.04 + ROS Indigo is deprecated as of January 2021.
  if [ "${UBUNTU_CODE}" = "trusty" ]
  then
    echo
    echo "Ubuntu 14.04 detected. This Ubuntu version is no longer supported by InOrbit."
    echo
    echo "Please contact our support team at support@inorbit.ai in case you want to get"
    echo "an older version of InOrbit's agent with support for Ubuntu 14.04."
    echo
    echo "The installation will terminate now"
    echo
    exit 3
  fi

  if [ "${UBUNTU_CODE}" != "xenial" ] && [ "${UBUNTU_CODE}" != "bionic" ] && [ "${UBUNTU_CODE}" != "focal" ] && [ "${UBUNTU_CODE}" != "jammy" ] && [ "${UBUNTU_CODE}" != "noble" ]
  then
    echo
    echo "Unsupported or unable to detect Ubuntu distribution: [${UBUNTU_CODE}]."
    echo "The InOrbit agent currently supports Ubuntu 16.04, 18.04, 20.04, 22.04 and 24.04."
    echo
    echo "For any questions or problems, please contact our support team at"
    echo "support@inorbit.ai"
    echo
    echo "The installation will terminate now"
    echo
    exit 3
  fi

  PYTHON=python # default python runner (python 2.x)
  ROS_MAJOR_VERSION=1 # Default ROS version
  # Check ROS distribution
  set +u
  if [ -n "${ROS_CUSTOM_PATH}" ] && [ -d ${ROS_CUSTOM_PATH} ]
  then
    ROS_CODE="custom"
    ROS_PATH="${ROS_CUSTOM_PATH}"
    echo
    echo "WARNING: Using custom ROS package path: ${ROS_CUSTOM_PATH}."
    echo "This means that we won't be able to automatically detect and install any missing"
    echo "ROS-based dependencies. Please make sure your custom distribution includes the"
    echo "following ROS packages: rospy."
    echo
  # If ROS_DISTRO env var is set and there is a preferred ROS version, use that one.
  elif [ `echo ${SUPPORTED_ROS_DISTROS} | grep -o ${ROS_DISTRO}` ]
  then
    ROS_CODE="${ROS_DISTRO}"
    ROS_PATH="/opt/ros/${ROS_DISTRO}"

    # It's using Python 3
    if [ ${ROS_PYTHON_VERSION} = 3 ]
    then
       PYTHON=python3
    fi

    # It's ROS 2
    if [ `echo ${SUPPORTED_ROS2_DISTROS} | grep -o ${ROS_DISTRO}` ]
    then
      ROS_MAJOR_VERSION=2
    fi
  elif [ "${UBUNTU_CODE}" = "xenial" ] && [ -d /opt/ros/kinetic ]
  then
    ROS_CODE="kinetic"
    ROS_PATH="/opt/ros/kinetic"
  elif [ "${UBUNTU_CODE}" = "bionic" ] && [ -d /opt/ros/melodic ]
  then
    ROS_CODE="melodic"
    ROS_PATH="/opt/ros/melodic"
  elif [ "${UBUNTU_CODE}" = "focal" ] && [ -d /opt/ros/noetic ]
  then
    ROS_CODE="noetic"
    ROS_PATH="/opt/ros/noetic"
    PYTHON=python3
  elif [ "${UBUNTU_CODE}" = "focal" ] && [ -d /opt/ros/foxy ]
  then
    ROS_CODE="foxy"
    ROS_PATH="/opt/ros/foxy"
    PYTHON=python3
    ROS_MAJOR_VERSION=2
  elif [ "${UBUNTU_CODE}" = "jammy" ] && [ -d /opt/ros/humble ]
  then
    echo
    echo "ROS 2 Humble detected on Ubuntu 22.04. If you also have ROS 2 Iron "
    echo "and prefer to use it instead, please update the environment variables "
    echo "on local/agent.env.sh accordingly."
    ROS_CODE="humble"
    ROS_PATH="/opt/ros/humble"
    PYTHON=python3
    ROS_MAJOR_VERSION=2
  elif [ "${UBUNTU_CODE}" = "jammy" ] && [ -d /opt/ros/iron ]
  then
    ROS_CODE="iron"
    ROS_PATH="/opt/ros/iron"
    PYTHON=python3
    ROS_MAJOR_VERSION=2
  elif [ "${UBUNTU_CODE}" = "noble" ] && [ -d /opt/ros/jazzy ]
  then
    ROS_CODE="jazzy"
    ROS_PATH="/opt/ros/jazzy"
    PYTHON=python3
    ROS_MAJOR_VERSION=2
  else
    if [ "${INORBIT_KEY}" != "update" ]; then
      echo
      echo "Unable to detect or Unsupported ROS distribution."
      echo "InOrbit Agent installation can continue with limited functionality."
      echo
      echo "To use the ROS-InOrbit integration, you will need one of:"
      echo "- Ubuntu 16.04: ROS Kinetic"
      echo "- Ubuntu 18.04: ROS Melodic"
      echo "- Ubuntu 20.04: ROS Noetic"
      echo "- Ubuntu 20.04: ROS 2 Foxy"
      echo "- Ubuntu 22.04: ROS 2 Humble"
      echo "- Ubuntu 22.04: ROS 2 Iron"
      echo "- Ubuntu 24.04: ROS 2 Jazzy"
      echo
      echo "Press ENTER to resume installation or CTRL+C to cancel."
      read -r input </dev/tty
      INORBIT_AGENT_VARIANT="core"
    fi
    ROS_CODE=""
    if [ "${UBUNTU_CODE}" = "focal" ] || [ "${UBUNTU_CODE}" = "jammy" ]; then
      PYTHON=python3
    fi
  fi

  # Set up ROS 2 variables
  if [ "${ROS_MAJOR_VERSION}" = 2 ]; then
    if [ "${INORBIT_AGENT_VARIANT}" = "main" ]; then
      INORBIT_AGENT_VARIANT="ros2"
    fi

    # ROS 2. Check for ROS_DOMAIN_ID as it's needed by the agent to
    # communicate with the robot.
    if [ "${INORBIT_KEY}" != "update" ] && [ -z "${ROS_DOMAIN_ID}" ]
    then
      echo "ROS_DOMAIN_ID is missing. If you know the value, please enter it now, otherwise press Enter."
      read -r ROS_DOMAIN_ID </dev/tty

      if [ -z "${ROS_DOMAIN_ID}" ]
      then
        echo "InOrbit will need the value of ROS_DOMAIN_ID to work. Please add it to ${INORBIT_ENV}"
        ROS_DOMAIN_ID=""
      fi
    fi
  fi

  set -u
}

# Install system dependencies (require administrative access)
install_system_dependencies()
{
  echo
  echo "Preparing to install InOrbit Agent."

  # We will add here all the lines we need to execute as root
  SUDO_FILENAME=$(mktemp)

  # Only install external dependencies when really needed, otherwise run
  # silently with local dependencies.

  set +e
  checkdep_virtualenv
  if [ $? -ne 0 ]
  then
    echo
    echo "Missing dependency: Virtualenv"
    echo "sudo apt-get install -y ${PYTHON}-virtualenv" >> ${SUDO_FILENAME}
  fi

  set +e
  checkdep_package ${PYTHON}-dev
  if [ $? -ne 0 ]
  then
    echo
    echo "Missing dependency: ${PYTHON}-dev"
    echo "sudo apt-get install -y ${PYTHON}-dev" >> ${SUDO_FILENAME}
  fi

  # Only install rospy if the system has a non-custom ROS
  # NOTE(herchu) Skip this also for ROS 2
  if [ "${ROS_CODE}" != "" ] && [ "${ROS_CODE}" != "custom" ] && [ "${ROS_MAJOR_VERSION}" -ne 2 ]
  then
    checkdep_package rospy
    if [ $? -ne 0 ]
    then
      echo
      echo "Missing dependency: rospy"
      # TODO(adamantivm) Select the proper version of rospy according to user
      # configuration. Maybe even repo configuration is necessary.
      echo "sudo apt-get install -y ros-${ROS_CODE}-rospy" >> ${SUDO_FILENAME}
    fi
  fi
  set -e

  # Execute request
  cat ${SUDO_FILENAME}
  sudo_request ${SUDO_FILENAME}

  rm ${SUDO_FILENAME}

  # TODO(adamantivm) Check if it is already insallted before insalling.
}

# Execute something as sudo before showing clearly to the user what
# we are doing.
sudo_request()
{
  if [ -s "${1}" ]
  then
    # TODO(adamantivm) Figure out how to get the user to confirm, when we're running
    # the script through piping to sh.
    # Meanwhile, just in case clear the user timestamp to force sudo to request
    # the password again, as a sort of prompt
    sudo -k
    echo
    echo "The installation process requires root access for the following commands:"
    echo
    echo "---------------------------------------------------------------------------------"
    cat ${1}
    echo "---------------------------------------------------------------------------------"
    echo
    echo "You can cancel the installation and run them manually, then resume by re-using"
    echo "the initial curl command,"
    echo "OR"
    echo "You can enter your password and these commands will be executed for you using"
    echo "sudo."
    echo
    sh ${1}
    echo
  fi
}


# Check if we have virtualenv or not
checkdep_virtualenv()
{
  which virtualenv > /dev/null
  return $?
}

# Check if we have a given python package or not
checkdep_package()
{
  PACKAGE_NAME=$1
  PKG_COUNT=`dpkg -l | grep $PACKAGE_NAME | wc -l`
  if [ $? -ne 0 ]; then return $?; fi
  if [ ${PKG_COUNT} -eq 0 ]; then return 1; fi
  return 0
}

create_backup()
{
  # NOTE(ivanpauno): If you can't create the backup folder when updating, the script doesn't continue.
  # I have taken that decission because I think that if you can't copy files, nothing will go well.
  # All the things modified by this script are backuped: dist folder, inorbit.service, agent.env.sh
  if [ "${INORBIT_KEY}" = "update" ]; then
    set +e
    echo
    echo "Backing up old agent."

    mkdir -p ${INORBIT_BACKUP}/local

    tar -C ${INORBIT_HOME} -zcf ${INORBIT_BACKUP}/dist.tar.gz dist &&
    cp -f ${INORBIT_HOME}/local/inorbit.service ${INORBIT_BACKUP}/inorbit.service &&
    cp -f ${INORBIT_ENV} ${INORBIT_BACKUP}/agent.env.sh

    if [ $? -ne 0 ]; then
      set -e
      echo "Problems while creating agent backup."
      echo "Update will not continue."

      if [ -d "${INORBIT_BACKUP}" ]; then
        rm -fR "${INORBIT_BACKUP}"
      fi
      exit 3
    fi
    set -e
    echo "Backup done."
    echo
  fi
}

restore_backup()
{
  if [ -d "${INORBIT_BACKUP}" ]; then
    echo
    echo "Restoring backup after failed update."
    rm -fR "${INORBIT_DIST}"
    tar -zxf ${INORBIT_BACKUP}/dist.tar.gz -C ${INORBIT_HOME}
    rm -f ${INORBIT_BACKUP}/dist.tar.gz
    mv -f ${INORBIT_BACKUP}/inorbit.service ${INORBIT_HOME}/local/inorbit.service
    mv -f ${INORBIT_BACKUP}/agent.env.sh ${INORBIT_ENV}
    check_autostart
    echo "Backup restored."
    echo
  fi
}

clean_backup()
{
  if [ -d "${INORBIT_BACKUP}" ]; then
    echo
    echo "Cleaning backup."
    rm -fR "${INORBIT_BACKUP}"
    echo "Backup cleared."
  fi
}

# Gets and installs InOrbit agent from the Internet.
install_agent()
{
  echo "Installing InOrbit agent"

  AGENT_TMP=$(mktemp -u --suffix .tgz)

  ERROR_OUTPUT=$(mktemp)

  set +e
  curl --fail --silent --location --continue-at - "${MY_INORBIT_URL}/artifacts/agent?variant=${INORBIT_AGENT_VARIANT}&rnd=$(date +%N)" --output "${AGENT_TMP}" 2>${ERROR_OUTPUT}
  AGENT_DOWNLOAD_EXIT_CODE=$?
  set -e

  if [ $AGENT_DOWNLOAD_EXIT_CODE -ne 0 ]
  then
    echo
    echo "curl command for agent download failed with error code $AGENT_DOWNLOAD_EXIT_CODE"
    echo
    echo "Please check your network connection and try again by running the initial"
    echo "curl command."
    echo
    echo "The installation will terminate now."
    echo

    rm ${AGENT_TMP}
    exit 3
  fi

  # Wipe all version and install fresh.
  cd
  rm -rf "${INORBIT_DIST}"
  mkdir -p "${INORBIT_DIST}"
  cd "${INORBIT_DIST}"
  tar -zxf "${AGENT_TMP}"
  rm ${AGENT_TMP}
  set +e
  echo "Downloaded agent:" >> ${INORBIT_INSTALLER_LOG}
  grep VERSION ./inorbit/__init__.py >> ${INORBIT_INSTALLER_LOG}
  grep VARIANT ./inorbit/__init__.py >> ${INORBIT_INSTALLER_LOG}
  set -e

  # If there is no local installation yet, create and configure.
  if [ ! -d "${INORBIT_HOME}/local" -o ! -f "${INORBIT_ENV}" ]
  then
    # Create directory for local date
    mkdir -p "${INORBIT_HOME}/local"

    # Generate new robot ID
    # TODO(adamantivm) Use a proper, more robust UUID generation method
    INORBIT_ID="$(date +%N)"

    # Generate systemd service script tailored for this configuration.
    cat > ${INORBIT_HOME}/local/inorbit.service <<EOM
[Unit]
Description=InOrbit Agent Service

[Service]
Type=simple
User=${USER}
WorkingDirectory=${INORBIT_DIST}
ExecStart=${INORBIT_DIST}/scripts/start.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOM

  else
    # There already is an environment configuration file, read
    # Robot ID and other current configuration variables from there
    . ${INORBIT_ENV}

    # 0.2.7: Previous versions used systemd forking mode but this is now non-forking
    sed -i 's/forking/simple/' ${INORBIT_HOME}/local/inorbit.service

    if [ -z "${INORBIT_ID}" ]; then
      echo "No INORBIT_ID found even though there should be an environment already set."
      exit 3
    fi

    set +u
    # When updating with a custom ROS location, ROS_CUSTOM_PATH env
    # var is lost, so we need to update ROS_CODE and ROS_PATH with
    # the previous INORBIT_ROS_PATH
    if [ "${INORBIT_ROS}" = "custom" ]; then
      ROS_CODE="${INORBIT_ROS}"
      ROS_PATH="${INORBIT_ROS_PATH}"
    fi
    set -u
  fi

  # Create or Update the agent configuration file
  if [ "${INORBIT_KEY}" = "update" ]; then
    echo "WARNING! Previously configured INORBIT_KEY not found! Couldn't re-configure agent!"
  else
    cat > ${INORBIT_ENV} <<EOM
export INORBIT_ID="${INORBIT_ID}"
export INORBIT_KEY="${INORBIT_KEY}"
export INORBIT_AGENT_VARIANT="${INORBIT_AGENT_VARIANT}"
export INORBIT_URL="${MY_INORBIT_URL}"
export INORBIT_PATH="${INORBIT_PATH}"
export INORBIT_HOME="${INORBIT_HOME}"
export INORBIT_LOG_LEVEL="${INORBIT_LOG_LEVEL_DEFAULT}"
export INORBIT_ENABLE_WATCHDOG="no"
EOM

    # Add ROS CODE to the config file only if ROS is present on the system
    if [ "$ROS_CODE" != "" ]; then
    cat >> ${INORBIT_ENV} <<EOM
export INORBIT_ROS="${ROS_CODE}"
export INORBIT_ROS_PATH="${ROS_PATH}"
EOM
    fi

    if [ `echo ${SUPPORTED_ROS2_DISTROS} | grep -o ${ROS_CODE}` ] && [ ! -z "${ROS_DOMAIN_ID}" ]; then
    cat >> ${INORBIT_ENV} <<EOM
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
EOM
    fi

    # If HTTP_PROXY variable is set during install time, append it to agent.env
    set +u
    if [ -n "$HTTP_PROXY" ]; then
    cat >> ${INORBIT_ENV} <<EOM
export HTTP_PROXY=${HTTP_PROXY}
EOM
    fi
    set -u

    # Only append ROS environment variables if they are found in the system during install
    # NOTE: They will subsequently be copied over from one agent update to the next,
    # without update if the host environment is changed.
    # TODO(adamantivm) Allow a more flexible and intelligent mechanism to avoid
    # duplicating where this is configured.
    set +u
    if [ -n "${ROS_IP}" ]; then
      echo "export ROS_IP=${ROS_IP}" >> ${INORBIT_ENV}
    fi
    if [ -n "${ROS_MASTER_URI}" ] && [ "${ROS_MASTER_URI}" != "http://localhost:11311" ]; then
      echo "export ROS_MASTER_URI=${ROS_MASTER_URI}" >> ${INORBIT_ENV}
    fi
    if [ -n "${LD_LIBRARY_PATH}" ]; then
      echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}" >> ${INORBIT_ENV}
    fi
    set -u

    # If Agent API customization variables are set, append it to agent.env
    # Used by the core variant
    set +u
    if [ -n "$INORBIT_AGENT_API_PORT" ]; then
    cat >> ${INORBIT_ENV} <<EOM
export INORBIT_AGENT_API_PORT="${INORBIT_AGENT_API_PORT}"
EOM
    fi
    if [ -n "$INORBIT_AGENT_API_BINDING" ]; then
    cat >> ${INORBIT_ENV} <<EOM
export INORBIT_AGENT_API_BINDING="${INORBIT_AGENT_API_BINDING}"
EOM
    fi
    set -u
  fi
}

install_local_dependencies()
{
  if [ ! -f $ERROR_OUTPUT ]; then
    ERROR_OUTPUT=$(mktemp)
  fi

  # Create virtualenv
  cd ${INORBIT_DIST}
  if [ ! -d "${INORBIT_DIST}/venv" ]
  then

    # Choose an explicit python interpreter in case the environment
    # is configured in an unusual way to use the non-default version
    # of python for that operating system and platform
    PYTHON_INTERPRETER="python2"
    if [ "${PYTHON}" = "python3" ]
    then
      PYTHON_INTERPRETER="python3"
    fi

    set +e
    virtualenv -p "${PYTHON_INTERPRETER}" "${INORBIT_DIST}/venv" 2>${ERROR_OUTPUT}
    CREATE_VIRTUALENV_EXIT_CODE=$?
    set -e

    if [ $CREATE_VIRTUALENV_EXIT_CODE -ne 0 ]
    then
      echo
      echo "virtualenv failed with error code $CREATE_VIRTUALENV_EXIT_CODE"
      echo
      echo "Please check your network connection and try again by running the initial"
      echo "curl command."
      echo
      echo "The installation will terminate now."
      echo

      # If directory was created, then delete it to do a fresh re-install
      if [ -d "${INORBIT_DIST}/venv" ]
      then
        rm -rf ${INORBIT_DIST}/venv
      fi

      restore_backup
      exit 3
    fi
  fi

  # Activate virtualenv
  set +u
  . "${INORBIT_DIST}/venv/bin/activate"
  set -u

  # Install python dependencies
  set +e

  # Check which requirements are needed: core (limited, non-ROS) or full (+ ROS)
  if [ "$ROS_CODE" = "" ]
  then
    # TODO (FlorGrosso): update + split requirements based on python version.
    # Currently the support for non-ROS environments installs python 2 packages.
    pip install -r core_requirements.txt 2>${ERROR_OUTPUT}
    PIP_EXIT_CODE=$?

    if [ $PIP_EXIT_CODE -eq 0 ] && [ -f ${PYTHON}_requirements.txt ]; then
      pip install -r ${PYTHON}_requirements.txt 2>${ERROR_OUTPUT}
      PIP_EXIT_CODE=$?
    fi
  else
    # TODO (FlorGrosso): update the version with the exact value when merging
    # noetic variant to master.
    # AGENT VER 3.X
    # [Placeholder] Backwards compatibility. Agent versions prior to 3.X use a different
    # naming convention for requirements files.
    if [ -f full_requirements.txt ]
    then
      pip install -r full_requirements.txt 2>${ERROR_OUTPUT}
      PIP_EXIT_CODE=$?

    else
      # Agent version > 3.3
      # Install Python packages depending on the Python version.
      # TODO (FlorGrosso): split requirements based on ROS distro as well.
      pip install -r ${PYTHON}_requirements.txt 2>${ERROR_OUTPUT}
      PIP_EXIT_CODE=$?

      if [ $PIP_EXIT_CODE -eq 0 ]; then
        # Now install common packages pinned to a version that supports python 2.7 and python 3.
        pip install -r common_requirements.txt 2>${ERROR_OUTPUT}
        PIP_EXIT_CODE=$?
      fi
    fi
  fi

  set -e

  if [ $PIP_EXIT_CODE -ne 0 ]
  then
    echo
    echo "pip failed with error code $PIP_EXIT_CODE."
    echo "You can find detailed error output in the ${ERROR_OUTPUT} file."
    echo
    echo "Please check your network connection and try again by running the initial"
    echo "curl command."
    echo
    echo "The installation will terminate now."
    echo

    restore_backup
    exit 3
  fi

  # Workaround to use protobuf 3.5.x with Python > 3.9
  # https://github.com/vm03/payload_dumper/issues/27#issuecomment-1021334222
  if [ "$ROS_CODE" = "humble" ] || [ "$ROS_CODE" = "iron" ]
  then
    PROTOC_DIR=${INORBIT_DIST}/venv/lib/python3.10/site-packages/google/protobuf/internal
    sed -i "s|import collections|import collections.abc as collections|g" ${PROTOC_DIR}/containers.py
    sed -i "s|import collections|import collections.abc as collections|g" ${PROTOC_DIR}/well_known_types.py
  elif [ "$ROS_CODE" = "jazzy" ]
  then
    PROTOC_DIR=${INORBIT_DIST}/venv/lib/python3.12/site-packages/google/protobuf/internal
    sed -i "s|import collections|import collections.abc as collections|g" ${PROTOC_DIR}/containers.py
    sed -i "s|import collections|import collections.abc as collections|g" ${PROTOC_DIR}/well_known_types.py
  fi

  # Everything went ok. Wipe error file.
  rm ${ERROR_OUTPUT}

  # Create a link to cv2 for ROS Melodic
  # TODO(Flor_Grosso) Find a better way to access cv2 python package
  if [ "${UBUNTU_CODE}" = "bionic" ]
  then
    # Automatically find and softlink cv2 package for all CPU architectures
    cv2_directory=/usr/lib/python2.7/dist-packages/
    if [ -f ${cv2_directory}cv2* ]
    then
      cv2_file=$(ls ${cv2_directory} | grep ^cv2)
      ln -s ${cv2_directory}${cv2_file} ${INORBIT_DIST}/venv/lib/python2.7/
    else
      echo "Python cv2 module not found. Skipping soft link."
    fi
  fi
}

# Install if necessary and (re)start the InOrbit agent through the appropriate
# init system for the target distribution
check_autostart()
{
  if [ "${UBUNTU_CODE}" = "xenial" ] || [ "${UBUNTU_CODE}" = "bionic" ] || [ "${UBUNTU_CODE}" = "focal" ] || [ "${UBUNTU_CODE}" = "jammy" ] || [ "${UBUNTU_CODE}" = "noble" ]
  then
    check_systemd
  else
    no_autostart
  fi
}

# Display this message when we still don't have an autostart configuration
# for this distribution
no_autostart()
{
  echo "Agent autostart and respawn not available for your distribution: [${UBUNTU_CODE}]"
  echo
  echo "You will have to run ${INORBIT_DIST}/scripts/start.sh to start the InOrbit agent"
}

# Check if the InOrbit systemd service is running and start it otherwise
check_systemd()
{
  # TODO(adamantivm) Better check if it is already set-up and running

  if [ ! -h /etc/systemd/system/inorbit.service ]
  then
    echo
    echo "Setting-up InOrbit agent as a systemd service."
    SUDO_FILENAME=$(mktemp)
    echo "sudo cp ${INORBIT_HOME}/local/inorbit.service /etc/systemd/system" > ${SUDO_FILENAME}
    echo "sudo systemctl enable inorbit.service" >> ${SUDO_FILENAME}
    echo "sudo systemctl start inorbit" >> ${SUDO_FILENAME}
    sudo_request ${SUDO_FILENAME}
    rm ${SUDO_FILENAME}
  fi
}

end_message()
{
  echo
  echo "---------------------------------------------------------------------------------"
  echo "Agent installation successful. Your robot is now InOrbit!"
  echo
  echo "Go to https://control.inorbit.ai/ to work with your robot fleet."
  echo
  echo "If you wish to uninstall the agent, please run the script:"
  echo
  echo "${INORBIT_HOME}/dist/scripts/uninstall.sh"
  echo
  echo "For more information, please visit https://www.inorbit.ai/faq or"
  echo "contact our support team at support@inorbit.ai"
  echo
}

main
