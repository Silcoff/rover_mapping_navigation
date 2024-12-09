#!/bin/bash

# Script usage information
function usage {
  echo "Usage: $0 [OPTIONS]"
  echo "Options:"
  echo "  --name=CONTAINER_NAME      Name of the Docker container."
  echo "  --image=IMAGE_NAME         Name of the image to run."
  echo "  --mode=MODE                "
  echo "      test:                  Temporary, removed on stop."
  echo "      devel:                 Persistent, for development."
  echo "      autostart:             Starts on boot, always running."
  echo "      stop_autostart:        Stops and disables autostart."
  echo
  echo "Example:"
  echo "  $0 --name=rover_container --mode=test"
  echo
}


# If no arguments are provided, display usage information
if [ $# -eq 0 ]; then
    usage
    exit 1
fi

# Initial variables with default values
container_name=""
container_autostart_name="rover_autostart"
image_name="rover-image"
mode="test"
docker_options=""

# Echo the input for debugging
echo "Starting script with the following input arguments:"


# Parse command-line options manually
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --name=*)
      container_name="${1#*=}"
      echo "--name=$container_name"
      ;;
    --image=*)
      image_name="${1#*=}"
      echo "--name=$image_name"
      ;;
    --mode=*)
      mode="${1#*=}"
      #echo "--mode=$mode"
      if ! [[ "$mode" =~ ^(test|devel|autostart|stop_autostart)$ ]]; then
        echo "ERROR: Invalid mode specified. Use one of the following modes: test, devel, autostart, stop_autostart."
        exit 1
      fi
      ;;
    --help)
      usage
      exit 0
      ;;
    --options=*)
      docker_options="${1#*=}"
      echo "Parsed --options: $docker_options"
      ;;
    *)
      echo "Unknown option or missing value: $1"
      exit 1
      ;;
  esac
  shift # past argument or value
done

# Echo params
echo "--mode=$mode"

if [ -z "$container_name" ]; then
  echo "ERROR: Please specify the container_name using e.g. './run.sh --name=CONTAINER_NAME'"
  container_name="rover-image-test"
  exit 1
fi

# Check if container with the specified name already exists
existing_container=$(docker ps -a -q -f name=^/${container_name}$)
if [ -n "$existing_container" ]; then
  echo "Container with name '$container_name' already exists, attempting to start it..."
  docker start "${container_name}"
  docker exec -it "${container_name}" bash
else
  # Set options based on mode
  case $mode in
    "test")
      docker_options="--rm"
      ;;
    "devel")
      docker_options=""
      ;;
    "autostart")
      container_name="${container_autostart_name}"
      start_command="autostart"
      docker_options="--restart always"
      ;;
    "stop_autostart")
      docker container stop "${container_autostart_name}"
      exit
      ;;
    *)
      echo "Invalid mode."
      exit 1
      ;;
  esac

  # Run the Docker container with the specified options
  docker run $docker_options --name "${container_name}" \
              -it \
              --privileged \
	            --runtime=nvidia \
              -v /lib/modules/5.15.148-tegra:/lib/modules/5.15.148-tegra \
              -v /etc/localtime:/etc/localtime:ro \
              -v /etc/timezone:/etc/timezone:ro \
              --network=host \
               "${image_name}" \
               ${start_command} 
fi
