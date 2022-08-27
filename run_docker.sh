#!/bin/bash

set -e

. settings.sh

function usage() {
    printf "Usage: $0 [options]\n\n"
    printf "Options:\n"
    printf "  -d DISTRIBUTION_NAME\t Select ubuntu distribution (image name suffix)\n"
    printf "  -h\t\t\t Shows this help message\n"
    printf "  -p PROJECT_NAME\t Select project (image name prefix)\n"
    printf "  -t TAG\t\t Image tag to use\n"
    printf "  -u USERNAME\t\t Select the username inside container\n"
    printf "  -x\t\t\t Disable X display support\n"

    exit 0
}

TAG_NAME=devel
while getopts "d:hp:t:u:x" OPT; do
    case "${OPT}" in
        "d") ROSDISTRO=${OPTARG};;
        "h") usage;;
        "p") PROJECT=${OPTARG};;
        "t") TAG_NAME=${OPTARG};;
        "u") CONTAINER_USER=${OPTARG};;
        "x") DISABLE_X="1";;
        "?") exit 1;;
    esac
done

# PROJECT_PATH=${PROJECT_PATH-"$HOME/workspace/${PROJECT}_${ROSDISTRO}"}

if [ ! -d "${PROJECT_PATH}" ]; then
    mkdir -p "${PROJECT_PATH}"

    if [ ! -d "${PROJECT_PATH}" ]; then
        echo "Could not create ${PROJECT}'s home"
        exit 1
    fi
fi

if [ ! -d "${PROJECT_PATH}/.ssh" ]; then
    if [ ! -d "${HOME}/.ssh" ]; then
        printf "Please, setup ssh keys before running the container\n"
        exit 1
    fi
    cp -R "${HOME}/.ssh" "${PROJECT_PATH}/"
fi

if [ ! -f "${PROJECT_PATH}/.gitconfig" ]; then
    if [ ! -f "${HOME}/.gitconfig" ]; then
        printf "Please, setup git before running the container\n\n"
        printf "Use: \n"
        printf "git config --global user.name \"John Doe\"\n"
        printf "git config --global user.email johndoe@example.com\n"

        exit 1
    fi
    cp "${HOME}/.gitconfig" "${PROJECT_PATH}/"
fi

if [ ! "${DISABLE_X}" = "1" ]; then
    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth
    touch "${XAUTH}"
    xauth nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -

    DISPLAY_ARGS+=("--volume=${XSOCK}:${XSOCK}:rw"
                    "--volume=${XAUTH}:${XAUTH}:rw"
                    "--env=XAUTHORITY=${XAUTH}"
                    "--env=DISPLAY")
fi

if [ -z "${CONTAINER_USER}" ]; then
    CONTAINER_USER=${PROJECT}_${ROSDISTRO}
fi

if [ ! "$(docker ps -q -f name=${PROJECT}_${ROSDISTRO})" ]; then
    if [ ! "$(docker ps -aq -f status=exited -f name=${PROJECT}_${ROSDISTRO})" ]; then
        echo "Create docker"
        docker create -it \
            --volume="${PROJECT_PATH}:/home/${CONTAINER_USER}:rw" \
            --volume="/etc/localtime:/etc/localtime:ro" \
            --volume="$(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK)" \
            --env="SSH_AUTH_SOCK=$SSH_AUTH_SOCK" \
            --env="TERM" \
            --name ${PROJECT}_${ROSDISTRO} \
            --privileged \
            "${DISPLAY_ARGS[@]}" \
            ${PROJECT}_${ROSDISTRO}:devel
    fi
    docker start -ai ${PROJECT}_${ROSDISTRO}
else
    docker exec -ti ${PROJECT}_${ROSDISTRO} /bin/bash
fi
