URCAP_VERSION=1.0.5 # latest version as if writing this
ROBOT_MODEL=UR5e

docker pull universalrobots/ursim_e-series
mkdir -p "${HOME}"/.ursim/programs
mkdir -p "${HOME}"/.ursim/urcaps
curl -L -o "${HOME}"/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.urcap \
  https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar
docker run --rm -it -e ROBOT_MODEL=${ROBOT_MODEL} \
    -p 2222:2222 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
    -p 29999:29999 -p 30004:30004 -p 5900:5900 -p 6081:6080 \
    -v "${HOME}"/.ursim/urcaps:/urcaps \
    -v "${HOME}"/.ursim/programs:/ursim/programs \
    --name ursim universalrobots/ursim_e-series

