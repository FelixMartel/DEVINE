ARG base_docker=latest

FROM devineproject/devine-base:$base_docker

COPY . /usr/src/DEVINE

WORKDIR /usr/src/DEVINE

RUN bash -c ". scripts/installutils.sh && install_devine $(pwd)/.. $(pwd)"
