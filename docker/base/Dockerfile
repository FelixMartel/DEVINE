FROM guesswhat

COPY . /usr/src/DEVINE

WORKDIR /usr/src/DEVINE

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    && sh -c 'echo "deb http://debian.snips.ai/jessie stable main" > /etc/apt/sources.list.d/snips.list' \
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-keys F727C778CCB0A455 \
    && apt update \
    && apt install -y ros-kinetic-desktop-full \
    && apt install -y snips-platform-voice \
    && apt install -y git \
    && curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash \
    && . ~/.bashrc \
    && nvm install --lts

