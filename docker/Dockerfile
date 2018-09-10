FROM devineproject/devine-base

COPY . /usr/src/DEVINE

WORKDIR /usr/src/DEVINE

RUN \
# dashboard
    cd dashboard \
    && pip3 install -r requirements.txt \
    && bash -ci 'npm install && npm run build' \
# guesswhat
    && cd ../guesswhat \
    && unzip /data/weights.zip -d devine_guesswhat/data \
    && rm -f /data/weights.zip \
# image_processing
    && cd ../image_processing \
    && pip3 install Cython \
    && pip3 install scikit-image bson pymongo pycocotools keras==2.1.6 catkin_pkg rospkg \
    && ln -s /data/mask_rcnn_coco.h5 mask_rcnn_coco.h5 \
    && tar xzf /data/vgg_16_2016_08_28.tar.gz \
    && rm -f /data/vgg_16_2016_08_28.tar.gz \
# game_system
    && cd ../game_system \
    && pip2 install transitions \
# snips
    && pip2 install paho-mqtt \
# robot sim
    && cd ../robot_control \
    && mkdir /root/.rviz \
    && cp irl_control/irl_point.rviz /root/.rviz/default.rviz 

# catkin
RUN cd /usr \
    && . /opt/ros/kinetic/setup.sh \
    && rosdep init && rosdep update \
    && catkin_make \
    && echo ". /opt/ros/kinetic/setup.sh" >> ~/.bashrc \
    && echo "export ROS_PACKAGE_PATH=/usr/src:$ROS_PACKAGE_PATH" >> ~/.bashrc
