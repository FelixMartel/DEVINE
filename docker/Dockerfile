FROM devine-base

RUN \
# dashboard
    cd dashboard \
    && apt install -y ros-kinetic-rosbridge-server \
    && pip3 install -r requirements.txt \
    && bash -ci 'npm install && npm run build' \
# guesswhat
    && cd ../guesswhat \
    && wget https://github.com/projetdevine/static/releases/download/v0.0.1/weights.zip \
    && unzip weights.zip -d devine_guesswhat/data \
    && rm -f weights.zip \
# image_processing
    && cd ../image_processing \
    && pip3 install Cython \
    && pip3 install scikit-image bson pymongo pycocotools keras==2.1.6 catkin_pkg rospkg \
    && wget https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5 \
    && wget http://download.tensorflow.org/models/vgg_16_2016_08_28.tar.gz \
    && tar xzf vgg_16_2016_08_28.tar.gz \
    && rm -f vgg_16_2016_08_28.tar.gz \
# kinect
    && cd ../kinect \
    && apt install ros-kinetic-openni-launch ros-kinetic-openni-camera ros-kinetic-openni-description ros-kinetic-compressed-image-transport \
# game_system
    && cd ../game_system \
    && pip install transitions \
# snips
    && pip install paho-mqtt 

# catkin
RUN cd /usr \
    && . /opt/ros/kinetic/setup.sh \
    && catkin_make \
    && echo ". /opt/ros/kinetic/setup.sh" >> ~/.bashrc \
    && echo "export ROS_PACKAGE_PATH=/usr/src:$ROS_PACKAGE_PATH" >> ~/.bashrc
