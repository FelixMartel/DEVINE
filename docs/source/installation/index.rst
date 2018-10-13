Installation
############

DEVINE is a project with **many** dependencies such as ``ROS``. 

We recommand you install it on a fresh Ubuntu installation. 

Fresh Ubuntu 16.04 LTS
======================

* Create a ``catkin workspace`` directory like explained in the `ROS tutorial`_
* Create ``src`` directory under it
* Clone the `DEVINE repository`_ in ``src/`` **Make sure not to rename the repository**
* Navigate to ``DEVINE/scripts``
* Run the following command

.. code-block:: bash

    ./install.sh {Path/To/src} {Path/To/src/Repo} 

You may have to manually run some commands from the ``/scripts/installutils.sh`` but the heavy lifting will be done.

Docker
======

Docker is an application which runs a program in an isolated environment with its dependencies, akin to a virtual machine. Docker is portable, lightweight and allows for compatibility.

How to get started
------------------

Navigate to the docker folder. Run `build.sh`. This will get the devine-base image and build the devine image. The devine-base image contains all of the projects dependencies and can be rebuilt if necessary. The devine image contains the code. This may seem inane, but speeds up future builds if you were to modify the code.

Once the build is complete, you can validate by running ``sudo docker images``. One docker should be named DEVINE.

With an image in hand, run the command ``./run.sh``. This will launch an instance of your docker image. You will arive in a ubuntu like terminal which has the same layout as the code base. To exit, use ctrl+d. 

Tools
-----

* ``sudo docker container ls``: Lists all containers currently running
* ``sudo docker exec -it {containerId} bash``: starts another bash in a given docker container
* ``docker_cleaner.sh``: Script found in the script folder which removes untagged (excess) docker images
* ``docker cp {path/to/filename} {containerId}:{Destination/Path/}`` copy a file into a specific docker image

.. _DEVINE repository: https://github.com/FelixMartel/DEVINE
.. _ROS tutorial: https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
