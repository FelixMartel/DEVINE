Testing the image quality
#########################

Their is som simple tests in `/test` that allows to validate the quality of the image segmentation.
In order to launch these tests, you can use the following commands:


* Launch the dashboard in order to see the segmentation (optional):

.. code-block:: bash

    $ roslaunch devine devine.launch launch_all:=false dashboard:=true

If you don't launch the `devine` package, you will need to start a `roscore`.

* Launch segmentation image processing:

.. code-block:: bash

    $ rosrun devine_image_processing segmentation.py

* Launch the test against the test.json file:

.. code-block:: bash

    tests/image_tests$ python2 image_segmentation.py -t test

