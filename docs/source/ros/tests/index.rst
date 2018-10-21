About the tests
###############

The tests are made with `Python unittest <https://docs.python.org/3/library/unittest.html>`_.

Running the unit tests suite
############################

In order to launch the devine unit tests, you can use the following commands:


* To launch all the nodes used by the tests:

.. code-block:: bash

    $ roslaunch devine_tests default.test

* Launch the unit tests suite:

.. code-block:: bash

    $ python DEVINE/tests/src/devine_tests/test_suite.py

*Make sure all the node have been fully initalized before running the tests*.

Launching only one test case
===========================

Each `test_*.py` file correspond to a test case (a list of tests that are related).

Each one of theses files can be run individually like so:

.. code-block:: bash

    $ python DEVINE/tests/src/devine_tests/*/test_*.py


Running the unit tests with catkin
##################################

A simpler way to run the tests would be to start them with catkin:

.. code-block:: bash

    $ catkin_make run_tests

This will launch all the necessary nodes and run the tests.
However, as `"there is no way to externally know when a node is fully initialized" <http://wiki.ros.org/roslaunch/XML/node>`_,
the tests tend to fail using that method (*to be investigated*).


Adding tests
############

To add a serie of tests, simply copy the `testcase_template.py` into your test folder, then import your test case into `test_suite.py`.