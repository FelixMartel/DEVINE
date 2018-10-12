How to generate the documentation
#################################

Using a Docker
==============

We use a `Sphinx Server`_ as the container.

The documentation will automatically refresh if you change the docs!

* navigate to ``/docs``
* run ``./gen_doc.sh``
* In your browser, navigate to ``http://localhost:8000/``
* profit!


Local Env
=========

Environment Setup
-----------------

.. code-block:: bash

    sudo pip install sphinx
    sudo pip install sphinx-rtd-theme
    sudo pip install sphinxcontrib-plantuml
    brew install graphviz

Build
-----

* navigate to ``/docs``;
* `make html`
* verify build succeeded;
* open `/build/html/index.html` in your favorite browser;
* profit!


.. _Sphinx Server: https://github.com/dldl/sphinx-server