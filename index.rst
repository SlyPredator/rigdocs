.. RIG Docs documentation master file, created by
   sphinx-quickstart on Fri Jan 23 18:07:12 2026.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. docs/index.rst

RIG Documentation
================================

Documentation and operational guides for hardware platforms, sensors, and state estimation pipelines maintained by the **Robotics Interest Group (RIGNITC)**.

.. grid:: 1 2 2 3
   :gutter: 3

   .. grid-item-card:: Svan M2 Quadruped
      :link: content/m2/etiquette
      :link-type: doc

      Operational safety, static network setup, DDS telemetry discovery, Joystick & API control, and simulation.
      
      +++
      View Svan M2 Guides »

   .. grid-item-card:: Sensor Calibration
      :link: content/kalibr/index
      :link-type: doc

      Camera-IMU calibration procedures using Dockerized Kalibr with Intel RealSense D435i and Pixhawk IMU.
      
      +++
      View Kalibr Guide »

   .. grid-item-card:: Visual Inertial Odometry
      :link: content/rovio/index
      :link-type: doc

      Robust Visual Inertial Odometry (ROVIO) deployment, custom configurations, and real-time RViz path trajectory mapping.
      
      +++
      View ROVIO Guide »

.. toctree::
   :maxdepth: 2
   :caption: Svan M2
   :hidden:

   content/m2/etiquette
   content/m2/receiving_topics
   content/m2/visualising_data
   content/m2/controlling_robot
   content/m2/simulation

.. toctree::
   :maxdepth: 2
   :caption: Robotics & Sensors
   :hidden:

   content/kalibr/index
   content/rovio/index

.. toctree::
   :maxdepth: 2
   :caption: Test
   :hidden:

   content/test/index