Overview
++++++++

提供機能
--------

- HSRBでMoveItを使用するための、launch群を提供する
- HSRBでのMoveItサンプル群を提供する

How to use
++++++++++

Launchファイル
----------------

Rvizのみのシミュレータで動作確認する場合は、以下のlaunchを起動する。
(実機や、Gazeboがなくても確認できる)

.. code-block:: bash

   roslaunch hsrb_moveit_config hsrb_demo.launch

MoveItから直接、実機、Gazebo等実際のコントローラに対して指令を発行するには、以下のlaunchを起動する。

.. code-block:: bash

   roslaunch hsrb_moveit_config hsrb_demo_with_controller.launch

サンプル
----------------

いくつかのサンプルがデフォルトでインストールされている。
``demo.launch`` を起動したあと、 ``rosrun`` で実行が可能である。
実行できるサンプルは、下記のように ``[TAB]`` を押し補完させることで確認が可能である。

.. code-block:: bash

   rosrun hsrb_moveit_config moveit_[TAB]

Internal
++++++++

.. ifconfig:: internal

   * controller_managerのテストをしていないのでテストするべき
