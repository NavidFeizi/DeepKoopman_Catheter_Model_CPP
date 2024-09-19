<div align="center">

# Deep Koopman Modeling for Tendon Driven Catheters

</div>

This code comprised of deep koopman based model of a tendon driven catheter. The Koopman model was calibrated based on the dataset generated using a ground truth Cosserat Rod model. All calibratios are done for the bending sectio of Biosense Webster Termocouple Abalation catheter. 
The model gets tendon displacement as input and computes the states of the system, inclusing position and velocity of the distal end of the bending section in the actuation plane, so the model is currently 2D.

## Building Requirements

ensure that you have the following libraries installed in your system:

### Deep-Koopman model

* [Libtorch](https://pytorch.org/)
* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/)

### Cosserat Rod model

* [Boost](https://www.boost.org/)
* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/)
* [LAPACK](http://www.netlib.org/lapack/)
* [NLopt](https://nlopt.readthedocs.io/en/latest/)














I have tested number of torch threads and numebr of cores to achive the most consistent and optimized elapsed time.
The best performance was achived with two torch threads and two CPU cores.
So set Core Affinity at launch -> run the built code as below:

$ taskset -c 0,1 ./CatheterRobot

For better and more consistent performance you can isolate code 0 and 1 by following the instruction below. 
However, I have noticed that multi-threading with torch significantly reduces the performance when using core isolaton!
So use torch::set_num_threads(1);

$ sudo nano /etc/default/grub

Add the isolcpus parameter to the GRUB_CMDLINE_LINUX_DEFAULT line. For example, to isolate cores 1 and 2:

$ GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=1,2"
$ sudo update-grub
$ sudo reboot

Check CPU activity using htop or another system monitor:

$ htop

dedicating more core also improves the performance
$ taskset -c 0,1,2,3 ./CatheterRobot