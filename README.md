# ball_delivery
This repo is for ball delivery control of SubT competition.

# setup
```
pi3 $ cd ~/
pi3 $ git clone https://github.com/brianfire/ball_delivery.git
pi3 $ cd ~/ball_delivery
pi3 $ source environment.sh
pi3 $ cd ~/ball_delivery/catkin_ws/
pi3 $ catkin_make
```

# how to run

## deliver or launch
```
pi3 $ cd ~/ball_delivery
pi3 $ source environment.sh
pi3 $ sudo chmod 777 /dev/ttyACM0
pi3 $ roslaunch launcher switcher.launch 
```

## activate
```
pi3 $ cd ~/ball_delivery
pi3 $ source environment.sh
pi3 $ roslaunch demo ball_delivery.launch 
```