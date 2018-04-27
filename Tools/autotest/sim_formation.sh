#! /bin/bash
#example : sim_formation.sh 0 172.30.110.1:14554 ~/SIM/ardupilot_gazebo/worlds/test.world 
#
#
#
#
#
tcp_port=$((5760+(10*$1)))
sitl_port=$((5501+(10*$1)))

# first setup gazebo at back
gazebo  $3 &

# second setup arducopter bin at back
../build/sitl/bin/arducopter -S -I$1 --home -35.363261,149.165230,584,353 --model gazebo-iris --speedup 1 --defaults ../Tools/autotest/default_params/copter.parm,../Tools/autotest/default_params/gazebo-iris.parm  & 

# third setup mavproxy 
mavproxy.py --master tcp:127.0.0.1:$tcp_port --sitl 127.0.0.1:$sitl_port --out 127.0.0.1:14560 --out 127.0.0.1:14561 --out $2

