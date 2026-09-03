## Gazebo Cmd

### Open Gz Sim
```
gz sim <path to .sdf file>
```

### Topic
```
gz topic -l // list all topic
gz topic -i -t /world/world_demo/stats // check topic information
gz topic -e -t /world/world_demo/stats // check topic data
```

### Control
```
gz topic -t /model/turtlebot3_burger/cmd_vel -m gz.msgs.Twist -p 'linear: {x:0.2}, angular: {z:0.0}'
```