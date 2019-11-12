## Hydroponics ROS

ROS stuff for hydroponics project with cable robot + arm

IMPORTANT

### Trossen Pincher
To run, connect to computer and run
```
roslaunch pincher_test pincher.launch
```

Then, run `scripts/gerry00_armcontroller.py`.

**IMPORTANT**: you can damage the robot if not careful.  Be careful that you never send bad commands.

If you want to change parameters, edit `pincher.yaml` as appropriate.