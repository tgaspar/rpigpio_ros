# RPIGPIO_ROS

This is a simple ROS package to help you expose some of the Raspberry Pi GPIO's functionalities to as standard ROS interfaces.

# Install

1. Clone this repository to your workspace.
2. Build the package using `catkin_make`, `catkin build` or `colcon`.
3. Source the workspace

# Running the service server

This package provides a node that acts as a server that can dynamicaly create new ROS Services to manipulate with the GPIOs.

Running the command
```
$ rosrun rpigpio_ros gpio_control.py
```
will start the server and expose the `generate_rpi_services` service of type Trigger.

Upon calling this service, the server will read the desired configuration stored on the parameter server at the address `new_gpio_configuration`.

Let's put some configuration parameters to the parameter server manually with the following commands
```
$ rosparam set new_gpio_configuration/door_lock/name/ 'set_lock'
$ rosparam set new_gpio_configuration/door_lock/address/ 3
$ rosparam set new_gpio_configuration/brake/name/ 'set_brake'
$ rosparam set new_gpio_configuration/brake/address/ 4
```
Let's display the content of the parameter `new_gpio_configuration`
```
$ rosparam get new_gpio_configuration
brake: {address: 4, name: set_brake}
door_lock: {name: set_lock, set_lock: 3}
```

This read as: if you want to activate the `brake` you should call the `set_brake` service (type `SetBool`) that will change the state of the GPIO nr. `4`.


Now that the configuration is on the parameter server, you can call the service
```
rosservice call /generate_rpi_services
```

The services `set_brake` and `set_lock` should be visible:
```
$ rosservice list
...
/set_brake
/set_lock
...
```

# Command line arguments

You can start the server with a set of arguments.

## Reading configuration from file 

Starting the server with the `-c` followed by the location of the configuration file (YAML) will expose the services listed there. An example of the configuraiton file is provided with this package in `conf/example.yml`

```
$ rosrun rpigpio_ros gpio_control.py -c $(rospack find rpigpio_ros)/con f/example.yml
```

## Providing mappings directly as arguments
You can start the server specify which services you want to have linked to which GPIOs with the `-n` and `-a` flag.

Running
```
$ rosrun rpigpio_ros gpio_control.py -n set_lock set_brake -a 4 3
```
will have the same result as (in terms of exposing the services) as setting the parameters as descibed above.

# Starting the server at boot

To manage this server in the background, we will be using [supervisor](http://supervisord.org/). If you already have it installed, you can skip the next step

## Install supervisor

If you haven't done it yet, run:
```
$ sudo apt-get update
```
Then install `supervisor`:
```
$ sudo apt-get install supervisor
```

## Configure the supervisor configuration file

**Note**: The following instructions are to be used as a template. Don't simply copy-paste stuff in the configuration file and expect it to work. Read carefuly and adjust according to your environment

To simplify your life, we included a script (`util/supervisor.sh`) that can be run from `supervisor`. This script takes care to set up the environment variables so that the running of the commands is smoother. Pay extra attention to the line that specifies the `ROS_MASTER_URI` environment variable and adjust it to your needs.

Open `supervisor`'s configuration file (`/etc/supervisor/supervisord.conf`) with your favorite editor (don't forget to `sudo`) and add the following lines at the bottom:
```
[program:rpigpio_srv]
command=/bin/bash /home/<USER>/<ROS_WORKSPACE>/src/rpigpio_ros/util/supervisor.sh
user=<USER>
autorestart=true
startretries=100
stdout_logfile=</path/to/log/file.log>
stderr_logfile=</path/to/errorlog/file.log>
```

Replace `<USER>` with the name of your user.

Run supervisor:
```
$ supervisor
```



