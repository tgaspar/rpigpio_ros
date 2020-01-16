#!/usr/bin/python
import sys
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool

from raspgpio.srv import *
import time
from argparse import ArgumentParser
import yaml

CONF_PARAM = 'current_gpio_conf'

class GPIOControl(object):
    _config = {}
    _handlers = {}
    _gpio_services = []
    _topic_publishers = []

    def __init__(self, configuration):

        # Prepare the GPIO
        GPIO.setmode(GPIO.BCM)  # setup control map
        GPIO.setwarnings(False)  # set pin warnings to off

        # Announce what will be started
        rospy.loginfo("Starting the following services:\n{0}".format(configuration.keys()))
        rospy.loginfo("On the following GPIO addresses:\n{0}".format(configuration.values()))

        self._config = configuration
        self.conf_to_srv(configuration)
        # Write the current configuration to the parameter server
        self.write_conf_to_param()

        # Create the service that will create NEW GPIO services on request
        self._service_creator_srv = rospy.Service('generate_rpi_services', Trigger, self.create_service_cb)
        # print(self._service_creator_srv.resolved_name)
        
#         # Start the topic publisher
#         self._topic_publisher = rospy.Publisher('{0}_status'.format(service_name), Bool, latch=True, queue_size=10)

#         # Publish the first message and since it is latched it will stay like that untill the service is called
#         self._topic_publisher.publish(self._gpio_status)

#         rospy.loginfo("Setup complete.")

        rospy.spin()
    def create_service_cb(self, req):
        try:
            new_conf = rospy.get_param('new_gpio_configuration')
            msg = 'Updated services.'
            
            self.conf_to_srv(new_conf)
            rospy.delete_param('new_gpio_configuration')

        except KeyError:
            rospy.logwarn('Configuration is empty')
            msg = 'Configuration parameter empty.'
        except Exception as e:
            rospy.logerr('Unexpected exception occured:\n{}'.format(e))
            return[False, 'Error']
        
        return [True, msg]

    def conf_to_srv(self, conf):
        try:
            for srv in conf:
                print(srv)
                address = conf[srv]['address']
                name = conf[srv]['name']

                running_services = []
                for srv in self._gpio_services:
                    running_services.append(srv.resolved_name)                

                if address not in running_services:
                    rospy.loginfo(
                        'Creating service [{0}] mapped to GPIO [{1}]'.format(
                            name, address))

                    self._config[name] = {
                        'name': name,
                        'address': address
                    }

                    # Set the GPIO at the given address as output
                    GPIO.setup(address, GPIO.OUT)

                    # The GPIO should be LOW when the service starts
                    GPIO.output(address, int(False))

                    # Start the service
                    self._gpio_services.append(rospy.Service(name, SetBool, self.set_gpio))
                    # self._topic_publishers.append(
                    #     rospy.Publisher('{}'.format(name), Bool, latch=True, queue_size=10)

                else:
                    rospy.logwarn(
                        'Desired service [{0}] already mapped to a GPIO [{1}]'.format(
                            name, address))

            self.write_conf_to_param()

        except Exception as e:
            rospy.logerr('Could not read configuration. Reason:\n{}'.format(e))
            raise Exception('Bad configuration.')

    def write_conf_to_param(self):
        if rospy.has_param(CONF_PARAM):
            rospy.delete_param(CONF_PARAM)
        print(self._config)
        rospy.set_param(CONF_PARAM, self._config)

    def set_gpio(self, req):
        try:
            object_attributes = dir(req)
            header = req._connection_header
            srv_name = header['service']
            address = self._config[srv_name[1:]]['address']
            GPIO.output(address, int(req.data))
            msg = 'GPIO {0} set to {1}'.format(address, req.data)
            return [True, msg]
        except Exception as e:
            rospy.logerr('Could not process request:\n{}'.format(e))
            return [False, 'Error' ]


if __name__ == '__main__':
    parser = ArgumentParser(description='A server that provides ROS services to trigger GPIOs on the Raspberry Pi.')
    service_name = 'gpio_control'
    parser.add_argument("-n", "--name",  metavar='NAME', type=str, nargs='+',
                     help='Names of the services you would like to start.')
    parser.add_argument("-a", "--addr",  metavar='ADDRESS', type=int, nargs='+',
                     help='GPIO addresses to which you would like to map the services')
    parser.add_argument("-c", "--config",  metavar='CONFIGURATION', type=str, nargs=1,
                     help='Configuration file (YAML).')

    args = parser.parse_args()

    rospy.init_node('gpio_service_server')
    
    conf_args = {}
    conf_yaml = {}
    conf = {}
    try:

        if (args.name is None) ^ (args.addr is None):
            rospy.logerr('Bad inputs.')
            raise Exception('If you chose to specify the names at lunch, you also must provide the addreses and vice versa.')


        if (args.name is not None) and (args.addr is not None):

            if len(args.name) is not len(args.addr):
                rospy.logerr('The number of provided names do not match the number of provided addresses!')
                raise Exception('Bad inputs.')

            srv_names = (args.name)
            srv_addr = (args.addr)
            for name in srv_names:
                conf_args[name] = {
                    'name': name,
                    'address': srv_addr[srv_names.index(name)]
                }
                

        if args.config is not None:
            conf_yaml = yaml.load(file(args.config[0], 'r'))

        # Merge the dictionaries
        print(conf_yaml)
        print(conf_args)
        conf.update(conf_yaml)
        conf.update(conf_args)

        GPIOControl(conf)

    except Exception as e:

        rospy.logerr('Exception occured:\n{0}'.format(e))
        # rospy.logerr('Did you provide the right arguments? Probably not ... moron.')
        raise
