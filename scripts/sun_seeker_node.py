#!/usr/bin/env python

import os
import math
import rospy

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3
from skyfield.api import PlanetaryConstants, Loader, load

load = Loader(os.path.dirname(os.path.abspath(__file__)) + '/../include')

sim_time = 0
msg_timeout = 0.5
last_msg_time = 0.0

def sim_time_callback(data):
    last_msg_time = rospy.get_time()
    sim_time = data.clock

def run():
    rospy.init_node('sun_seeker_node', anonymous=True)
    rospy.Subscriber('/clock', Clock, sim_time_callback)

    pub = rospy.Publisher('/sun_seeker/vector', Vector3, queue_size=1)
    rate = rospy.Rate(0.1)

    eph = load('de421.bsp')
    sun = eph['sun']
    moon = eph['moon']

    path = os.path.realpath(__file__)
    pc = PlanetaryConstants()
    pc.read_text(load('moon_080317.tf'))
    pc.read_text(load('pck00008.tpc'))
    pc.read_binary(load('moon_pa_de421_1900-2050.bpc'))

    frame = pc.build_frame_named('MOON_ME_DE421')
    place = moon + pc.build_latlon_degrees(frame, 90.0, 180.0)

    sun_vector_msg = Vector3()

    while not rospy.is_shutdown():
        ts = load.timescale()
        t = ts.utc(2020, 4, 23)

        sunpos = place.at(t).observe(sun).apparent()
        alt, az, distance = sunpos.altaz()

        sun_vector_msg.x = - math.sin(az.radians) * math.cos(alt.radians)
        sun_vector_msg.y = - math.cos(az.radians) * math.cos(alt.radians)
        sun_vector_msg.z = math.sin(alt.radians)

        if rospy.get_time() - last_msg_time < msg_timeout:
            pub.publish(sun_vector_msg)
        else:
            rospy.logwarn('Timeout {0:.1f} seconds: Is /clocks publishing?'.format(msg_timeout))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
