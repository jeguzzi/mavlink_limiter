#!/usr/bin/env python

from __future__ import division
import rospy
from mavros_msgs.msg import Mavlink
from mavros_msgs.srv import CommandLong
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
from collections import defaultdict

from mavros.mavlink import mavutil

msg_prefix = 'MAVLINK_MSG_ID_'

_messages = {k: v for k, v in mavutil.mavlink.__dict__.items() if msg_prefix in k}
_ids = {v: k for k, v in _messages.items()}

MAV_CMD_SET_MESSAGE_INTERVAL = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL

# print('MAV_CMD_SET_MESSAGE_INTERVAL', MAV_CMD_SET_MESSAGE_INTERVAL)
#
# print(_messages)
# print(_ids)


def msg_id(name):
    if name == 'ALL':
        return 'ALL'
    return _messages.get(msg_prefix + name, None)


def msg_name(_id):
    if _id == 'ALL':
        return 'ALL'
    if _id in _ids:
        name = _ids[_id]
        return name.split(msg_prefix)[-1]
    return None


class Limiter(object):

    def __init__(self):
        rospy.init_node('mavlink_limiter', anonymous=False)
        self.pub = rospy.Publisher('mavlink/from/filtered', Mavlink, queue_size=10)
        rospy.wait_for_service('mavros/cmd/command')
        self.cmd_srv = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
        self.rates = rospy.get_param('~rates')
        self.tol = rospy.get_param('~tolerance', 0.1)
        self.pub_rates = {}
        for k, v in self.rates.items():
            i = msg_id(k)
            if i is None:
                rospy.logwarn('Mavlink msg %s%s not found', msg_prefix, k)
                continue
            if 'input' in v and i != 'ALL':
                rate = v['input']
                self.set_rate(i, rate)
            if 'output' in v:
                rate = v['output']
                self.pub_rates[i] = rate

        rospy.loginfo('Publihing rate %s', self.pub_rates)

        self.last_message = {}
        self.reset_counters()
        self.diagnostic_period = 4

        rospy.Subscriber('mavlink/from', Mavlink, self.has_received_mavlink)

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID('MAVLINK limiter')
        self.updater.add("Msg", self.update_msg_diagnostic)

        rospy.Timer(rospy.Duration(self.diagnostic_period), lambda evt: self.updater.update())
        rospy.spin()

    def rate_for(self, msg_id):
        return self.pub_rates.get(msg_id, self.pub_rates.get('ALL', -1))

    def should_send(self, msg_id):
        rate = self.rate_for(msg_id)
        # rospy.loginfo('Should send %s %f', msg_id, rate)
        if rate == -1:
            return True
        if rate == 0:
            return False
        if msg_id not in self.last_message:
            return True
        dt = rospy.Time.now() - self.last_message[msg_id]
        # rospy.loginfo('delay %f > %f?', dt.to_sec(), 1 / rate)
        return dt.to_sec() > (1 / rate) - self.tol

    def has_received_mavlink(self, msg):
        msg_id = msg.msgid
        self.in_msgs[msg_id] += 1
        if self.should_send(msg_id):
            self.pub.publish(msg)
            self.out_msgs[msg_id] += 1
            self.last_message[msg_id] = rospy.Time.now()

    def reset_counters(self):
        self.in_msgs = defaultdict(lambda: 0)
        self.out_msgs = defaultdict(lambda: 0)

    def set_rate(self, msg_id, rate):

        if rate == 0:
            us = -1
        elif rate == -1:
            us = 0
        else:
            us = 1e6 / rate
        rospy.loginfo('Set rate %d to %f (%d us)', msg_id, rate, us)
        r = self.cmd_srv(False, MAV_CMD_SET_MESSAGE_INTERVAL, True, msg_id, us, 0, 0, 0, 0, 0)
        rospy.loginfo('Response %s', r)

    def update_msg_diagnostic(self, stat):
        status = DiagnosticStatus.OK
        stat.summary(status, "")
        for i, v in sorted(self.in_msgs.items()):
            name = msg_name(i)
            if not name:
                rospy.logwarn('Mavlink msg id %s not found', i)
                continue
            s = 'IN {0:.1f} 1/s\t'.format(self.in_msgs[i] / self.diagnostic_period)
            if i in self.out_msgs:
                s += 'OUT {0:.1f} 1/s\t'.format(self.out_msgs[i] / self.diagnostic_period)
            stat.add(name, s)
        s = 'IN {0:.1f} 1/s\t'.format(sum(self.in_msgs.values()) / self.diagnostic_period)
        s += 'OUT {0:.1f} 1/s\t'.format(sum(self.out_msgs.values()) / self.diagnostic_period)
        stat.add('TOTAL', s)
        self.reset_counters()
        return stat


if __name__ == '__main__':
    try:
        Limiter()
    except rospy.ROSInterruptException:
        pass
