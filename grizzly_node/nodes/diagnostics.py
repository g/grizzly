#!/usr/bin/python

import roslib; roslib.load_manifest('grizzly_node')
import rospy

from grizzly_msgs.msg import RawStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32


class Diagnostics(object):
    def __init__(self):
        rospy.init_node('grizzly_diagnostics')

        self.pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        self.rate = rospy.Rate(rospy.get_param('~hz', 1))

        tmpl = "mcu: %s"
        self.status_power = DiagnosticStatus(name=tmpl % "Power Status")
        self.status_cooling = DiagnosticStatus(name=tmpl % "Cooling Status")
        self.status_faults = DiagnosticStatus(name=tmpl % "System Faults")
        self.msg = DiagnosticArray(status=[self.status_power, self.status_cooling, self.status_faults])
        self.latest_temp = 0

        rospy.Subscriber('status', RawStatus, self._status_callback)
        rospy.Subscriber('body_temp',Float32, self._temp_callback)
        self.statuses = []
    
    def _status_callback(self, msg):
        self.statuses.append(msg)

    def _temp_callback(self, msg):
        self.latest_temp = msg.data
    
    def latest(self, field_fn):
        return field_fn(self.statuses[-1])

    def avg(self, field_fn):
        values = map(field_fn, self.statuses)
        return sum(values) / len(values)

    def max(self, field_fn):
        values = map(field_fn, self.statuses)
        return max(values)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if len(self.statuses) < 1:
                # No messages received in this period. Bail out.
                continue
           
            voltage = self.avg(lambda m: m.voltage)
            current = self.avg(lambda m: m.user_current)
            self.status_power.message = "OK"
            self.status_power.level = DiagnosticStatus.OK
            if voltage < 48:
                self.status_power.message = "Voltage Critical"
                self.status_power.level = DiagnosticStatus.ERROR
            elif voltage < 50:
                self.status_power.message = "Voltage Low"
                self.status_power.level = DiagnosticStatus.WARN
            self.status_power.values = [
                    KeyValue('battery voltage (V)', str(voltage)),
                    KeyValue('payload current (A)', str(current)),
                    ]

            self.status_cooling.message = "OK"
            self.status_cooling.level = DiagnosticStatus.OK
            self.status_cooling.values = [
                    KeyValue('body temperature (deg C)', str(self.latest_temp)),
                    KeyValue('fans on', str(self.latest(lambda m: m.fans_on)))
                    ]

            error = self.latest(lambda m: m.error)
            self.status_faults.message = "OK"
            self.status_faults.level = DiagnosticStatus.OK
            if error != 0:
                self.status_faults.message = "see flags"
                self.status_faults.level = DiagnosticStatus.ERROR
            self.status_faults.values = [
                    KeyValue('e-stop', str((error & RawStatus.ERROR_ESTOP) != 0)),
                    KeyValue('reset required', str((error & RawStatus.ERROR_ESTOP_RESET) != 0)),
                    KeyValue('undervoltage', str((error & RawStatus.ERROR_UNDERVOLT) != 0)),
                    KeyValue('command timeout', str((error & RawStatus.ERROR_COMMAND_TIMEOUT) != 0)),
                    KeyValue('brake (pre-charge) detect', str((error & RawStatus.ERROR_BRK_DET) != 0)),
                    ]

            self.pub.publish(self.msg)
            self.statuses = []


if __name__ == "__main__": 
    Diagnostics().spin()
