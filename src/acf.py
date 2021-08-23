#!/usr/bin/env python

import rospy
from ferrobotics_acf.msg import ACFTelem
from ferrobotics_acf.srv import (
    SetFloat,
    SetFloatResponse,
    SetDuration,
    SetDurationResponse,
)
from std_msgs.msg import Float32
import socket
import sys

bytes_args = ()
if sys.version_info.major >= 3:
    bytes_args = ("ASCII",)


class FerroboticsACF:
    DEFAULT_IP = "192.168.99.1"
    DEFAULT_PORT = 7070
    DEFAULT_AUTHENTICATION = "ferba"
    DEFAULT_ID = 1040
    DEFAULT_F_MAX = 100
    TERMINATOR = "k"
    DISCONNECT = "c"
    BUFSIZE = 128
    DELIMINATOR = " "
    DEFAULT_TIMEOUT = 5
    MAX_T_RAMP = 10
    PAYLOAD_SHARE = 0.1
    ERROR_CODE_BIN_LENGTH = 8
    ERROR_MESSAGES = [
        "valve error",
        "sensor error",
        "Head and controller are not compatible",
        "set_f_target cannot be reached",
        "set_f_zero set to 0",
        "one or more input values out of range",
        "INCAN no communication",
        "Reading of INCAN parameters not complete",
    ]

    def __init__(self):
        rospy.init_node("ACF")
        self.get_params()
        self.connect()
        assert self.authenticate()
        self.ros_setup()

    def get_params(self):
        self.ip = rospy.get_param("~ip", self.DEFAULT_IP)
        self.port = rospy.get_param("~port", self.DEFAULT_PORT)
        self.authentication = rospy.get_param(
            "~authentication", self.DEFAULT_AUTHENTICATION
        )
        self.id = rospy.get_param("~id", self.DEFAULT_ID)
        self.f_max = rospy.get_param("~f_max", self.DEFAULT_F_MAX)
        self.initial_force = rospy.get_param("~initial_force", 0)
        assert self.check_force(self.initial_force)
        self.ramp_duration = rospy.get_param("~ramp_duration", 0)
        assert self.check_ramp_duration(self.ramp_duration)
        self.payload = rospy.get_param("~payload", 0)
        assert self.check_payload(self.payload)

    def check_force(self, force):
        return -self.f_max <= force <= self.f_max

    def check_ramp_duration(self, duration):
        return 0 <= duration <= self.MAX_T_RAMP

    def check_payload(self, payload):
        return 0 <= payload <= self.PAYLOAD_SHARE * self.f_max

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))

    def authenticate(self):
        data = bytes(self.authentication + self.TERMINATOR, *bytes_args)
        self.sock.send(data)
        return data == self.sock.recv(self.BUFSIZE)

    def send_command(self, force):
        try:
            self.sock.send(
                bytes(
                    self.DELIMINATOR.join(
                        str(field)
                        for field in (
                            self.id,
                            force,
                            self.initial_force,
                            self.ramp_duration,
                            self.payload,
                            0,
                        )
                    )
                    + self.TERMINATOR,
                    *bytes_args
                )
            )
        except:
            rospy.logwarn("Reconnecting...")
            self.connect()
            assert self.authenticate()
            self.send_command(force)

    def disconnect(self):
        try:
            self.sock.send(self.DISCONNECT)
        except:
            pass

    def command_handler(self, msg):
        self.send_command(msg.data)
        self.initial_force = msg.data
        self.handle_telem()

    def handle_telem(self):
        data = self.recv_telem()
        telem = ACFTelem()
        telem.id = int(data[0])
        telem.force = float(data[1])
        telem.position = float(data[2])
        telem.in_contact = bool(data[3])
        telem.in_error = int(data[4]) > 0
        telem.errors = [
            bool(int(data[4]) & (1 << i)) for i in range(self.ERROR_CODE_BIN_LENGTH)
        ]
        telem.error_messages = [
            self.ERROR_MESSAGES[i] for i, error in enumerate(telem.errors) if error
        ]
        self.telem_pub.publish(telem)

    def recv_telem(self):
        return (
            self.sock.recv(self.BUFSIZE).decode().strip(self.TERMINATOR).split(self.DELIMINATOR)
        )

    def set_payload(self, req):
        if not self.check_payload(req.value):
            return SetFloatResponse(False, "Invalid payload value.")
        self.payload = req.value
        return SetFloatResponse(True, "")

    def set_f_zero(self, req):
        if not self.check_force(req.value):
            return SetFloatResponse(False, "Invalid force value.")
        self.initial_force = req.value
        return SetFloatResponse(True, "")

    def set_t_ramp(self, req):
        duration = req.duration.to_secs()
        if not self.check_ramp_duration(duration):
            return SetDurationResponse(False, "Invalid duration.")
        self.ramp_duration = duration
        return SetDurationResponse(True, "")

    def ros_setup(self):
        rospy.Subscriber("~/ACF/force", Float32, self.command_handler)
        self.telem_pub = rospy.Publisher("~/ACF/telem", ACFTelem, queue_size=5)
        rospy.Service("~/ACF/set_payload", SetFloat, self.set_payload)
        rospy.Service("~/ACF/set_f_zero", SetFloat, self.set_f_zero)
        rospy.Service("~/ACF/set_t_ramp", SetDuration, self.set_t_ramp)


if __name__ == "__main__":
    acf = FerroboticsACF()
    rospy.spin()
    acf.disconnect()
