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

    def __init__(self):
        self.ros_init()
        self.get_params()
        self.connect()
        self.authenticate()

    def get_params(self):
        local =rospy.get_param("~")
        self.ip = local.get("ip", self.DEFAULT_IP)
        self.port = local.get("port", self.DEFAULT_PORT)
        self.authentication = local.get("authentication", self.DEFAULT_AUTHENTICATION)
        self.id = local.get("id", self.DEFAULT_ID)
        self.f_max = local.get("f_max", self.DEFAULT_F_MAX)
        self.initial_force = local.get("initial_force", 0)
        assert self.check_force(self.initial_force)
        self.ramp_duration = local.get("ramp_duration", 0)
        assert self.check_ramp_duration(self.ramp_duration)
        self.payload = local.get("payload", 0)
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
        data = bytes(self.authentication + self.TERMINATOR)
        self.sock.send(data)
        return data == self.sock.recv(self.BUFSIZE)

    def send_command(self, force):
        self.sock.send(bytes(self.DELIMINATOR.join((self.id, self.initial_force, self.ramp_duration, self.payload, 0)) + self.TERMINATOR))

    def disconnect(self):
        self.sock.send(self.DISCONNECT)

    def command_handler(self, msg):
        self.send_command(msg.data)
        self.initial_force = msg.data
        self.handle_telem()

    def handle_telem(self):
        data = self.recv_telem()
        telem = ACFTelem()
        telem.id = data[0]
        telem.force = data[1]
        telem.position = data[2]
        telem.in_contact = bool(data[3])
        telem.in_error = data[4] > 0
        telem.errors = [bool(data[4] & (1<<i) for i in range(self.ERROR_CODE_BIN_LENGTH))]
        telem.error_messages = [self.ERROR_MESSAGES[i] for i, error in enumerate(telem.errors) if error]
        self.telem_pub.publish(telem)

    def recv_telem(self):
        return self.sock.recv(self.BUFSIZE).strip(self.TERMINATOR).split(self.DELIMINATOR)

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

    def set_payload(self, req):
        duration = req.duration.to_secs()
        if not self.check_ramp_duration(duration):
            return SetDurationResponse(False, "Invalid duration.")
        self.ramp_duration = duration
        return SetDurationResponse(True, "")

    def ros_init(self):
        rospy.init_node("ACF")
        rospy.Subscriber("~/ACF/force", Float32, self.command_handler)
        self.telem_pub = rospy.Publisher("~/ACF/telem", ACFTelem, queue_size=5)
        rospy.Service("~/ACF/set_payload", SetFloat, self.set_payload)
        rospy.Service("~/ACF/set_f_zero", SetFloat, self.set_f_zero)
        rospy.Service("~/ACF/set_t_ramp", SetDuration, self.set_t_ramp)

if __name__ == "__main__":
    FerroboticsACF()
    rospy.spin()
