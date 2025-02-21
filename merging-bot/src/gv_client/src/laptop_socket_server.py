#!/usr/bin/env python3
import threading
from socketserver import BaseRequestHandler, UDPServer
import struct

import rospy
from std_msgs.msg import Header

from gv_client.msg import LaptopSpeed
from gullivutil import parse_packet

GV_POSITION_TOPIC = "gv_laptop"


def unpack_data(buf: bytearray, start: int) -> int:
    """Helper method to unpack big-endian uint32's from a buffer."""
    return struct.unpack('>I', buf[start:start+4])[0]


class LaptopPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView packets and publish data on a ROS topic.

    The handle() method is called when a 'request' (i.e. UDP packet) is
    received from GulliView on the roof system. The received data is
    unpacked and sent on the ROS topic that ``cls.Publisher`` is publishing on.
    """
    publisher = None  # Static reference to ROS topic publisher
    listen_tag_id = None  # Tag ID to listen for

    def handle(self):
        # Receiving binary detection data from GulliView
        recv_buf = bytearray(self.request[0])
        print(f'[*] Recieved packet {recv_buf}')
        # Parse data in format of (>IfIf)

        main_id = unpack_data(recv_buf, 0)
        main_speed = struct.unpack('>f', recv_buf[4:8])[0]
        main_restart_ready = struct.unpack('>?', recv_buf[8:9])[0]
        ramp_id = unpack_data(recv_buf, 9)
        ramp_speed = struct.unpack('>f', recv_buf[13:17])[0]
        ramp_restart_ready = struct.unpack('>?', recv_buf[17:18])[0]

        print(f'[*] Main ID: {main_id}, Main Speed: {main_speed}')
        print(f'[*] Ramp ID: {ramp_id}, Ramp Speed: {ramp_speed}')

        print(f'[*] Main RESTART: {main_restart_ready}, Ramp RESTART: {ramp_restart_ready}')


        # ramp_endposition = struct.unpack('>?', recv_buf[16:20])[0]





        
        # print(f'[*] Main ID: {main_id}, Main endpos: {main_endposition}')
        # print(f'[*] Ramp ID: {ramp_id}, Ramp endpos: {ramp_endposition}')

        # packet = parse_packet(recv_buf)

        # Safety cutoff
        safe_speed = 0.8
        if(abs(main_speed) > safe_speed or abs(ramp_speed) > safe_speed):
            print()
            print('_'*32)
            print(f'[!] Safety Cut-off: speeds from UDP recieved: ({main_speed}, {ramp_speed})')
            print('_'*32)
            print()
            if(main_speed > 0):
                main_speed = safe_speed
            else:
                main_speed = 0.01
            if(ramp_speed > 0):
                ramp_speed = safe_speed
            else:
                ramp_speed = 0.01

        if self.listen_tag_id == main_id:
            header = Header()
            header.stamp = rospy.Time.now()
            # restart=main_restart_ready
            msg = LaptopSpeed(header=header, tag_id=main_id, speed=main_speed, restart=main_restart_ready)
            print(f'[!]msccccccccc: ({msg.restart},')

            self.publisher.publish(msg)
        elif self.listen_tag_id == ramp_id:
            header = Header()
            header.stamp = rospy.Time.now()
            # restart=ramp_restart_ready
            msg = LaptopSpeed(header=header, tag_id=ramp_id, speed=ramp_speed, restart=ramp_restart_ready)
            print(f'[!]msccccccccc: ({msg.restart},')

            self.publisher.publish(msg)

        return
        # Old code after this point
        
        for det in packet.detections:
            # If we aren't listening for all tags, and this is not the tag
            # we are listening for, skip it.
            if self.listen_tag_id != "all" and det.tag_id != self.listen_tag_id:
                continue

            header = Header()
            header.stamp = rospy.Time.from_sec(packet.header.timestamp / 1000)
            msg = LaptopSpeed(header=header, main_id=det.main_id, main_speed=det.main_speed, ramp_id=det.ramp_id, ramp_speed=det.ramp_speed)
#            print("theta = " + str(det.theta)) # DEBUG line, TODO remove later!
            print("[*] Msg recieved from laptop = " + str(det.main_id) + str(det.main_speed) + str(ramp_id) + str(ramp_speed)) # DEBUG line, TODO remove later!

            # Safety
            if (det.speed > 0.3):
                print(f'[!] Satefy cut-off, speed recieved: {det.speed}]')
                return
            self.publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node('gulliview', anonymous=True)
    rospy.loginfo(f"Started gulliview node")

    topic = rospy.names.canonicalize_name(GV_POSITION_TOPIC)
    rospy.loginfo(f"Setting up publisher on {topic}")

    host = rospy.get_param("~host", default="0.0.0.0")
    port = rospy.get_param("~port", default=2222)
    listen_tag_id = rospy.get_param("~tag_id", default="all")

    # Set static variables on packet handler class to pass information to its instances
    LaptopPacketHandler.publisher = rospy.Publisher(topic, LaptopSpeed, queue_size=10)
    LaptopPacketHandler.listen_tag_id = listen_tag_id

    rospy.loginfo(f"Starting UDP server on {host}:{port}, listening for tag ID: {listen_tag_id}")
    with UDPServer((host, port), LaptopPacketHandler) as server:
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        # Spin main thread (the ROS node) until shutdown
        rospy.spin()

        # Shut down server when node shuts down
        rospy.loginfo("Node received shutdown signal, shutting down server")
        server.shutdown()
        rospy.loginfo("Server shutdown, exiting")
