#!/usr/bin/env python3

import os
import sys
import json

import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import RTCM
from nmea_msgs.msg import Sentence

from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.srv import NtripClientConnect, NtripClientConnectResponse
from ntrip_client.srv import NtripClientSettings, NtripClientSettingsResponse
from ntrip_client.srv import NtripClientStatus, NtripClientStatusResponse

# If no RTCM received for `kKeepAlivePeriod` seconds, connection will be re-opened
kKeepAlivePeriod = 5.0
kAttemptReconnectTimeout = 3.0

class NTRIPRos:
  def __init__(self):
    # Read a debug flag from the environment that should have been set by the launch file
    try:
      self._debug = json.loads(os.environ["NTRIP_CLIENT_DEBUG"].lower())
    except:
      self._debug = False

    # Init the node and read some mandatory config
    if self._debug:
      rospy.init_node('ntrip_client', anonymous=True, log_level=rospy.DEBUG)
    else:
      rospy.init_node('ntrip_client', anonymous=True)
    host = rospy.get_param('~host', '127.0.0.1')
    port = rospy.get_param('~port', '2101')
    mountpoint = rospy.get_param('~mountpoint', 'mount')

    # Optionally get the ntrip version from the launch file
    ntrip_version = rospy.get_param('~ntrip_version', None)
    if ntrip_version == '':
      ntrip_version = None

    # If we were asked to authenticate, read the username and password
    username = None
    password = None
    if rospy.get_param('~authenticate', False):
      username = rospy.get_param('~username', None)
      password = rospy.get_param('~password', None)
      if username is None:
        rospy.logerr(
          'Requested to authenticate, but param "username" was not set')
        sys.exit(1)
      if password is None:
        rospy.logerr(
          'Requested to authenticate, but param "password" was not set')
        sys.exit(1)

    # Read an optional Frame ID from the config
    self._rtcm_frame_id = rospy.get_param('~rtcm_frame_id', 'odom')

    # Setup the RTCM publisher
    self._rtcm_timer = None
    self._rtcm_pub = rospy.Publisher('rtcm', RTCM, queue_size=10)

    # Setup connect request server
    self._connect_server = rospy.Service("/ntrip_client_connect", NtripClientConnect, self.handle_connect_srv)
    self._settings_server = rospy.Service("/ntrip_client_settings", NtripClientSettings, self.handle_settings_srv)
    self._connect_server = rospy.Service("/ntrip_client_status", NtripClientStatus, self.handle_status_srv)

    # Initialize the client
    self.init_client(host, port, mountpoint, ntrip_version, username, password)

    # Keep-alive purposes
    self._last_rtcm_time = rospy.Time.now()
    self._last_reconnect_attempt_time = self._last_rtcm_time

    # Settings
    self._nmea_up = False

  def init_client(self, host, port, mountpoint, ntrip_version, username, password):
    self._client = NTRIPClient(
      host=host,
      port=port,
      mountpoint=mountpoint,
      ntrip_version=ntrip_version,
      username=username,
      password=password,
      logerr=rospy.logerr,
      logwarn=rospy.logwarn,
      loginfo=rospy.loginfo,
      logdebug=rospy.logdebug
    )

  def run(self):
    # Connect the client
    if not self._client.connect():
      rospy.logerr('Unable to connect to NTRIP server')

    # Setup our subscriber
    self._nmea_sub = rospy.Subscriber('nmea', Sentence, self.subscribe_nmea, queue_size=10)

    # Start the timer that will check for RTCM data
    self._rtcm_timer = rospy.Timer(rospy.Duration(0.1), self.publish_rtcm)

  def stop(self):
    rospy.loginfo('Stopping RTCM publisher')
    if self._rtcm_timer:
      self._rtcm_timer.shutdown()
      self._rtcm_timer.join()
    rospy.loginfo('Disconnecting NTRIP client')
    self._client.disconnect()

  def handle_connect_srv(self, req):
    self.stop()
    if req.is_connect:
      try:
        self.init_client(
          req.host,
          int(req.port),
          req.mountpoint,
          req.ntrip_version,
          req.username,
          req.password
        )
        self.run()
      except Exception as e:
        rospy.logerr("Exception while connecting: " + str(e))
        return NtripClientConnectResponse(False)
    return NtripClientConnectResponse(True)

  def handle_settings_srv(self, req):
    rospy.loginfo("New NTRIP settings:\n" + str(req))
    self._nmea_up = req.nmea_up
    self._nmea_sub = rospy.Subscriber(req.nmea_topic, Sentence, self.subscribe_nmea, queue_size=10)
    return NtripClientSettingsResponse(True)

  def handle_status_srv(self, req):
    return NtripClientStatusResponse(
      0 if rospy.Time.now() - self._last_rtcm_time < rospy.Duration(2.0) else -1
    )

  def subscribe_nmea(self, nmea):
    # Just extract the NMEA from the message, and send it right to the server
    if self._nmea_up:
      self._client.send_nmea(nmea.sentence)

  def publish_rtcm(self, event):
    # Do not try to get RTCM if not connected
    if not self._client._connected:
      raw_rtcms = []
    else:
      raw_rtcms = self._client.recv_rtcm()

    curr_time = rospy.Time.now()
    if len(raw_rtcms) > 0:
      self._last_rtcm_time = curr_time
    if curr_time - self._last_rtcm_time > rospy.Duration(kKeepAlivePeriod):
      if curr_time - self._last_reconnect_attempt_time > rospy.Duration(kAttemptReconnectTimeout):
        try:
          self._last_reconnect_attempt_time = curr_time
          # Reconnect with original client connection info
          rospy.loginfo("Attempting to reconnect to NTRIP server")
          self._client.disconnect()
          self.init_client(
            self._client._host,
            self._client._port,
            self._client._mountpoint,
            self._client._ntrip_version,
            self._client._username,
            self._client._password
          )
          if self._client.connect():
            rospy.loginfo("Reconnected to NTRIP server")
        except Exception as e:
          rospy.logerr("Exception while reconnecting: " + str(e))
          return
    for raw_rtcm in raw_rtcms:
      self._rtcm_pub.publish(RTCM(
        header=Header(
          stamp=rospy.Time.now(),
          frame_id=self._rtcm_frame_id
        ),
        data=raw_rtcm
      ))

if __name__ == '__main__':
  ntrip_ros = NTRIPRos()
  rospy.on_shutdown(ntrip_ros.stop)
  ntrip_ros.run()
  rospy.spin()
  exit(0)
