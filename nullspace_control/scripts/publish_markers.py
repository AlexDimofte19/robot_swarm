import rospy
from visualization_msgs.msg import Marker, MarkerArray

class PublishMarkers(object):

  def __init__(self, frame_id, topic):
    self.marker_publisher = rospy.Publisher(topic, MarkerArray, queue_size = 1)
    self.frame_id = frame_id

  def publish_markers(self, data, r = 0, g = 0, b = 0):
    marker_array = MarkerArray()
    id_count = 0

    for coordinate in data:
      marker = Marker()
      marker.id = id_count
      marker.header.frame_id = self.frame_id
      marker.header.stamp = rospy.Time.now()
      marker.lifetime = rospy.Duration(0.5)
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      marker.scale.x = 0.05
      marker.scale.y = 0.05
      marker.scale.z = 0.05
      marker.color.r = r
      marker.color.g = g
      marker.color.b = b
      marker.color.a = 1.0
      marker.pose.orientation.w = 1.0
      marker.pose.position.x = coordinate[0]
      marker.pose.position.y = coordinate[1]
      # we made the pointers appear at the level of the lidar (approx same z-plane)
      marker.pose.position.z = 0.1

      marker_array.markers.append(marker)
      id_count += 1

    self.marker_publisher.publish(marker_array)


