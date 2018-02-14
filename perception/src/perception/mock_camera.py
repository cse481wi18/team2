import rosbag

class MockCamera(object): 
    """A MockCamera reads saved point clouds.
    """
    def __init__(self):
      pass

    def read_cloud(self, path):
      """Returns the sensor_msgs/PointCloud2 in the given bag file.

      Args:
          path: string, the path to a bag file with a single
          sensor_msgs/PointCloud2 in it.

      Returns: A sensor_msgs/PointCloud2 message, or None if there were no
          PointCloud2 messages in the bag file.
      """
      bag = rosbag.Bag(path)
      msg_list = []
      # topic, msg, t
      for topic, msg, t in bag.read_messages(topics=['head_camera/depth_registered/points']):
        msg_list.append(msg)
      bag.close()
      return msg_list[0]
