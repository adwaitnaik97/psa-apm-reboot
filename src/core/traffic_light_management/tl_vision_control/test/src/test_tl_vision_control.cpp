#include <tl_vision_control/tl_vision_control.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace asf = aipe::tl_vision_control;

class TrafficLightControlTestSuite : public ::testing::Test {
public:
  TrafficLightControlTestSuite() {}

  ~TrafficLightControlTestSuite() {}
};

class TrafficLightControlTestClass {
public:
  TrafficLightControlTestClass() {
    //node_tl_vision_control_ptr = std::make_shared<aipe::SensorFusionApp>(new aipe::SensorFusionApp());

  };
  aipe::SensorFusionApp node_tl_vision_control;
  //std::shared_ptr<aipe::SensorFusionApp> node_tl_vision_control_ptr;
};

std::shared_ptr<TrafficLightControlTestClass> ssf_test;

TEST(TestSuite, LOAD_DATA) {

  /*asf::SensorData sensor_data;
  sensor_data[aipe::tl_vision_control::CAMERA] = boost::shared_ptr<asf::InputTracks>(new asf::InputTracks(asf::CAMERA));

  aipe_msgs::DetectedObjectArray vis_array;
  for(int i = 0; i < 6; ++i)
  {
    aipe_msgs::DetectedObject obj;
    obj.id = i;
    obj.label = "Traffic_light";
    obj.x = i*100;
    obj.y = i*100;
    obj.width = 20;
    obj.height = 50;
    vis_array.objects.push_back(obj);
  }
  asf::InputTracks cam_tracks(asf::CAMERA,vis_array);

  aipe_msgs::DetectedObjectArray class_array;
  for(int i = 0; i < 3; ++i)
  {
    aipe_msgs::DetectedObject obj;
    obj.id = i;
    obj.label = "GREEN";
    obj.x = i*200;
    obj.y = i*200;
    obj.width = 20;
    obj.height = 50;
    class_array.objects.push_back(obj);
  }
  asf::InputTracks class_tracks(asf::CLASSIFICATION, class_array);

  sensor_data[asf::CAMERA] = boost::make_shared<asf::InputTracks>(cam_tracks);
  sensor_data[asf::CLASSIFICATION] = boost::make_shared<asf::InputTracks>(class_tracks);

  ASSERT_EQ(ssf_test->node_tl_vision_control.LoadBuffer(sensor_data), true) << "Could not load the sensor data";
  */
}

TEST(TestSuite, REPLACE_TRAFFIC_LIGHTS) {
  /*
  int num_TLs = ssf_test->node_tl_vision_control.NumberOfTLs();
  ASSERT_EQ(ssf_test->node_tl_vision_control.ReplaceTrafficLights(), true) << "Could not replace traffic lights";
  ASSERT_NE(num_TLs, ssf_test->node_tl_vision_control.NumberOfTLs()) << "Traffic light number should not be same";
  */
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TrafficLightControlTestNode");
  ssf_test = std::make_shared<TrafficLightControlTestClass>();
  return RUN_ALL_TESTS();
}