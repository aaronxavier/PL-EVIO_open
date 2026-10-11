#pragma once

#include <dvs_msgs/msg/event_array.hpp>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>

namespace event_packet_decoder
{

// Subscribes to a libcaer_driver-style event_camera_msgs/EventPacket topic and
// republishes the decoded events as dvs_msgs/EventArray, so unmodified PL-EVIO
// feature-tracking code (originally written against ROS1 dvs_msgs) can consume
// the Jetson's DAVIS346 driver stack without caring about the packet encoding.
class EventPacketDecoderNode : public rclcpp::Node, public event_camera_codecs::EventProcessor
{
public:
  explicit EventPacketDecoderNode(const rclcpp::NodeOptions & options);

  // event_camera_codecs::EventProcessor interface, invoked synchronously from
  // within decoder_->decode() while handling one EventPacket.
  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override;
  bool eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) override;
  void finished() override;
  void rawData(const char * data, size_t len) override;

private:
  void packetCallback(const event_camera_msgs::msg::EventPacket::ConstSharedPtr msg);

  rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr sub_;
  rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr pub_;
  event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, EventPacketDecoderNode>
    decoder_factory_;

  // Scratch state, valid only while decoding the packet currently being processed
  // inside packetCallback() -- eventCD() appends into out_.events.
  dvs_msgs::msg::EventArray out_;
};

}  // namespace event_packet_decoder
