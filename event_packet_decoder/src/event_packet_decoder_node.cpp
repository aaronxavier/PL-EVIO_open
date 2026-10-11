#include "event_packet_decoder/event_packet_decoder_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace event_packet_decoder
{

EventPacketDecoderNode::EventPacketDecoderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("event_packet_decoder", options)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
  pub_ = create_publisher<dvs_msgs::msg::EventArray>("events_decoded", qos);
  sub_ = create_subscription<event_camera_msgs::msg::EventPacket>(
    "events", qos,
    std::bind(&EventPacketDecoderNode::packetCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "subscribed to '%s', publishing decoded events on '%s'",
    sub_->get_topic_name(), pub_->get_topic_name());
}

void EventPacketDecoderNode::packetCallback(
  const event_camera_msgs::msg::EventPacket::ConstSharedPtr msg)
{
  auto * decoder = decoder_factory_.getInstance(*msg);
  if (!decoder) {
    RCLCPP_ERROR_ONCE(
      get_logger(), "no decoder available for encoding '%s'", msg->encoding.c_str());
    return;
  }
  out_.header = msg->header;
  out_.width = msg->width;
  out_.height = msg->height;
  out_.events.clear();
  out_.events.reserve(msg->events.size() / 2);  // rough upper-bound guess, avoids reallocs

  while (decoder->decode(*msg, this)) {
    // decode() must be called until it returns false (see event_camera_codecs::Decoder docs);
    // in practice this loop body never actually runs since eventExtTrigger() never aborts.
  }

  if (!out_.events.empty()) {
    pub_->publish(out_);
  }
}

void EventPacketDecoderNode::eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity)
{
  dvs_msgs::msg::Event e;
  e.x = ex;
  e.y = ey;
  e.polarity = (polarity != 0);
  e.ts = static_cast<double>(sensor_time) * 1e-9;  // seconds since epoch
  out_.events.push_back(e);
}

bool EventPacketDecoderNode::eventExtTrigger(uint64_t /*sensor_time*/, uint8_t /*edge*/, uint8_t /*id*/)
{
  return true;  // continue decoding; PL-EVIO has no use for external trigger pulses
}

void EventPacketDecoderNode::finished()
{
}

void EventPacketDecoderNode::rawData(const char * /*data*/, size_t /*len*/)
{
}

}  // namespace event_packet_decoder

RCLCPP_COMPONENTS_REGISTER_NODE(event_packet_decoder::EventPacketDecoderNode)
