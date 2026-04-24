// gs_voice/src/tts_manager.cpp
// TTS 管理器：优先级队列，离线 TTS（espeak-ng 或 piper）

#include "tts_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <cstring>

TTSManager::TTSManager()
  : node_(nullptr)
  , is_speaking_(false)
  , current_priority_(0) {}

void TTSManager::set_node(rclcpp::Node::SharedPtr node) {
  node_ = node;
}

void TTSManager::speak(const std::string& text, int priority) {
  if (!node_) return;

  if (priority > current_priority_ && is_speaking_) {
    tts_queue_.clear();
    tts_queue_.push_back({text, priority});
    std::reverse(tts_queue_.begin(), tts_queue_.end());
    stopCurrent();
  } else {
    tts_queue_.push_back({text, priority});
    std::sort(tts_queue_.begin(), tts_queue_.end(),
              [](const TTSItem& a, const TTSItem& b) {
                return a.priority > b.priority;
              });
  }

  processQueue();
}

void TTSManager::stop() {
  stopCurrent();
  tts_queue_.clear();
  is_speaking_ = false;
}

void TTSManager::processQueue() {
  if (is_speaking_ || tts_queue_.empty() || !node_) return;

  auto item = tts_queue_.front();
  tts_queue_.erase(tts_queue_.begin());
  current_priority_ = item.priority;

  RCLCPP_INFO(node_->get_logger(), "TTS: %s", item.text.c_str());

  std::string cmd = "espeak-ng \"" + item.text + "\" 2>/dev/null &";
  (void)system(cmd.c_str());

  is_speaking_ = true;
  current_priority_ = item.priority;
}

void TTSManager::stopCurrent() {
  (void)system("pkill -9 espeak-ng 2>/dev/null");
  is_speaking_ = false;
  current_priority_ = 0;
}
