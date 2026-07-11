// gs_voice/src/tts_manager.hpp
// TTS 管理器头文件

#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

struct TTSItem {
  std::string text;
  int priority;  // 0=最低，数字越大优先级越高
};

class TTSManager {
public:
  TTSManager();

  void set_node(rclcpp::Node::SharedPtr node);

  // 添加 TTS 任务到队列
  // priority: 优先级，0=最低，数字越大优先级越高
  void speak(const std::string& text, int priority);

  // 停止当前播放并清空队列
  void stop();

private:
  void processQueue();
  void stopCurrent();

  rclcpp::Node::SharedPtr node_;
  std::vector<TTSItem> tts_queue_;
  bool is_speaking_;
  int current_priority_;
};
