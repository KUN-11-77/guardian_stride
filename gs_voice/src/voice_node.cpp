// gs_voice/src/voice_node.cpp
// 唤醒词 + ASR 主节点：whisper.cpp NPU 推理
// 完全离线，首字延迟 ≤ 800ms

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gs_msgs/msg/intent.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "intent_parser.hpp"
#include "tts_manager.hpp"

class VoiceNode : public rclcpp::Node {
public:
  VoiceNode()
    : Node("voice_node"),
      wake_word_detected_(false),
      is_listening_(false) {
    declare_parameter("wake_word", "小助手");
    declare_parameter("asr_model_path", "/models/whisper-tiny-openvino");
    declare_parameter("enable_tts", true);

    wake_word_ = get_parameter("wake_word").as_string();

    intent_pub_ = create_publisher<gs_msgs::msg::Intent>("/intent", 10);
    tts_sub_ = create_subscription<std_msgs::msg::String>(
        "/tts_priority", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          tts_manager_.speak(msg->data, 0);
        });

    tts_manager_.set_node(shared_from_this());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { checkWakeWord(); });

    RCLCPP_INFO(get_logger(), "VoiceNode 已启动，唤醒词：%s", wake_word_.c_str());
  }

private:
  void checkWakeWord() {
    // 简化的唤醒词检测（实际应用中需要独立的唤醒词检测模型）
    // 这里仅作占位符
  }

  void processAudioChunk(const std::vector<float>& audio_data) {
    if (!wake_word_detected_) return;

    // Whisper ASR 推理（NPU）
    std::string text = runWhisperInference(audio_data);

    if (!text.empty()) {
      RCLCPP_INFO(get_logger(), "识别到语音：%s", text.c_str());

      // 解析意图
      auto intent = intent_parser_.parse(text);

      if (intent) {
        intent_pub_->publish(*intent);
      }
    }
  }

  std::string runWhisperInference(const std::vector<float>& audio_data) {
    // 占位符：实际使用 whisper.cpp + OpenVINO NPU 推理
    // 返回空字符串表示未识别到语音
    return "";
  }

  std::string wake_word_;
  bool wake_word_detected_;
  bool is_listening_;

  rclcpp::Publisher<gs_msgs::msg::Intent>::SharedPtr intent_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  IntentParser intent_parser_;
  TTSManager tts_manager_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoiceNode>());
  rclcpp::shutdown();
  return 0;
}
