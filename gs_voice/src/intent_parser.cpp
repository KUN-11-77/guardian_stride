// gs_voice/src/intent_parser.cpp
// 槽位填充 → Intent JSON

#include "intent_parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cstring>

IntentParser::IntentParser() {
  // 初始化意图关键词映射
  intent_keywords_["navigate"] = {"去", "到", "导航", "走", "前进", "goto", "navigate"};
  intent_keywords_["query_status"] = {"状态", "情况", "怎么样", "查询", "status", "query"};
  intent_keywords_["mode_switch"] = {"切换", "模式", "mode", "switch"};
  intent_keywords_["emergency"] = {"停止", "停下", "紧急", "estop", "emergency", "stop"};
}

std::optional<gs_msgs::msg::Intent> IntentParser::parse(const std::string& text) {
  gs_msgs::msg::Intent intent;
  intent.header.stamp = rclcpp::Node::make_shared("intent_parser")->now();
  intent.confidence = 0.0f;

  std::string lower_text = text;
  std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), ::tolower);

  // 识别意图类型
  for (const auto& [intent_type, keywords] : intent_keywords_) {
    for (const auto& keyword : keywords) {
      std::string lower_kw = keyword;
      std::transform(lower_kw.begin(), lower_kw.end(), lower_kw.begin(), ::tolower);

      if (lower_text.find(lower_kw) != std::string::npos) {
        intent.type = intent_type;
        intent.confidence = 0.8f;
        break;
      }
    }
    if (intent.confidence > 0.5f) break;
  }

  if (intent.confidence < 0.5f) {
    // 默认查询状态
    intent.type = "query_status";
    intent.confidence = 0.3f;
  }

  // 提取目标位置
  intent.target = extractTarget(text);

  return intent;
}

std::string IntentParser::extractTarget(const std::string& text) {
  std::vector<std::string> locations = {
    "入口", "电梯", "楼梯", "门口", "前台", "会议室",
    "entrance", "elevator", "stairs", "door", "lobby"
  };

  std::string lower_text = text;
  std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), ::tolower);

  for (const auto& loc : locations) {
    std::string lower_loc = loc;
    std::transform(lower_loc.begin(), lower_loc.end(), lower_loc.begin(), ::tolower);

    if (lower_text.find(lower_loc) != std::string::npos) {
      return loc;
    }
  }

  return "";
}
