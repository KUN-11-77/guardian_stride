// gs_voice/src/intent_parser.hpp
// 意图解析器头文件

#pragma once

#include <string>
#include <optional>
#include <vector>
#include <map>
#include <gs_msgs/msg/intent.hpp>

class IntentParser {
public:
  IntentParser();

  // 解析文本为 Intent 消息
  // 返回 std::nullopt 如果解析失败
  std::optional<gs_msgs::msg::Intent> parse(const std::string& text);

private:
  // 从文本中提取目标位置
  std::string extractTarget(const std::string& text);

  // 意图类型 → 关键词列表
  std::map<std::string, std::vector<std::string>> intent_keywords_;
};
