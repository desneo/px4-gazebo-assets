// 极简 JSON 解析/序列化工具（控制指令解析与遥测序列化用）。
#ifndef GIMBAL_CONTROLLER_MINI_JSON_HH_
#define GIMBAL_CONTROLLER_MINI_JSON_HH_

#include <string>
#include <utility>
#include <vector>

namespace gimbal {
namespace json {

struct Value
{
  enum class Type
  {
    NUL,
    BOOL,
    NUM,
    STR,
    ARR,
    OBJ
  };

  Type type{Type::NUL};
  bool b{false};
  double num{0.0};
  std::string str;
  std::vector<Value> arr;
  std::vector<std::pair<std::string, Value>> obj;

  const Value *Find(const std::string &_key) const;
  bool IsNum() const { return type == Type::NUM || type == Type::BOOL; }
  bool IsStr() const { return type == Type::STR; }
  double AsNum(double _def = 0.0) const;
  std::string AsStr(const std::string &_def = "") const;
};

/// 解析 JSON 文本；成功返回 true。
bool Parse(const std::string &_text, Value &_out);

/// JSON 字符串转义（含引号）。
std::string Escape(const std::string &_s);

/// 数值格式化（固定小数位，去除多余 0）。
std::string Num(double _v, int _digits = 3);

}  // namespace json
}  // namespace gimbal

#endif  // GIMBAL_CONTROLLER_MINI_JSON_HH_
