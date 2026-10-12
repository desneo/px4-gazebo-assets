#include "gimbal_controller/mini_json.hh"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace gimbal {
namespace json {

namespace {

struct Parser
{
  const std::string &s;
  size_t i{0};

  explicit Parser(const std::string &_s) : s(_s) {}

  void SkipWs()
  {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i])))
      ++i;
  }

  bool ParseValue(Value &_out)
  {
    SkipWs();
    if (i >= s.size())
      return false;
    const char c = s[i];
    if (c == '{')
      return ParseObject(_out);
    if (c == '[')
      return ParseArray(_out);
    if (c == '"')
    {
      _out.type = Value::Type::STR;
      return ParseString(_out.str);
    }
    if (c == 't' || c == 'f')
    {
      if (s.compare(i, 4, "true") == 0)
      {
        _out.type = Value::Type::BOOL;
        _out.b = true;
        i += 4;
        return true;
      }
      if (s.compare(i, 5, "false") == 0)
      {
        _out.type = Value::Type::BOOL;
        _out.b = false;
        i += 5;
        return true;
      }
      return false;
    }
    if (c == 'n')
    {
      if (s.compare(i, 4, "null") == 0)
      {
        _out.type = Value::Type::NUL;
        i += 4;
        return true;
      }
      return false;
    }
    return ParseNumber(_out);
  }

  bool ParseNumber(Value &_out)
  {
    const size_t start = i;
    auto digit = [&]() {
      return i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]));
    };
    if (i < s.size() && s[i] == '-')
      ++i;
    if (!digit())
      return false;
    if (s[i] == '0')
      ++i;
    else
      while (digit()) ++i;
    if (i < s.size() && s[i] == '.')
    {
      ++i;
      if (!digit()) return false;
      while (digit()) ++i;
    }
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E'))
    {
      ++i;
      if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
      if (!digit()) return false;
      while (digit()) ++i;
    }
    _out.type = Value::Type::NUM;
    _out.num = std::strtod(s.substr(start, i - start).c_str(), nullptr);
    return std::isfinite(_out.num);
  }

  bool ParseString(std::string &_out)
  {
    if (i >= s.size() || s[i] != '"')
      return false;
    ++i;
    _out.clear();
    while (i < s.size())
    {
      const char c = s[i++];
      if (c == '"')
        return true;
      if (c == '\\')
      {
        if (i >= s.size())
          return false;
        const char e = s[i++];
        switch (e)
        {
          case 'n': _out.push_back('\n'); break;
          case 't': _out.push_back('\t'); break;
          case 'r': _out.push_back('\r'); break;
          case 'b': _out.push_back('\b'); break;
          case 'f': _out.push_back('\f'); break;
          case '/': _out.push_back('/'); break;
          case '\\': _out.push_back('\\'); break;
          case '"': _out.push_back('"'); break;
          case 'u':
          {
            if (i + 4 <= s.size())
            {
              const unsigned code =
                  std::strtoul(s.substr(i, 4).c_str(), nullptr, 16);
              i += 4;
              if (code < 0x80)
              {
                _out.push_back(static_cast<char>(code));
              }
              else if (code < 0x800)
              {
                _out.push_back(static_cast<char>(0xC0 | (code >> 6)));
                _out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
              }
              else
              {
                _out.push_back(static_cast<char>(0xE0 | (code >> 12)));
                _out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
                _out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
              }
            }
            break;
          }
          default: _out.push_back(e); break;
        }
      }
      else
      {
        _out.push_back(c);
      }
    }
    return false;
  }

  bool ParseArray(Value &_out)
  {
    if (s[i] != '[')
      return false;
    ++i;
    _out.type = Value::Type::ARR;
    SkipWs();
    if (i < s.size() && s[i] == ']')
    {
      ++i;
      return true;
    }
    while (i < s.size())
    {
      Value v;
      if (!ParseValue(v))
        return false;
      _out.arr.push_back(std::move(v));
      SkipWs();
      if (i < s.size() && s[i] == ',')
      {
        ++i;
        continue;
      }
      if (i < s.size() && s[i] == ']')
      {
        ++i;
        return true;
      }
      return false;
    }
    return false;
  }

  bool ParseObject(Value &_out)
  {
    if (s[i] != '{')
      return false;
    ++i;
    _out.type = Value::Type::OBJ;
    SkipWs();
    if (i < s.size() && s[i] == '}')
    {
      ++i;
      return true;
    }
    while (i < s.size())
    {
      SkipWs();
      std::string key;
      if (!ParseString(key))
        return false;
      SkipWs();
      if (i >= s.size() || s[i] != ':')
        return false;
      ++i;
      Value v;
      if (!ParseValue(v))
        return false;
      _out.obj.emplace_back(std::move(key), std::move(v));
      SkipWs();
      if (i < s.size() && s[i] == ',')
      {
        ++i;
        continue;
      }
      if (i < s.size() && s[i] == '}')
      {
        ++i;
        return true;
      }
      return false;
    }
    return false;
  }
};

}  // namespace

const Value *Value::Find(const std::string &_key) const
{
  for (const auto &kv : obj)
    if (kv.first == _key)
      return &kv.second;
  return nullptr;
}

double Value::AsNum(double _def) const
{
  if (type == Type::NUM)
    return num;
  if (type == Type::BOOL)
    return b ? 1.0 : 0.0;
  return _def;
}

std::string Value::AsStr(const std::string &_def) const
{
  if (type == Type::STR)
    return str;
  return _def;
}

bool Parse(const std::string &_text, Value &_out)
{
  Parser p(_text);
  if (!p.ParseValue(_out))
    return false;
  p.SkipWs();
  return p.i >= _text.size();
}

std::string Escape(const std::string &_s)
{
  std::string out;
  out.reserve(_s.size() + 2);
  for (char c : _s)
  {
    switch (c)
    {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20)
        {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        }
        else
        {
          out.push_back(c);
        }
    }
  }
  return out;
}

std::string Num(double _v, int _digits)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", _digits, _v);
  std::string s(buf);
  if (s.find('.') != std::string::npos)
  {
    while (!s.empty() && s.back() == '0')
      s.pop_back();
    if (!s.empty() && s.back() == '.')
      s.pop_back();
  }
  if (s == "-0")
    s = "0";
  return s;
}

}  // namespace json
}  // namespace gimbal
