#pragma once

// JSON support used by the gateway and nodes.
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autorob {

// Minimal JSON support, included so the project can build without downloads.
class Json {
 public:
  using Array = std::vector<Json>;
  using Object = std::map<std::string, Json>;
  using Value = std::variant<std::nullptr_t, bool, double, std::string, Array, Object>;

  Json() : value_(nullptr) {}
  Json(std::nullptr_t) : value_(nullptr) {}
  Json(bool value) : value_(value) {}
  Json(int value) : value_(static_cast<double>(value)) {}
  Json(std::size_t value) : value_(static_cast<double>(value)) {}
  Json(double value) : value_(value) {}
  Json(const char* value) : value_(std::string(value)) {}
  Json(std::string value) : value_(std::move(value)) {}
  Json(Array value) : value_(std::move(value)) {}
  Json(Object value) : value_(std::move(value)) {}

  bool is_null() const { return std::holds_alternative<std::nullptr_t>(value_); }
  bool is_bool() const { return std::holds_alternative<bool>(value_); }
  bool is_number() const { return std::holds_alternative<double>(value_); }
  bool is_string() const { return std::holds_alternative<std::string>(value_); }
  bool is_array() const { return std::holds_alternative<Array>(value_); }
  bool is_object() const { return std::holds_alternative<Object>(value_); }

  bool as_bool() const { return std::get<bool>(value_); }
  double as_number() const { return std::get<double>(value_); }
  const std::string& as_string() const { return std::get<std::string>(value_); }
  const Array& as_array() const { return std::get<Array>(value_); }
  Array& as_array() { return std::get<Array>(value_); }
  const Object& as_object() const { return std::get<Object>(value_); }
  Object& as_object() { return std::get<Object>(value_); }

  const Json* get(const std::string& key) const {
    if (!is_object()) return nullptr;
    auto it = as_object().find(key);
    return it == as_object().end() ? nullptr : &it->second;
  }

  std::string dump() const {
    std::ostringstream out;
    dump_to(out);
    return out.str();
  }

  static Json parse(const std::string& text) {
    Parser parser(text);
    Json value = parser.parse_value();
    parser.skip_whitespace();
    if (!parser.at_end()) throw std::runtime_error("trailing characters after JSON value");
    return value;
  }

 private:
  class Parser {
   public:
    explicit Parser(const std::string& input) : input_(input) {}

    Json parse_value() {
      skip_whitespace();
      if (at_end()) fail("expected JSON value");
      char c = input_[position_];
      if (c == 'n') return parse_literal("null", Json(nullptr));
      if (c == 't') return parse_literal("true", Json(true));
      if (c == 'f') return parse_literal("false", Json(false));
      if (c == '"') return Json(parse_string());
      if (c == '[') return parse_array();
      if (c == '{') return parse_object();
      if (c == '-' || (c >= '0' && c <= '9')) return Json(parse_number());
      fail("invalid JSON value");
      return Json();
    }

    void skip_whitespace() {
      while (!at_end()) {
        char c = input_[position_];
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
        ++position_;
      }
    }

    bool at_end() const { return position_ >= input_.size(); }

   private:
    Json parse_literal(const char* literal, Json value) {
      std::size_t length = std::strlen(literal);
      if (input_.compare(position_, length, literal) != 0) fail("invalid literal");
      position_ += length;
      return value;
    }

    static void append_utf8(std::string& output, unsigned codepoint) {
      if (codepoint <= 0x7f) {
        output.push_back(static_cast<char>(codepoint));
      } else if (codepoint <= 0x7ff) {
        output.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
        output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
      } else if (codepoint <= 0xffff) {
        output.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
        output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
        output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
      } else {
        output.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
        output.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
        output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
        output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
      }
    }

    unsigned parse_hex4() {
      if (position_ + 4 > input_.size()) fail("short unicode escape");
      unsigned value = 0;
      for (int i = 0; i < 4; ++i) {
        char c = input_[position_++];
        value <<= 4;
        if (c >= '0' && c <= '9') value += static_cast<unsigned>(c - '0');
        else if (c >= 'a' && c <= 'f') value += static_cast<unsigned>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') value += static_cast<unsigned>(c - 'A' + 10);
        else fail("invalid unicode escape");
      }
      return value;
    }

    std::string parse_string() {
      if (input_[position_++] != '"') fail("expected string");
      std::string output;
      while (!at_end()) {
        unsigned char c = static_cast<unsigned char>(input_[position_++]);
        if (c == '"') return output;
        if (c < 0x20) fail("control character in string");
        if (c != '\\') {
          output.push_back(static_cast<char>(c));
          continue;
        }
        if (at_end()) fail("unfinished escape");
        char escaped = input_[position_++];
        switch (escaped) {
          case '"': output.push_back('"'); break;
          case '\\': output.push_back('\\'); break;
          case '/': output.push_back('/'); break;
          case 'b': output.push_back('\b'); break;
          case 'f': output.push_back('\f'); break;
          case 'n': output.push_back('\n'); break;
          case 'r': output.push_back('\r'); break;
          case 't': output.push_back('\t'); break;
          case 'u': {
            unsigned first = parse_hex4();
            unsigned codepoint = first;
            if (first >= 0xd800 && first <= 0xdbff) {
              if (position_ + 2 > input_.size() || input_[position_] != '\\' ||
                  input_[position_ + 1] != 'u') fail("missing low surrogate");
              position_ += 2;
              unsigned second = parse_hex4();
              if (second < 0xdc00 || second > 0xdfff) fail("invalid low surrogate");
              codepoint = 0x10000 + ((first - 0xd800) << 10) + (second - 0xdc00);
            } else if (first >= 0xdc00 && first <= 0xdfff) {
              fail("unexpected low surrogate");
            }
            append_utf8(output, codepoint);
            break;
          }
          default: fail("invalid string escape");
        }
      }
      fail("unterminated string");
      return {};
    }

    double parse_number() {
      std::size_t start = position_;
      if (input_[position_] == '-') ++position_;
      if (at_end()) fail("unfinished number");
      if (input_[position_] == '0') {
        ++position_;
      } else {
        if (input_[position_] < '1' || input_[position_] > '9') fail("invalid number");
        while (!at_end() && input_[position_] >= '0' && input_[position_] <= '9') ++position_;
      }
      if (!at_end() && input_[position_] == '.') {
        ++position_;
        if (at_end() || input_[position_] < '0' || input_[position_] > '9') fail("invalid fraction");
        while (!at_end() && input_[position_] >= '0' && input_[position_] <= '9') ++position_;
      }
      if (!at_end() && (input_[position_] == 'e' || input_[position_] == 'E')) {
        ++position_;
        if (!at_end() && (input_[position_] == '+' || input_[position_] == '-')) ++position_;
        if (at_end() || input_[position_] < '0' || input_[position_] > '9') fail("invalid exponent");
        while (!at_end() && input_[position_] >= '0' && input_[position_] <= '9') ++position_;
      }
      errno = 0;
      char* end = nullptr;
      std::string token = input_.substr(start, position_ - start);
      double value = std::strtod(token.c_str(), &end);
      if (errno == ERANGE || end != token.c_str() + token.size() || !std::isfinite(value)) {
        fail("number is not finite");
      }
      return value;
    }

    Json parse_array() {
      ++position_;
      Array values;
      skip_whitespace();
      if (!at_end() && input_[position_] == ']') {
        ++position_;
        return Json(std::move(values));
      }
      for (;;) {
        values.push_back(parse_value());
        skip_whitespace();
        if (at_end()) fail("unterminated array");
        char delimiter = input_[position_++];
        if (delimiter == ']') return Json(std::move(values));
        if (delimiter != ',') fail("expected comma in array");
      }
    }

    Json parse_object() {
      ++position_;
      Object values;
      skip_whitespace();
      if (!at_end() && input_[position_] == '}') {
        ++position_;
        return Json(std::move(values));
      }
      for (;;) {
        skip_whitespace();
        if (at_end() || input_[position_] != '"') fail("expected object key");
        std::string key = parse_string();
        skip_whitespace();
        if (at_end() || input_[position_++] != ':') fail("expected colon");
        Json value = parse_value();
        values[std::move(key)] = std::move(value);
        skip_whitespace();
        if (at_end()) fail("unterminated object");
        char delimiter = input_[position_++];
        if (delimiter == '}') return Json(std::move(values));
        if (delimiter != ',') fail("expected comma in object");
      }
    }

    [[noreturn]] void fail(const std::string& message) const {
      throw std::runtime_error(message + " at byte " + std::to_string(position_));
    }

    const std::string& input_;
    std::size_t position_ = 0;
  };

  static void dump_string(std::ostream& out, const std::string& value) {
    out << '"';
    static const char* hex = "0123456789abcdef";
    for (unsigned char c : value) {
      switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
          if (c < 0x20) out << "\\u00" << hex[(c >> 4) & 0xf] << hex[c & 0xf];
          else out << static_cast<char>(c);
      }
    }
    out << '"';
  }

  void dump_to(std::ostream& out) const {
    if (is_null()) out << "null";
    else if (is_bool()) out << (as_bool() ? "true" : "false");
    else if (is_number()) out << std::setprecision(17) << as_number();
    else if (is_string()) dump_string(out, as_string());
    else if (is_array()) {
      out << '[';
      bool first = true;
      for (const Json& value : as_array()) {
        if (!first) out << ',';
        first = false;
        value.dump_to(out);
      }
      out << ']';
    } else {
      out << '{';
      bool first = true;
      for (const auto& [key, value] : as_object()) {
        if (!first) out << ',';
        first = false;
        dump_string(out, key);
        out << ':';
        value.dump_to(out);
      }
      out << '}';
    }
  }

  Value value_;
};

}  // namespace autorob
