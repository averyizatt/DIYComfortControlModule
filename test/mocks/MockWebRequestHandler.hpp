#pragma once

#include <string>

struct MockWebRequestHandler {
  int status = 200;
  std::string body = "{}";

  void rejectMalformedJson(const std::string& payload) {
    if (payload.size() < 2 || payload.front() != '{' || payload.back() != '}') {
      status = 400;
      body = "{\"error\":\"invalid_json\"}";
    }
  }

  void respondOk(const std::string& responseBody) {
    status = 200;
    body = responseBody;
  }
};
