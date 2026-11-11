#pragma once
// ROS-free replacement for ros::NodeHandle parameters + vk::param<T>().
// Parameters are read from one or more flat YAML files (same files/keys as the
// original svo_ros/param/*.yaml) and may be overridden from the command line
// with "key=value" pairs.

#include <map>
#include <set>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace svo {

class ParamServer
{
public:
  ParamServer() = default;

  /// Load a flat YAML file (scalars only, nested maps are flattened with '/').
  /// Later files override earlier ones.
  bool loadYaml(const std::string& filename);

  /// Set / override a single value (string representation).
  void set(const std::string& key, const std::string& value);

  /// Parse "key=value" override.
  bool setFromAssignment(const std::string& assignment);

  bool has(const std::string& key) const;

  template <typename T>
  T get(const std::string& key, const T& default_value) const;

  /// Print all parameters that were requested so far (with value and source).
  void printUsed(std::ostream& out) const;

private:
  std::map<std::string, std::string> values_;
  mutable std::map<std::string, std::string> used_;
  mutable std::mutex mutex_;
};

// ----------------------------------------------------------------------------
namespace param_detail {
bool toBool(const std::string& s, bool def);
}

template <typename T>
T ParamServer::get(const std::string& key, const T& default_value) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = values_.find(key);
  if (it == values_.end())
  {
    std::ostringstream ss;
    ss << default_value;
    used_[key] = ss.str() + "  (default)";
    return default_value;
  }
  std::istringstream ss(it->second);
  T v = default_value;
  ss >> v;
  if (ss.fail())
  {
    used_[key] = it->second + "  (PARSE ERROR -> default)";
    return default_value;
  }
  used_[key] = it->second;
  return v;
}

template <>
inline bool ParamServer::get<bool>(const std::string& key,
                                   const bool& default_value) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = values_.find(key);
  if (it == values_.end())
  {
    used_[key] = std::string(default_value ? "true" : "false") + "  (default)";
    return default_value;
  }
  used_[key] = it->second;
  return param_detail::toBool(it->second, default_value);
}

template <>
inline std::string ParamServer::get<std::string>(
    const std::string& key, const std::string& default_value) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = values_.find(key);
  if (it == values_.end())
  {
    used_[key] = default_value + "  (default)";
    return default_value;
  }
  used_[key] = it->second;
  return it->second;
}

} // namespace svo

// Drop-in replacement of vk::param(nh, name, default) used by the factories.
namespace vk {
template <typename T>
inline T param(const svo::ParamServer& p, const std::string& name,
               const T& defaultValue)
{
  return p.get<T>(name, defaultValue);
}
inline std::string param(const svo::ParamServer& p, const std::string& name,
                         const char* defaultValue)
{
  return p.get<std::string>(name, std::string(defaultValue));
}
} // namespace vk
