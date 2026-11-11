#include "param_server.h"

#include <algorithm>
#include <cctype>
#include <iostream>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace svo {

namespace param_detail {
bool toBool(const std::string& s_in, bool def)
{
  std::string s = s_in;
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (s == "true" || s == "1" || s == "yes" || s == "on")
    return true;
  if (s == "false" || s == "0" || s == "no" || s == "off")
    return false;
  return def;
}
} // namespace param_detail

namespace {
void flatten(const YAML::Node& node, const std::string& prefix,
             std::map<std::string, std::string>& out)
{
  if (node.IsMap())
  {
    for (auto it = node.begin(); it != node.end(); ++it)
    {
      const std::string key = it->first.as<std::string>();
      flatten(it->second, prefix.empty() ? key : prefix + "/" + key, out);
    }
  }
  else if (node.IsScalar())
  {
    out[prefix] = node.as<std::string>();
  }
  else if (node.IsSequence())
  {
    // store sequences as space separated string (not used by SVO params)
    std::string s;
    for (const auto& n : node)
      if (n.IsScalar())
        s += (s.empty() ? "" : " ") + n.as<std::string>();
    out[prefix] = s;
  }
}
} // namespace

bool ParamServer::loadYaml(const std::string& filename)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(filename);
  }
  catch (const std::exception& e)
  {
    LOG(ERROR) << "Could not load parameter file '" << filename
               << "': " << e.what();
    return false;
  }
  std::map<std::string, std::string> flat;
  flatten(root, "", flat);
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& kv : flat)
    values_[kv.first] = kv.second;
  return true;
}

void ParamServer::set(const std::string& key, const std::string& value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  values_[key] = value;
}

bool ParamServer::setFromAssignment(const std::string& assignment)
{
  const size_t pos = assignment.find('=');
  if (pos == std::string::npos || pos == 0)
    return false;
  set(assignment.substr(0, pos), assignment.substr(pos + 1));
  return true;
}

bool ParamServer::has(const std::string& key) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return values_.count(key) > 0;
}

void ParamServer::printUsed(std::ostream& out) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  out << "---------------- SVO parameters ----------------\n";
  for (const auto& kv : used_)
    out << "  " << kv.first << ": " << kv.second << "\n";
  out << "------------------------------------------------" << std::endl;
}

} // namespace svo
