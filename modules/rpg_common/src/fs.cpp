#include "rpg_common/fs.h"

// Standalone port: POSIX dirent/stat replaced by std::filesystem (portable).
#include <filesystem>

#include <glog/logging.h>

namespace rpg_common
{
namespace fs
{

namespace stdfs = std::filesystem;

void concatenateFolderAndFileName(
        const std::string& folder, const std::string& file_name,
        std::string* path) {
    CHECK_NOTNULL(path);
    CHECK(!file_name.empty());
    *path = folder;
    if (!path->empty() && path->back() != '/' && path->back() != '\\') {
        *path += '/';
    }
    *path = *path + file_name;
}

std::string concatenateFolderAndFileName(
        const std::string& folder, const std::string& file_name) {
    std::string path;
    concatenateFolderAndFileName(folder, file_name, &path);
    return path;
}

bool fileExists(const std::string& path)
{
  std::error_code ec;
  return stdfs::is_regular_file(stdfs::path(path), ec);
}

bool pathExists(const std::string& path)
{
  std::error_code ec;
  return stdfs::is_directory(stdfs::path(path), ec);
}

void splitPathAndFilename(
    const std::string& str, std::string* path, std::string* filename)
{
  CHECK_NOTNULL(path)->clear();
  CHECK_NOTNULL(filename)->clear();
  const size_t right_delim = str.find_last_of("/\\");
  if (right_delim != std::string::npos)
  {
    *path = str.substr(0, right_delim);
  }
  *filename = str.substr(right_delim + 1);
}

// Returns full paths. No recursion.
void getFilesAndSubfolders(const std::string& path,
                           std::vector<std::string>* files,
                           std::vector<std::string>* folders)
{
  CHECK_NOTNULL(files)->clear();
  CHECK_NOTNULL(folders)->clear();
  CHECK(pathExists(path));

  for (const auto& entry : stdfs::directory_iterator(stdfs::path(path)))
  {
    const std::string abs_path = path + "/" + entry.path().filename().string();
    if (entry.is_regular_file())
    {
      files->emplace_back(abs_path);
    }
    else if (entry.is_directory())
    {
      folders->emplace_back(abs_path);
    }
  }
}

}  // namespace fs
}  // namespace rpg_common
