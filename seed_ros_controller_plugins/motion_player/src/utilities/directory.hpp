#pragma once

#include "common.hpp"

std::vector<std::string> getFileNames(std::string folderPath, std::string extension, std::string prefix = "") ;
std::vector<std::string> searchDirectoryPath(std::string tgtPath, std::string name);
std::vector < std::string > getFileIncludeDirectoryList(std::string tgtPath,std::string fname);
bool makeDirectory(std::string dir);
bool removeDirectory(std::string dir);
bool moveDirectory(std::string dir_old,std::string dir_new);
void removeFile(std::string fpath);
void copyFile(std::string from_path,std::string to_path,bool recursive = false);
void renameFile(std::string from_path,std::string to_path);
bool fileExist(std::string fpath);
std::string getDataDirectory();
