#include "directory.hpp"

bool fileExist(std::string fpath){
    return std::filesystem::exists(fpath);
}

std::vector<std::string> getFileNames(std::string folderPath, std::string extension, std::string prefix) {

    std::vector < std::string > file_names;
    std::filesystem::path path(folderPath);
    if (std::filesystem::is_directory(path)) {
        auto dir_it = std::filesystem::directory_iterator(path);
        for (auto &file : dir_it) {

            if (file.path().extension() == extension) {
                auto fname = file.path().filename().string();
                if (prefix.empty() || std::equal(std::begin(prefix), std::end(prefix), std::begin(fname)))
                    file_names.push_back(file.path().string());
            }
        }
    }

    return file_names;
}

void getFileIncludeDirectoryListImpl(const std::filesystem::path& tgtPath,std::string fname,std::vector < std::string >& ret){
    auto dir_it = std::filesystem::directory_iterator(tgtPath);
    for (auto &child : dir_it) {
        if(std::filesystem::is_directory(child)){
            //小ディレクトリをたどっていく
            getFileIncludeDirectoryListImpl(child.path(),fname,ret);
        }else{
            //子ファイルが、対象のファイル名の場合は、結果に格納する
            if(child.path().filename().string() == fname){
                ret.push_back(tgtPath.filename().string());
            }
        }
    }
}

std::vector < std::string > getFileIncludeDirectoryList(std::string tgtPath,std::string fname){
    std::vector < std::string > ret;
    std::filesystem::path path(tgtPath);
    if(!std::filesystem::is_directory(path)){
        return ret;
    }

    getFileIncludeDirectoryListImpl(path,fname,ret);
    return ret;
}

std::vector<std::string> searchDirectoryPath(std::string tgtPath, std::string name) {

    std::vector < std::string > dir_paths;
    std::filesystem::path path(tgtPath);
    auto dir_it = std::filesystem::recursive_directory_iterator(path);
    for (auto &dir : dir_it) {
        if(std::filesystem::is_directory(dir)){
            auto dirname = dir.path().filename().string();
            if (dirname == name){
                dir_paths.push_back(dir.path().string());
            }
        }
    }
    return dir_paths;
}


bool makeDirectory(std::string dir) {
#ifdef WIN32
    system(("mkdir "+dir).c_str());
#else
    std::filesystem::create_directories(dir);
#endif
    return true;

}

bool removeDirectory(std::string dir) {
#ifdef WIN32
    system(("rmdir "+dir).c_str());
#else
    std::filesystem::remove_all(dir);
#endif
    return true;
}

bool moveDirectory(std::string dir_old,std::string dir_new){
    std::filesystem::rename(dir_old,dir_new);
    return true;
}

void removeFile(std::string fpath){
    if (std::filesystem::is_regular_file(fpath)) {
        std::filesystem::remove_all(fpath);
    }else{
        LOG_ERROR_STREAM("file is not a regular file : "<<fpath<<LOG_END);
    }
}

void copyFile(std::string from_path, std::string to_path,bool recursive) {
    try {
        auto options = std::filesystem::copy_options::overwrite_existing;
        if(recursive){
            options = options|std::filesystem::copy_options::recursive;
        }
        std::filesystem::copy(from_path, to_path, std::filesystem::copy_options::overwrite_existing);
    } catch (std::filesystem::filesystem_error &err) {
        LOG_ERROR_STREAM("cannot copy file : ["<<from_path<<"] -> ["<<to_path<<"] cause:"<<err.what()<<LOG_END);
    }
}

void renameFile(std::string from_path,std::string to_path){
    std::filesystem::rename(from_path,to_path);
}

std::string getDataDirectory(){
    auto curpath = std::filesystem::absolute(std::filesystem::current_path()).string()+"/data";
//    getEnvStr("SGS_DATA_DIR","./plugins/task_editor_plugins");
    return curpath;
}
