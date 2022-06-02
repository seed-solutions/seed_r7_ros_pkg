#pragma once

#include "common.hpp"

#include "structure_robotstate.hpp"
#include "sound_extension.hpp"
#include "audio_extension.hpp"



class ExtensionList{
public:
    ExtensionList(QObject* parent = Q_NULLPTR){
        ext_list.push_back(Extension::Ptr(new SoundExtension(parent)));//
        ext_list.push_back(Extension::Ptr(new AudioExtension));
    }

    ~ExtensionList(){
        LOG_INFO_STREAM("closed."<<LOG_END);
        for(auto itr = ext_list.begin();itr != ext_list.end();){
            auto extension = *itr;
            itr = ext_list.erase(itr);
            delete extension;
        }
    }

    void entry(){
        for(auto &ext:ext_list){
            ext->entry();
        }
    }

    void exit(){
        for(auto &ext:ext_list){
            ext->exit();
        }
    }

    void execute(int step, std::vector<ExtensionParam> extension) const {

        for(auto &param:extension){
            auto name = param.name;
            auto args = param.args;
            for(auto &ext:ext_list){
                if(ext->get_name() == name){
                    ext->execute(args);
                }
            }
        }
    }

private:
    std::vector<Extension::Ptr> ext_list;
};
