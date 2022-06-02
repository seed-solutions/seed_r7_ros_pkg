#pragma once

#include "common.hpp"
#include "archive.hpp"
#include "utilities.hpp"

constexpr double JOINT_VALUE_INVALID =  std::numeric_limits<double>::max(); // 値を利用しない場合

class RobotState {
    public:
        typedef std::map<std::string, std::pair<double, double>> type_jlimits;
        typedef std::vector<std::string>  type_jnames;
        typedef std::map<std::string, double> type_jpos;

        RobotState(){}

        RobotState(const std::vector<std::string> &jnames){
            set_jnames(jnames);
        }

        /** \brief 関節名も含めてデータをクリアする
         *
         */
        void clear(){
            jpos.clear();
        }

        /** \brief ロボットの状態をリセットする
         *
         * \関節名はリセットしない
         */
        void reset(){
            for (auto itr = jpos.begin(); itr != jpos.end(); ++itr) {
                itr->second = JOINT_VALUE_INVALID;
            }
        }

        void set_jnames(const std::vector<std::string> &jnames){
            jpos.clear();
            for (auto jname : jnames) {
                jpos[jname] = JOINT_VALUE_INVALID;
            }
        }

        unsigned int get_size() const{
            return jpos.size();
        }

        bool set_position(std::string name, double value) {

            auto itr = jpos.find(name);
            if(itr != jpos.end()){
                itr->second = value;
                return true;
            }else{
                LOG_ERROR_STREAM("no such joint:"<<name<<LOG_END);
                return false;
            }
        }

        bool add_joint(std::string name, double value = JOINT_VALUE_INVALID) {
            if(jpos.count(name) == 0){
                jpos[name] = value;
                return true;
            }else{
                LOG_ERROR_STREAM("joint already exist :"<<name<<LOG_END);
                return false;
            }
        }

        void set_position(const std::map<std::string, double> &joint_values){
            for(auto jvalue:joint_values){
                set_position(jvalue.first,jvalue.second);
            }
        }

        std::map<std::string, double> get_position() const{
            return jpos;
        }

        double get_position(std::string jname) const{
            if(jpos.count(jname) != 0){
                return jpos.at(jname);
            }
            return JOINT_VALUE_INVALID;
        }

        std::vector<std::string> get_jnames() const{
            std::vector<std::string> jnames;
            for (auto itr = jpos.begin(); itr != jpos.end(); ++itr) {
                jnames.push_back(itr->first);
            }
            return jnames;
        }

        std::map<std::string, double>::iterator begin(){
            return jpos.begin();
        }

        std::map<std::string, double>::iterator end(){
            return jpos.end();
        }


protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(jpos);
    }

private:
    std::map<std::string, double> jpos; //!< 関節座標
};
