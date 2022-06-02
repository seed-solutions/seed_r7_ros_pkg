#include "common.hpp"
#include "utilities.hpp"
#include "joint_state_list.hpp"

JointStateListLoader::JointStateListLoader(){

}

JointStateListLoader::~JointStateListLoader(){
    if(th.joinable()){
        th.join();
    }
}

void JointStateListLoader::run(std::ifstream &ifs){
    if(!done.load()){
        LOG_INFO_STREAM("still doing"<<LOG_END);
    }
    if(th.joinable()){
        th.join();
    }

    this->ifs = std::move(ifs);
    done.store(false);
    th = std::thread(&JointStateListLoader::loadStates,this);
}

void JointStateListLoader::wait(){
    while(!done.load()){
        millisleep(1);
    }

    if(th.joinable()){
        th.join();
    }
}


void JointStateListLoader::registerCallbacks(std::function<void(JointStateListLoader*)> available_callback,std::function<void(JointStateListLoader*)> finish_callback){
    available_cb = available_callback;
    finish_cb = finish_callback;
}


std::vector<RobotState> JointStateListLoader::getData(){
    std::lock_guard<std::mutex> lock(mtx);
    auto retData = states_readed;
    states_readed.clear();
    return retData;
}


void JointStateListLoader::loadStates(){
    if(joint_names.size () == 0 || !ifs){
        LOG_ERROR_STREAM("error");
        done.store(true);
        return;
    }

    RobotState state;
    state.set_jnames(joint_names);

    std::string line;
    while(getline(ifs, line)){
        auto elems = split(line,",");
        if(elems.size() != joint_names.size()){
            LOG_ERROR_STREAM("joint size error");
            continue;
        }

        for(size_t jidx = 0;jidx < elems.size();++jidx){
            double pos = std::stod(elems[jidx]);
            state.set_position(joint_names[jidx], pos);
        }
        bool data_available = false;
        mtx.lock();
        states_readed.push_back(state);
        if (states_readed.size() > batch_size) {
            data_available = true;
        }
        mtx.unlock();
        state.reset();

        if(data_available && available_cb){
            available_cb(this);
        }
    }

    if(finish_cb){
        finish_cb(this);
    }
    done.store(true);
}


JointStateList::JointStateList(){
    jlist_loader = new JointStateListLoader;
}

JointStateList::~JointStateList(){
    delete jlist_loader;
}

int JointStateList::size() const{
    return state_size;
}

void JointStateList::clear(){
    std::lock_guard<std::mutex> lk(mtx);
    states.clear();
    state_size = 0;
}


RobotState& JointStateList::operator[](int idx){
    std::lock_guard<std::mutex> lk(mtx);
    return states[idx];
}

const RobotState& JointStateList::operator[](int idx)const{
    std::lock_guard<std::mutex> lk(mtx);
    return states[idx];
}


std::vector<RobotState>::iterator JointStateList::begin(){
    return states.begin();
}

std::vector<RobotState>::iterator JointStateList::end(){
    return states.end();
}

void JointStateList::saveStates(std::ofstream &ofs) const{
    std::lock_guard<std::mutex> lk(mtx);
    for(auto &state:states){
        for (auto &jname : joint_names) {
            double pos = state.get_position(jname);
            ofs << std::to_string(pos)<<",";
        }
        ofs << std::endl;
    }
}

void JointStateList::loadStates(std::ifstream &ifs,bool async){
    jlist_loader->init(joint_names);
    jlist_loader->registerCallbacks(std::bind(&JointStateList::available_cb,this,std::placeholders::_1),std::bind(&JointStateList::finish_cb,this,std::placeholders::_1));
    jlist_loader->run(ifs);
    if (!async) {
        jlist_loader->wait();
    }
}

