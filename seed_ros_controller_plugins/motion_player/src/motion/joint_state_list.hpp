#pragma once

#include "common.hpp"
#include "structure_robotstate.hpp"

class JointStateListLoader{
public:
    JointStateListLoader();

    ~JointStateListLoader();

    void init(std::vector<std::string> joint_names){
        this->joint_names = joint_names;
    }

    /** \brief 実行
     *
     *  \param ifs: 入力ストリーム
     */
    void run(std::ifstream &ifs);

    /** \brief 終了待ち
     *
     */
    void wait();

    /** \brief コールバックを登録する
     *
     * \param available_callback:データが一定以上蓄積したら呼び出される
     * \param finish_callback:終了時に呼び出される
     */
    void registerCallbacks(std::function<void(JointStateListLoader*)> available_callback,std::function<void(JointStateListLoader*)> finish_callback);

    /** \brief 読み込んだデータを取得する
     *
     *  コールバックからは、本関数以外呼び出さないこと。
     */
    std::vector<RobotState> getData();


private:


    void loadStates();


private:
    std::function<void(JointStateListLoader*)> available_cb;
    std::function<void(JointStateListLoader*)> finish_cb;

    std::atomic<bool> done = true;
    std::thread th;
    std::ifstream ifs;
    size_t batch_size = 10;
    std::mutex mtx;
    std::vector<RobotState> states_readed;
    std::vector<std::string> joint_names;
};

class JointStateList {
public:
    JointStateList();

    ~JointStateList();

    JointStateList(const JointStateList& rhs):JointStateList(){
        state_size = rhs.state_size;
        joint_names = rhs.joint_names;
        states = rhs.states;
    }

    const JointStateList& operator=(const JointStateList& rhs){
        state_size = rhs.state_size;
        joint_names = rhs.joint_names;
        states = rhs.states;
        return *this;
    }

    /** \brief 状態の総サイズ
     *
     *  読み込まれているとは限らない。
     */
    int size() const;

    /** \brief 読み込まれている状態のサイズ
     *
     */
    int actual_size() const {
        std::lock_guard<std::mutex> lk(mtx);
        return states.size();
    }

    void clear();

    RobotState& operator[](int idx);

    const RobotState& operator[](int idx)const;

    template<class... Args>
    auto insert(Args... args){
        std::lock_guard<std::mutex> lk(mtx);
        auto ret = states.insert(args...);
        state_size = states.size();
        return ret;
    }

    template<class... Args>
    auto erase(Args... args){
        std::lock_guard<std::mutex> lk(mtx);
        auto ret = states.erase(args...);
        state_size = states.size();
        return ret;
    }

    std::vector<RobotState>::iterator begin();

    std::vector<RobotState>::iterator end();

    void saveStates(std::ofstream &ofs) const;

    void loadStates(std::ifstream &ifs,bool async = true);

    void available_cb(JointStateListLoader* loader){
        std::lock_guard<std::mutex> lk(mtx);
        auto data = loader->getData();
        states.insert(states.end(), data.begin(), data.end());
    }

    void finish_cb(JointStateListLoader* loader){
        std::lock_guard<std::mutex> lk(mtx);
        auto data = loader->getData();
        states.insert(states.end(), data.begin(), data.end());
    }

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        if (ar.getDirection() == T::DIRECTION_OUTPUT) {
            joint_names = states[0].get_jnames();
        }
        ar & ARCHIVE_NAMEDVALUE(state_size);
        ar & ARCHIVE_NAMEDVALUE(joint_names);
    }
private:
    JointStateListLoader *jlist_loader = nullptr;
    int state_size = 0; //!< 点列のサイズ
    std::vector<std::string> joint_names;
    std::vector<RobotState> states;
    mutable std::mutex mtx;
};
