#include "Robot.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <stdio.h>
#include <thread>
#include <future>
#include <condition_variable>
#include <mutex>
#include <functional>
#include <string>
// #include <Eigen/Dense>

std::vector<std::vector<double>> Robot::forward_kinematics(std::vector<std::vector<double>> configs) const
{
    /*
    summary: the function takes in a batch of configurations and computes corresponding end-effector positions
    param configs:
        1st dim - an array (vector) of different configurations;
                    dim1 = number of configs
        2nd dim - an array (vector) of settings (i.e. length/angle) for each Joint (idx_0 is base and idx_end is end-effector);
                    dim2 = joints.size()
    return: a 2d vector 
        1st dim - various configurations
        2nd dim - xyz cooridnates
    */
    std::vector<double> temp{1., 2., 3.};
    std::vector<std::vector<double>> end_positions;
    for (int i = 0; i < configs.size(); ++i)
    {
        end_positions.push_back(temp);
    }
    return end_positions;
}

void print_1dVec(std::vector<double> vec)
{
    std::cout << " =========== vector.begin() ========= " << std::endl;
    std::cout << "[ ";
    for (int j = 0; j < vec.size(); ++j)
    {
        std::cout << vec[j] << " ";
    }
    std::cout << "] ";
    std::cout << std::endl;
    std::cout << " ============ vector.end() ========== " << std::endl;
}

void print_2dVec(std::vector<std::vector<double>> vec)
{
    std::cout << " =========== vector.begin() ========= " << std::endl;
    for (int i = 0; i < vec.size(); ++i)
    {
        std::cout << "[ ";
        for (int j = 0; j < vec[i].size(); ++j)
        {
            std::cout << vec[i][j] << " ";
        }
        std::cout << "] ";
        std::cout << std::endl;
    }
    std::cout << " ============ vector.end() ========== " << std::endl;
}

void print_map(std::unordered_map<std::vector<double>, std::vector<std::vector<double>>, hashFunction> const &m)
{
    for (auto const &pair : m)
    {
        std::cout << "\nMap Key" << std::endl;
        print_1dVec(pair.first);
        std::cout << "Map Value" << std::endl;
        print_2dVec(pair.second);
        std::cout << std::endl;
        // std::cout << ": Value" << std::endl;
        // print_2dVec(pair.second);
        // std::cout << "}\n";
    }
}

void Robot::save2map(std::vector<std::vector<double>> &pos,
                     std::vector<std::vector<double>> &configs)
{
    /*
    param this->point_cloud_: a hashmap containing point cloud entries (keys) and corresponding configurations (values)
    param pos: the end positions computed from forward_kinematics
    param configs: corresponding configurations of the robotic arm
    */

    assert((pos.size() == configs.size()) && "Input dimension sanity check.");

    for (int i = 0; i < pos.size(); ++i)
    {
        // this->point_cloud_[pos[i]].push_back(configs[i]);
        if (this->point_cloud_.find(pos[i]) != this->point_cloud_.end())
        {
            std::cout << "Append new configurations to existing entries" << std::endl;
            // print_1dVec(pos[i]);
            // print_1dVec(configs[i]);
            // std::cout << std::endl;
            this->point_cloud_[pos[i]].push_back(configs[i]);
        }
        else
        {
            std::cout << "Creating new key-value pair" << std::endl;
            // print_1dVec(pos[i]);
            // print_1dVec(configs[i]);
            // std::cout << std::endl;
            this->point_cloud_[pos[i]].push_back(configs[i]);
        }
    }
}

// =================================== VERSION 2 ===========================================
void Robot::get_workspace(std::vector<std::vector<double>> configs, const int nThreads)
{
    /*
    param configs: a 2D array (vector) that stores all possible configurations
    param nThreads: number of threads available
    */
    int num_jobs = configs.size();
    // WorkerThread *subThr = new WorkerThread[nThreads];
    std::vector<std::vector<double>> all_results;
    for (int i = 0; i < nThreads; ++i)
    {
        // slice the vector
        auto start = configs.begin() + i*(num_jobs / nThreads);
        auto end = configs.begin() + (i+1)*(num_jobs / nThreads); // not including the last element
        if (end == configs.begin()) end = configs.end();
        if (end >= configs.end()) end = configs.end();
        auto some_configs = std::vector<std::vector<double>>(start, end); // same type as configs
        // feed to threads
        auto fut = std::async(std::launch::async, &Robot::forward_kinematics, this, some_configs);
        // typecast future objects; a vector of xyz coordinates
        std::vector<std::vector<double>> thread_res = fut.get();
        save2map(thread_res, some_configs);
        // ======================================================================================
        // // insert thread_results into all_results
        // all_results.insert(all_results.end(),
        //                 std::make_move_iterator(thread_res.begin()),
        //                 std::make_move_iterator(thread_res.end()));
    }
    // for (int i = 0; i < nThreads; ++i)
    // {
    //     subThr[i].Wait();
    // }
    // delete[] subThr;
    // subThr = nullptr;
}

int main(void)
{
    Robot robot;
    std::vector<double> temp{0, 0, 0, 0, 0};
    std::vector<std::vector<double>> configs{temp, temp, temp, temp, temp, temp, temp, temp};
    robot.get_workspace(configs, 8);
    print_map(robot.point_cloud_);
    return 0;
}

// ================================== UNUSED FUNCTIONS =====================================

// =================================== MULTITHREADING ======================================
// class WorkerThread
// {
// private:
//     enum
//     {
//         TASK_NONE,
//         TASK_QUIT,
//         TASK_RUN
//     };
//
//     int taskType = TASK_NONE;
//     std::function<void()> task;
//     std::thread thr;
//     std::mutex mtx;
//     std::condition_variable cond;
//     void ThreadFunc();
//
// public:
//     WorkerThread();
//     ~WorkerThread();
//     void Run(std::function<void()>);
//     void Wait(void);
// };
// 
// // constructor; on main thread; prestart the threads
// WorkerThread::WorkerThread()
// {
//     std::thread t(&WorkerThread::ThreadFunc, this);
//     thr.swap(t);
// }
// // destructor; on main thread
// WorkerThread::~WorkerThread()
// {
//     taskType = TASK_QUIT;
//     cond.notify_one();
//     thr.join();
// }
// void WorkerThread::Run(std::function<void()> newTask)
// {
//     mtx.lock();
//     taskType = TASK_RUN;
//     task = newTask;
//     mtx.unlock();
//     cond.notify_one();
// }
// void WorkerThread::Wait(void)
// {
//     std::unique_lock<std::mutex> lock(mtx);
//     cond.wait(lock, [&]{ return taskType == TASK_NONE; });
// }
// 
// void WorkerThread::ThreadFunc()
// {
//     for (;;)
//     {
//         std::unique_lock<std::mutex> lock(mtx);
//         cond.wait(lock, [&]{ return taskType != TASK_NONE; });
//         if (TASK_QUIT == taskType)
//         {
//             break;
//         }
//         else if (TASK_RUN == taskType)
//         {
//             task();
//             taskType = TASK_NONE;
//             cond.notify_one();
//         }
//     }
// }

// =================================== MULTITHREADING ======================================

// =================================== VERSION 1 ===========================================
// void Robot::get_workspace(std::vector<std::vector<double>> configs, const int nThreads) const
// {
//     /*
//     param configs: a 2D array (vector) that stores all possible configurations
//     param nThreads: number of threads available
//     */
//     int num_jobs = configs.size();
//     WorkerThread *subThr = new WorkerThread[nThreads];
//     std::vector<std::vector<double>> all_results;
//     for (int i = 0; i < nThreads; ++i)
//     {
//         // slice the vector
//         auto start = configs.begin() + i*(num_jobs / nThreads);
//         auto end = configs.begin() + (i+1)*(num_jobs / nThreads); // not including the last element
//         (end >= configs.end()) ? end = configs.end();
//         auto some_configs = std::vector<std::vector<double>>(start, end);
//         // feed to threads
//         std::vector<double> results;
//         subThr[i].Run([&]{ results = forward_kinematics(some_configs); });
//         all_results.push_back(results);
//     }
//     for (int i = 0; i < nThreads; ++i)
//     {
//         subThr[i].Wait();
//     }
//     delete[] subThr;
//     subThr = nullptr;
// }