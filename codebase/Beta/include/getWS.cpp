#include "Robot.h"

std::vector<std::vector<double>> Robot::generate_config(int resolution) const {
  std::vector<std::vector<double>> configs;
  return configs;
}

std::vector<std::vector<double>> Robot::forward_kinematics(
    std::vector<std::vector<double>> configs, bool testing) const
{
  /*
  summary: the function takes in a batch of configurations and computes
  corresponding end-effector positions 
  param configs: 
    1st dim - an array (vector) of different configurations; dim1 = number of configs 
    2nd dim - an array (vector) of settings (i.e. length/angle) for each Joint (idx_0 is base
      and idx_end is end-effector); dim2 = joints.size() 
  param testing: indicate whether to use the testing version or not
  return: a 2d vector 
    1st dim - various configurations 
    2nd dim - xyz cooridnates
  */
  if (!testing)
  {
    std::vector<std::vector<double>> end_positions = {};
    for (int i = 0; i < configs.size(); ++i)
    {
      Eigen::Vector3d single_res = Robot::forward_kinematics_single(configs[i]);
      end_positions[i].push_back(double(single_res[0]));
      end_positions[i].push_back(double(single_res[1]));
      end_positions[i].push_back(double(single_res[2]));
    }
    return end_positions;
  }
  else
  {
    // =============================== FOR TESTING ONLY ===================================
    /*
    In order to test the functionality of get_workspace, we for now assume forward_kinematics
    would be properly implemented in the future and will return correct results. This version
    only generates a fake output that is 10 times the value of the first element in config.
    */
    std::vector<std::vector<double>> end_positions;
    for (int i = 0; i < configs.size(); ++i) {
      std::vector<double> single_res(3, configs[i][0]*10);
      end_positions.push_back(single_res);
    }
    return end_positions;
    // =====================================================================================
  }
}



void Robot::save2map(std::vector<std::vector<double>> &pos,
                     std::vector<std::vector<double>> &configs) {
  /*
  param pos: the end positions computed from forward_kinematics 
  param configs: corresponding configurations of the robotic arm
  */

  assert((pos.size() == configs.size()) && "Input dimension sanity check.");

  for (int i = 0; i < pos.size(); ++i) {
    // this->point_cloud_[pos[i]].push_back(configs[i]);
    if (this->point_cloud_.find(pos[i]) != this->point_cloud_.end()) {
      std::cout << "Append new configurations to existing entries" << std::endl;
      // print_1dVec(pos[i]);
      // print_1dVec(configs[i]);
      // std::cout << std::endl;
      this->point_cloud_[pos[i]].push_back(configs[i]);
    } else {
      std::cout << "Creating new key-value pair" << std::endl;
      // print_1dVec(pos[i]);
      // print_1dVec(configs[i]);
      // std::cout << std::endl;
      this->point_cloud_[pos[i]].push_back(configs[i]);
    }
  }
}

// ===================================== VERSION 2 ===========================================
void Robot::get_workspace(std::vector<std::vector<double>> configs, int nThreads, bool testing) 
{
  /*
  param configs: a 2D array (vector) that stores all possible configurations
  param nThreads: number of threads available
  */
  int num_jobs = configs.size();
  if (num_jobs < nThreads)
    nThreads = num_jobs; // if fewer tasks than nThreads, then we use fewer number of threads
  // WorkerThread *subThr = new WorkerThread[nThreads];
  // std::vector<std::vector<double>> all_results;
  for (int i = 0; i < nThreads; ++i) {
    // slice the vector
    auto start = configs.begin() + i * (num_jobs / nThreads);
    auto end = configs.begin() + (i + 1) * (num_jobs / nThreads);  // not including the last element
    // if (end == configs.begin()) end = configs.end();
    if (end >= configs.end()) end = configs.end(); // the last batch could be smaller than regular batch
    auto some_configs = std::vector<std::vector<double>>(start, end);  // same type as configs
    // feed to threads
    auto fut = std::async(std::launch::async, &Robot::forward_kinematics, this,
                          some_configs, testing);
    // typecast future objects; a vector of xyz coordinates
    std::vector<std::vector<double>> thread_res = fut.get();
    save2map(thread_res, some_configs);
    // ======================================================================================
    // // insert thread_results into all_results
    // all_results.insert(all_results.end(),
    //                 std::make_move_iterator(thread_res.begin()),
    //                 std::make_move_iterator(thread_res.end()));
  }
  std::cout << "Number of threads used: " << nThreads << "." << std::endl;
  // for (int i = 0; i < nThreads; ++i)
  // {
  //     subThr[i].Wait();
  // }
  // delete[] subThr;
  // subThr = nullptr;
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
// void Robot::get_workspace(std::vector<std::vector<double>> configs, const int
// nThreads) const
// {
//     /*
//     param configs: a 2D array (vector) that stores all possible
//     configurations param nThreads: number of threads available
//     */
//     int num_jobs = configs.size();
//     WorkerThread *subThr = new WorkerThread[nThreads];
//     std::vector<std::vector<double>> all_results;
//     for (int i = 0; i < nThreads; ++i)
//     {
//         // slice the vector
//         auto start = configs.begin() + i*(num_jobs / nThreads);
//         auto end = configs.begin() + (i+1)*(num_jobs / nThreads); // not
//         including the last element (end >= configs.end()) ? end =
//         configs.end(); auto some_configs =
//         std::vector<std::vector<double>>(start, end);
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