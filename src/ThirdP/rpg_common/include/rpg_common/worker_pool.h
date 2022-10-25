#pragma once

#include <atomic>
#include <thread>

#include <glog/logging.h>

#include "rpg_common/threadsafe_queue.h"
#include "rpg_common/worker_base.h"

namespace rpg_common {

template <typename DataType, size_t NumThreads = 8u>
class WorkerPool : public WorkerBase
{
 public:
  WorkerPool() : paused_(true), num_threads_(NumThreads)
 {
    resume();
 }

  virtual ~WorkerPool()
  {
    shutdown();
  }

  void addTask(const DataType& item)
  {
    queue_.push(item);
  }

  virtual void shutdown() override
  {
    if (threads_.empty())
    {
      return;
    }
    queue_.shutdown();
    for (std::thread& thread : threads_)
    {
      thread.join();
    }
    threads_.clear();
  }

  virtual void softShutdown() override
  {
    if (threads_.empty())
    {
      return;
    }
    queue_.waitUntilEmpty();
    shutdown();
  }

  virtual void pause() override
  {
    CHECK(!paused_);
    CHECK(!threads_.empty());
    paused_ = true;
    for (std::thread& thread : threads_)
    {
      thread.join();
    }
    threads_.clear();
  }

  virtual bool isPaused() const override
  {
    return paused_;
  }

  virtual void resume() override
  {
    CHECK(paused_);
    CHECK(threads_.empty());
    paused_ = false;
    for (size_t i = 0u; i < num_threads_; ++i)
    {
      threads_.emplace_back(&WorkerPool<DataType, NumThreads>::workLoop, this);
    }
  }

 private:
  virtual void process(const DataType& item) = 0;

  void workLoop()
  {
    DataType item;
    while (!paused_ && queue_.waitAndPop(&item))
    {
      process(item);
    }
  }

  ThreadSafeQueue<DataType> queue_;
  std::atomic<bool> paused_;
  std::vector<std::thread> threads_;
  const size_t num_threads_;
};

}  // namespace rpg_common
namespace rpg = rpg_common;
