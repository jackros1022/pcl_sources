/*
Semaphore,是负责协调各个线程, 以保证它们能够正确、合理的使用公共资源。也是操作系统中用于控制进程同步互斥的量。
*/

#ifndef OBJECT_RECOGNITION_SEMAPHORE
#define OBJECT_RECOGNITION_SEMAPHORE

#include <mutex>
#include <condition_variable>

class Semaphore
{
private:
    std::mutex mutex_;
    std::condition_variable threads_condition_;
    std::condition_variable main_condition_;
    int count_;
    int max_count_;
    bool stop_;

public:
	// 构造函数
    Semaphore(int max_count): count_(0), max_count_(max_count), stop_(false){}

	void Notify2main()
	{
	    std::unique_lock<std::mutex> lock(mutex_);
	    if(++count_ == max_count_){
	    	threads_condition_.notify_all();
	    	count_ = 0;
	    }
	}

	void Wait4threads()
	{
	    std::unique_lock<std::mutex> lock(mutex_);
	    if(count_ < max_count_ )
	        threads_condition_.wait(lock);
	}

	void Notify2threads()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		main_condition_.notify_all();

	}

	void Wait4main()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		main_condition_.wait(lock);
	}

	void SetStop()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		stop_ = !stop_;
	}

	bool ToStop()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		return stop_;
	}

};

#endif // OBJECT_RECOGNITION_SEMAPHORE