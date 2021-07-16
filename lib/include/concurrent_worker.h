#pragma once

#include <glog/logging.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <mutex>
#include <thread>
#include <vector>
#include "tic_toc.hpp"

namespace jf {

/**
 *
 * @tparam inputT
 * @tparam outputT
 * @tparam Options
 *
 * @param inputs [in]
 * @param is [in] start index
 * @param ie [in] end index
 * @param func [in]
 * @param outputs [in, out]
 * @param mutex_ [in] mutex to assure thread safe of @param outputs
 * @param options [in] options to pass to func
 */
template <typename inputT, typename outputT, typename... Options>
inline void workerSingleThread(
		const std::vector<inputT>& inputs, size_t is, size_t ie,
		std::function<outputT(const inputT&, Options... options)> func,
		std::vector<outputT>& outputs, std::mutex& mutex_, Options... options) {
	for (size_t i = is; i < ie; ++i) {
		// DO NOT use perfect forwarding here, moveable types in options gets moved
		// afterwards
		// and leaves invalid data fornext iteration
		//    outputT output = func(inputs[i], std::forward<Options>(options)...);
		
		outputT output = func(inputs[i], options...);
		{
			std::unique_lock<std::mutex> lck(mutex_);
			outputs.emplace_back(std::move(output));
		}
		//    LOG_EVERY_N(INFO, inputs.size() / 50)
		//      << "Concurrent worker progress : "
		//      << outputs.size() << "/" << inputs.size();
	}
}

/**
 *
 * @tparam inputT
 * @tparam outputT
 * @tparam Options
 * @param inputs
 * @param func
 * @param num_threads
 * @param options
 * @return
 */
template <typename inputT, typename outputT, typename... Options>
inline std::vector<outputT> concurrentWorker(
		const std::vector<inputT>& inputs,
		std::function<outputT(const inputT&, Options... options)> func,
		int num_threads, Options... options) {
	size_t n = inputs.size();
	
	int thread_num =
			num_threads == 0 ? std::thread::hardware_concurrency() - 1 : num_threads;
	
	int delt = (int)n / thread_num;
	
	std::vector<size_t> input_idx;
	for (size_t i = 0; i < thread_num; ++i) input_idx.push_back(i * delt);
	input_idx.push_back(n);
	
	std::vector<outputT> outputs;
	std::thread threads[thread_num];
	std::mutex mutex_;
	for (size_t i = 0; i < thread_num; ++i)
		threads[i] = std::thread(
				workerSingleThread<inputT, outputT, Options...>, std::ref(inputs),
				input_idx[i], input_idx[i + 1], std::ref(func), std::ref(outputs),
				
				// DO NOT use perfect forwarding here, moveable types in options gets
				// moved afterwards
				// and leaves invalid data fornext iteration
				// std::ref(mutex_), std::forward<Options>(options)...);
				std::ref(mutex_), options...);
	
	for (auto& th : threads) th.join();
	return outputs;
}

/**
 *
 * @tparam inputT
 * @tparam outputT
 * @tparam Options
 *
 * @param inputs [in]
 * @param is [in] start index
 * @param ie [in] end index
 * @param func [in]
 * @param outputs [in, out]
 * @param mutex_ [in] mutex to assure thread safe of @param outputs
 * @param options [in] options to pass to func
 */
template <typename inputT, typename... Options>
inline void workerSingleThreadRef(
		std::vector<inputT>& inputs, size_t is, size_t ie,
		std::function<void(inputT&, Options... options)> func, Options... options) {
	for (size_t i = is; i < ie; ++i) {
		// DO NOT use perfect forwarding here, moveable types in options gets moved
		// afterwards
		// and leaves invalid data fornext iteration
		//    outputT output = func(inputs[i], std::forward<Options>(options)...);
		
		func(inputs[i], options...);
		
		//    LOG_EVERY_N(INFO, inputs.size() / 5)
		//      << "Concurrent worker progress : "
		//      << outputs.size() << "/" << inputs.size();
	}
}

/**
 *
 * @tparam inputT
 * @tparam outputT
 * @tparam Options
 * @param inputs
 * @param func
 * @param num_threads
 * @param options
 * @return
 */
template <typename inputT, typename... Options>
inline void concurrentWorker(
		std::vector<inputT>& inputs,
		std::function<void(inputT&, Options... options)> func, int num_threads,
		Options... options) {
	size_t n = inputs.size();
	
	int thread_num =
			num_threads == 0 ? std::thread::hardware_concurrency() - 1 : num_threads;
	
	int delt = (int)n / thread_num;
	
	std::vector<size_t> input_idx;
	for (size_t i = 0; i < thread_num; ++i) input_idx.push_back(i * delt);
	input_idx.push_back(n);
	
	std::thread threads[thread_num];
	for (size_t i = 0; i < thread_num; ++i)
		threads[i] = std::thread(
				workerSingleThreadRef<inputT, Options...>, std::ref(inputs),
				input_idx[i], input_idx[i + 1], std::ref(func),
				
				// DO NOT use perfect forwarding here, moveable types in options gets
				// moved afterwards
				// and leaves invalid data fornext iteration
				// std::ref(mutex_), std::forward<Options>(options)...);
				options...);
	
	for (auto& th : threads) th.join();
}

/**
 *
 * @tparam inputT
 * @tparam outputT
 * @tparam Options
 *
 * @param inputs [in]
 * @param is [in] start index
 * @param ie [in] end index
 * @param func [in]
 * @param outputs [in, out]
 * @param mutex_ [in] mutex to assure thread safe of @param outputs
 * @param options [in] options to pass to func
 */
template <typename inputT, typename outputT, typename... Options>
inline void nonworkerSingleThread(
		std::vector<inputT>& inputs, size_t is, size_t ie,
		std::function<outputT(inputT&, Options... options)> func,
		std::vector<outputT>& outputs, std::mutex& mutex_, Options... options) {
	for (size_t i = is; i < ie; ++i) {
		// DO NOT use perfect forwarding here, moveable types in options gets moved
		// afterwards
		// and leaves invalid data fornext iteration
		//    outputT output = func(inputs[i], std::forward<Options>(options)...);
		
		outputT output = func(inputs[i], options...);
		{
			std::unique_lock<std::mutex> lck(mutex_);
			outputs.emplace_back(std::move(output));
		}
		//    LOG_EVERY_N(INFO, inputs.size() / 5)
		//      << "Concurrent worker progress : "
		//      << outputs.size() << "/" << inputs.size();
	}
}

/**
 *
 * @tparam inputT
 * @tparam outputT
 * @tparam Options
 * @param inputs
 * @param func
 * @param num_threads
 * @param options
 * @return
 */
template <typename inputT, typename outputT, typename... Options>
inline std::vector<outputT> nonconcurrentWorker(
		std::vector<inputT>& inputs,
		std::function<outputT(inputT&, Options... options)> func, int num_threads,
		Options... options) {
	size_t n = inputs.size();
	
	int thread_num =
			num_threads == 0 ? std::thread::hardware_concurrency() - 1 : num_threads;
	
	int delt = (int)n / thread_num;
	
	std::vector<size_t> input_idx;
	for (size_t i = 0; i < thread_num; ++i) input_idx.push_back(i * delt);
	input_idx.push_back(n);
	
	std::vector<outputT> outputs;
	std::thread threads[thread_num];
	std::mutex mutex_;
	for (size_t i = 0; i < thread_num; ++i)
		threads[i] = std::thread(
				nonworkerSingleThread<inputT, outputT, Options...>, std::ref(inputs),
				input_idx[i], input_idx[i + 1], std::ref(func), std::ref(outputs),
				
				// DO NOT use perfect forwarding here, moveable types in options gets
				// moved afterwards
				// and leaves invalid data fornext iteration
				// std::ref(mutex_), std::forward<Options>(options)...);
				std::ref(mutex_), options...);
	
	for (auto& th : threads) th.join();
	return outputs;
}

/**
 * @brief inherit this in your implement and override run();
 *        you'd better use "input_", so that the reporter can get the size of
 * jobs.
 * @tparam inputT : can be any type, can be ignored anyway ,recommended to use.
 * @tparam bufferT : Must be the SAME with consumer!!
 */
template <typename inputT, typename bufferT>
class WorkProducer {
 public:
	using BufferQueue = tbb::concurrent_bounded_queue<std::pair<bool, bufferT>>;
	typedef std::shared_ptr<WorkProducer> Ptr;
	virtual ~WorkProducer(){};
	virtual void run() = 0;
	void setBuffer(std::shared_ptr<BufferQueue> buffer) { buffer_ = buffer; }
	size_t size() { return input_.size(); }
 
 protected:
	inputT input_;
	std::shared_ptr<BufferQueue> buffer_;
};

/**
 * @brief warpper class for the process you need . inherit this to override
 *          "run(bufferT& this_buffer)".
 * @tparam bufferT : Must be the SAME with producer!!!!
 * @tparam outputT :
 */
template <typename bufferT, typename outputT>
class WorkConsumer {
 public:
	typedef std::shared_ptr<WorkConsumer> Ptr;
	virtual ~WorkConsumer(){};
	virtual void run(bufferT& this_buffer) = 0;
	void setWorkResults(
			std::shared_ptr<tbb::concurrent_vector<outputT>> work_results) {
		work_results_ = work_results;
	}
 
 protected:
	std::shared_ptr<tbb::concurrent_vector<outputT>> work_results_;
};

// Todo : with this class. we can use a container template which supports
// forward
//        iterator, instead of just vector<inputT> and vector<outputT>
/**
 * @brief This class provides multi-thread method with 1 thread producer
 *          and multiple threads consumers.
 *          Producer preprocess items from io module and put them into buffer.
 *          Consumers waits for jobs in the buffer and output result in
 *          tbb::vector<outputT>
 */
template <typename inputT, typename bufferT, typename outputT>
class ConcurrentClass {
 public:
	ConcurrentClass(typename WorkProducer<inputT, bufferT>::Ptr work_producer,
	                typename WorkConsumer<bufferT, outputT>::Ptr work_consumer,
	                int num_threads_consumer = 0, int buffer_capacity = 2)
			: work_producer_(work_producer), work_consumer_(work_consumer) {
		// prepare producer
		buffer_queue_ = std::make_shared<BufferQueue>();
		buffer_queue_->set_capacity(buffer_capacity);
		work_producer_->setBuffer(buffer_queue_);
		// prepare consumer
		results_ = std::make_shared<tbb::concurrent_vector<outputT>>();
		results_->reserve(work_producer_->size());
		work_consumer_->setWorkResults(results_);
		num_threads_consumer_ = num_threads_consumer == 0
		                        ? (std::thread::hardware_concurrency() - 1)
		                        : num_threads_consumer;
	}
	
	tbb::concurrent_vector<outputT> run() {
		std::thread producer_threads;
		producer_threads = std::thread(&ConcurrentClass::produce, this);
		
		std::thread consumer_threads[num_threads_consumer_];
		for (size_t i = 0; i < num_threads_consumer_; ++i) {
			consumer_threads[i] = std::thread(&ConcurrentClass::consume, this);
		}
		producer_threads.join();
		std::pair<bool, bufferT> stop_mark;
		stop_mark.first = true;
		for (int i = 0; i < num_threads_consumer_; ++i)
			buffer_queue_->push(stop_mark);
		for (auto& th : consumer_threads) th.join();
		return *results_;
	}
 
 private:
	void produce() { work_producer_->run(); }
	
	void consume() {
		while (true) {
			std::pair<bool, bufferT> work;
			buffer_queue_->pop(work);
			if (work.first) break;
			work_consumer_->run(work.second);
			report(10);
		}
	}
	
	void report(int every_n) {
		if (results_->size() % every_n == 0 ||
		    results_->size() > work_producer_->size() - 20) {
			LOG(INFO) << "===========================================";
			LOG(INFO) << t_.toc("time elapsed since beginning: ", false);
			LOG(INFO) << "Work progress: " << results_->size() << "/"
			          << work_producer_->size() << " = "
			          << results_->size() * 100.0 / work_producer_->size() << "%";
			auto efficiency = t_.toc_milli(false) / 1000.0 / results_->size();
			LOG(INFO) << "Global efficiency: " << efficiency << " sec per item";
			LOG(INFO) << "Single thread efficiency: "
			          << efficiency * num_threads_consumer_ << " sec per item";
			
			LOG(INFO) << "Time left: "
			          << (work_producer_->size() - results_->size()) * efficiency
			          << " seconds";
			LOG(INFO) << "Buffer Volume: " << buffer_queue_->size() << "/"
			          << buffer_queue_->capacity();
			LOG(INFO) << "===========================================";
		}
	}
 
 private:
	using BufferQueue = tbb::concurrent_bounded_queue<std::pair<bool, bufferT>>;
	// all_works_ to input_queue_(single thread)
	typename WorkProducer<inputT, bufferT>::Ptr work_producer_;
	std::shared_ptr<BufferQueue> buffer_queue_;
	// input_queue_ to results_(thread safe)
	int num_threads_consumer_;
	typename WorkConsumer<bufferT, outputT>::Ptr work_consumer_;
	std::shared_ptr<tbb::concurrent_vector<outputT>> results_;
	TicToc t_;
};

}  // namespace jf
