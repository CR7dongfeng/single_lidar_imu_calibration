#pragma once

#include <chrono>
#include <sstream>
#include <string>

namespace jf {

class TicToc {
 public:
	/**
	 * Constructor, will start timing
	 */
	TicToc() { tic(); }
	
	/**
	 * Reset the timer
	 */
	void tic();
	
	/**
	 * Auto adaptive timer, output elapsed time in adapted time units
	 * @param message_prefix
	 * @param tic_after_toc
	 * @return example: "time elapsed: 80 ms;"
	 */
	std::string toc(const std::string& message_prefix = "time elapsed: ",
	                bool tic_after_toc = true);
	
	int64_t toc_nano(bool tic_after_toc = true);
	
	int64_t toc_micro(bool tic_after_toc = true);
	
	int64_t toc_milli(bool tic_after_toc = true);
	
	int64_t toc_sec(bool tic_after_toc = true);
	
	int64_t toc_min(bool tic_after_toc = true);
 
 private:
	std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

inline void TicToc::tic() {
	start_ = std::chrono::high_resolution_clock::now();
}

inline std::string TicToc::toc(const std::string& message_prefix,
                               bool tic_after_toc) {
	using namespace std::chrono;
	
	//  auto end_ = high_resolution_clock::now();
	auto diff = high_resolution_clock::now() - start_;
	
	std::stringstream ss;
	
	int64_t nano = duration_cast<nanoseconds>(diff).count();
	int64_t micro = duration_cast<microseconds>(diff).count();
	int64_t milli = duration_cast<milliseconds>(diff).count();
	int64_t sec = duration_cast<seconds>(diff).count();
	int64_t min = duration_cast<minutes>(diff).count();
	
	if (nano < 1000) {
		ss << message_prefix << nano << " ns; ";
	} else if (micro < 1000) {
		ss << message_prefix << micro << " µs; ";
	} else if (milli < 1000) {
		ss << message_prefix << milli << " ms; ";
	} else if (sec < 600) {
		ss << message_prefix << sec << " s; ";
	} else {
		ss << message_prefix << min << " min; ";
	}
	
	if (tic_after_toc) this->tic();
	return ss.str();
}

inline int64_t TicToc::toc_nano(bool tic_after_toc) {
	auto nano = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::high_resolution_clock::now() - start_)
			.count();
	if (tic_after_toc) this->tic();
	return nano;
}

inline int64_t TicToc::toc_micro(bool tic_after_toc) {
	auto micro = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::high_resolution_clock::now() - start_)
			.count();
	if (tic_after_toc) this->tic();
	return micro;
}

inline int64_t TicToc::toc_milli(bool tic_after_toc) {
	auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - start_)
			.count();
	if (tic_after_toc) this->tic();
	return milli;
}

inline int64_t TicToc::toc_sec(bool tic_after_toc) {
	auto sec = std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::high_resolution_clock::now() - start_)
			.count();
	if (tic_after_toc) this->tic();
	return sec;
}

inline int64_t TicToc::toc_min(bool tic_after_toc) {
	auto min = std::chrono::duration_cast<std::chrono::minutes>(
			std::chrono::high_resolution_clock::now() - start_)
			.count();
	if (tic_after_toc) this->tic();
	return min;
}
}
