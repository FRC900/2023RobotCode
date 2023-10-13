// Simple class to track cumulative / average execution
// time of a block of code

#ifndef TRACER_INC__
#define TRACER_INC__

#include <chrono>
#include <unordered_map>
#include <string>

#include <ros/console.h>

class TracerEntry
{
	public:
		TracerEntry()
			: count_{0}
			, total_time_{0.0}
			, started_{false}
		{
		}
		TracerEntry(const TracerEntry &) = delete;
		TracerEntry(TracerEntry &&) = default;
		~TracerEntry() = default;
		TracerEntry operator=(const TracerEntry &) = delete;
		TracerEntry &operator=(TracerEntry &&) = delete;

		void reset()
		{
			count_ = 0;
			total_time_ = std::chrono::duration<double>{0.0};
		}

		size_t count_;
		std::chrono::duration<double> total_time_;
		std::chrono::high_resolution_clock::time_point start_time_;
		bool started_;
};

class Tracer
{
	public:
		Tracer(const std::string &name)
			: name_(name)
			, last_report_time_(ros::Time::now())
		{
		}
		Tracer(const Tracer &) = delete;
		Tracer(Tracer &&) = default;
		~Tracer() = default;
		Tracer operator=(const Tracer &) = delete;
		Tracer &operator=(Tracer &&) = delete;

		// Mark the start of an event to time.  If the named event exists,
		// use it. Otherwise create a new event entry in the map of events
		// tracked by this Tracer object
		void start(const std::string &label)
		{
			auto entry = map_.find(label);

			// If not found, create a new entry for this label
			if (entry == map_.end())
			{
				map_.emplace(label, TracerEntry());
				entry = map_.find(label);
			}

			if (entry == map_.end())
			{
				ROS_ERROR_STREAM("Tracer::start : failed to insert label " << label);
				return;
			}

			if (entry->second.started_)
			{
				ROS_WARN_STREAM("Tracer::start : start called on already started label " << label);
				return;
			}
			entry->second.start_time_ = std::chrono::high_resolution_clock::now();
			entry->second.started_ = true;
		}

		// Stop all previously started timers,
		// then start the requested one
		void start_unique(const std::string &label)
		{
			stop();
			start(label);
		}

		void stop(const std::string &label)
		{
			auto entry = map_.find(label);

			// If not found, create a new entry for this label
			if (entry == map_.end())
			{
				ROS_ERROR_STREAM("Tracer::stop : couldn't find label " << label);
				return;
			}

			if (!entry->second.started_)
			{
				ROS_WARN_STREAM("Tracer::stop : label " << label << " not started");
				return;
			}
			stopEntry(entry->second);
		}

		// Stop all previously started timers
		void stop(void)
		{
			for (auto &it : map_)
			{
				if (it.second.started_)
				{
					stopEntry(it.second);
				}
			}
		}

		void report(const double timeout, const bool auto_stop = true)
		{
			if (auto_stop)
			{
				stop();
			}
			const auto now = ros::Time::now();
			if (((now - last_report_time_).toSec() >= timeout) ||
				(now < last_report_time_))
			{
				std::stringstream s;
				s << name_ << ":" << std::endl;
				for (auto &it : map_)
				{
					const double avg_time = it.second.total_time_ / std::chrono::duration<double>(it.second.count_);
					s << "\t" << it.first << " = " << avg_time << ", count = " << it.second.count_ << std::endl;
					it.second.reset();
				}
				ROS_INFO_STREAM(s.str());
				last_report_time_ = now;
			}
		}

	private:

		void stopEntry(TracerEntry &entry)
		{
			entry.total_time_ += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - entry.start_time_);
			entry.count_ += 1;
			entry.started_ = false;

		}
		std::string name_;
		std::unordered_map<std::string, TracerEntry> map_;
		ros::Time last_report_time_;
};

#endif