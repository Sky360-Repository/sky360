#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <chrono>

namespace sky360lib::utils
{
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::duration<double, std::nano>;

    struct ProfilerData
    {
        std::string name;
        TimePoint start_time;
        Duration duration;
        int count;

        double fps() const
        {
            return duration.count() > 0 ? (double)count / duration_in_seconds() : 0.0;
        }

        double avg_time_in_ns() const
        {
            return count > 0 ? duration.count() / count : 0.0;
        }

        double avg_time_in_s() const
        {
            return count > 0 ? (duration.count() * 1e-9) / count : 0.0;
        }

        double duration_in_seconds() const
        {
            return duration.count() * 1e-9;
        }
    };

    using DataMap = std::unordered_map<std::string, ProfilerData>;

    class Profiler
    {
    public:
        inline void start(const std::string &region)
        {
            auto time = Clock::now();
            if (m_profiler_data.empty())
                start_time = time;

            auto& data = m_profiler_data[region];
            if (data.name.empty())
                data.name = region;
            data.start_time = time;
        }

        inline void stop(const std::string &region)
        {
            auto& data = m_profiler_data[region];
            Duration elapsed_time = Clock::now() - data.start_time;
            data.duration += elapsed_time;
            data.count++;
        }

        inline void reset()
        {
            m_profiler_data.clear();
        }

        inline ProfilerData const& get_data(const std::string &region) const
        {
            return m_profiler_data.at(region);
        }

        inline DataMap const & get_data() const
        {
            return m_profiler_data;
        }

        inline TimePoint get_start_time() const
        {
            return start_time;
        }

        std::string report() const
        {
            auto stop_time = Clock::now();
            std::string report;
            for (const auto &entry : m_profiler_data)
            {
                report += "\n" + report_individual(entry.second, stop_time);
            }
            return report;
        }

        std::string report_individual(const ProfilerData& data, TimePoint stop_time = Clock::now()) const
        {
            std::ostringstream oss;
            auto totalDuration = stop_time - start_time;
            oss << "Region: " << data.name
                << ", Average Time (ns): " << data.avg_time_in_ns()
                << ", Average Time (s): " << data.avg_time_in_s()
                << ", Count: " << data.count
                << ", FPS: " << data.fps()
                << ", %: " << (data.duration / totalDuration) * 100.0;
            return oss.str();
        }

    private:
        DataMap m_profiler_data;
        TimePoint start_time;
    };
}