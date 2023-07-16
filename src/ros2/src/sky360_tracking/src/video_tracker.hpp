#pragma once
#ifndef __VIDEO_TRACKER_HPP__
#define __VIDEO_TRACKER_HPP__

#include <vector>
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <vector>
#include <queue>

#include <opencv2/opencv.hpp>

#include "tracker.hpp"

class VideoTracker
{
public:
    VideoTracker(const std::map<std::string, std::string> &settings, rclcpp::Logger logger)
        : settings(settings), total_trackers_finished(0), total_trackers_started(0), logger_(logger)
    {
        tracker_min_centre_point_distance_between_bboxes = 64; // settings["tracker_min_centre_point_distance_between_bboxes"]
        tracker_max_active_trackers = 10;                      // settings["tracker_max_active_trackers"]
    }

    bool is_tracking() const
    {
        return !live_trackers.empty();
    }

    std::vector<Tracker> get_active_trackers() const
    {
        std::vector<Tracker> trackers;
        for (const auto &tracker : live_trackers)
        {
            if (tracker.is_tracking())
            {
                trackers.push_back(tracker);
            }
        }
        return trackers;
    }

    const std::vector<Tracker> &get_live_trackers() const
    {
        return live_trackers;
    }

    void create_and_add_tracker(const cv::Mat &frame, const cv::Rect &bbox)
    {
        total_trackers_started += 1;

        Tracker tracker(settings, total_trackers_started, frame, bbox, logger_);
        tracker.update(frame);
        live_trackers.push_back(tracker);
    }

    void update_trackers(const std::vector<cv::Rect> &bboxes, const cv::Mat &frame)
    {
        std::vector<cv::Rect> unmatched_bboxes = bboxes;
        std::vector<Tracker> failed_trackers;
        const int tracker_count = live_trackers.size();

        std::vector<std::thread> threads(tracker_count);
        std::vector<std::pair<bool, cv::Rect>> results(tracker_count);

        for (int i = 0; i < tracker_count; ++i)
        {
            threads[i] = std::thread([this, &frame, &results, i]() { // update_tracker_task(live_trackers[i], frame, results[i]);
                results[i] = live_trackers[i].update(frame);
            });
        }

        for (auto &t : threads)
        {
            t.join();
        }

        for (int i = 0; i < tracker_count; ++i)
        {
            bool ok;
            cv::Rect bbox;
            std::tie(ok, bbox) = results[i];

            if (!ok)
            {
                failed_trackers.push_back(live_trackers[i]);
            }

            for (auto &new_bbox : bboxes)
            {
                auto it = std::find(unmatched_bboxes.begin(), unmatched_bboxes.end(), new_bbox);
                if (it != unmatched_bboxes.end())
                {
                    if (calc_centre_point_distance(bbox, new_bbox) < tracker_min_centre_point_distance_between_bboxes)
                    {
                        unmatched_bboxes.erase(it);
                    }
                }
            }
        }

        for (auto &tracker : failed_trackers)
        {
            live_trackers.erase(std::remove(live_trackers.begin(), live_trackers.end(), tracker), live_trackers.end());
            total_trackers_finished += 1;
        }

        for (auto &new_bbox : unmatched_bboxes)
        {
            if (is_valid_bbox(new_bbox, frame))
            {
                if (live_trackers.size() < tracker_max_active_trackers)
                {
                    if (!is_bbox_being_tracked(live_trackers, new_bbox))
                    {
                        create_and_add_tracker(frame, new_bbox);
                    }
                }
            }
        }
    }

    static void update_tracker_task(Tracker &tracker, const cv::Mat &frame, std::pair<bool, cv::Rect> &result)
    {
        result = tracker.update(frame);
    }

    static int calc_centre_point_distance(const cv::Rect &bbox1, const cv::Rect &bbox2)
    {
        cv::Point2f c1 = cv::Point2f(bbox1.x + bbox1.width / 2.0f, bbox1.y + bbox1.height / 2.0f);
        cv::Point2f c2 = cv::Point2f(bbox2.x + bbox2.width / 2.0f, bbox2.y + bbox2.height / 2.0f);

        return cv::norm(c1 - c2);
    }

    static bool is_valid_bbox(const cv::Rect &bbox, const cv::Mat &frame)
    {
        int max_dim = static_cast<int>(frame.cols * 0.15); // 15% of the width of the frame

        if (bbox.width < 1 || bbox.height < 1 || bbox.x < 1 || bbox.y < 1 || bbox.width > max_dim || bbox.height > max_dim)
        {
            return false;
        }

        if (bbox.x + bbox.width >= frame.cols || bbox.y + bbox.height >= frame.rows)
        {
            return false;
        }

        return true;
    }

    static bool is_bbox_being_tracked(std::vector<Tracker> &live_trackers, cv::Rect bbox)
    {
        for (auto &tracker : live_trackers)
        {
            if (tracker.is_bbox_contained(bbox) || tracker.does_bbox_overlap(bbox))
            {
                return true;
            }
        }
        return false;
    }

    int get_total_trackers_started() const
    {
        return total_trackers_started;
    }

    int get_total_trackers_finished() const
    {
        return total_trackers_finished;
    }

    size_t get_total_live_trackers() const
    {
        return live_trackers.size();
    }

    size_t get_total_trackable_trackers() const
    {
        return std::count_if(live_trackers.begin(), live_trackers.end(), [](const Tracker &tracker)
                      { return tracker.is_tracking(); });
    }

private:
    std::map<std::string, std::string> settings;
    int total_trackers_finished;
    int total_trackers_started;
    std::vector<Tracker> live_trackers;
    int tracker_min_centre_point_distance_between_bboxes;
    size_t tracker_max_active_trackers;
    rclcpp::Logger logger_;
};

#endif