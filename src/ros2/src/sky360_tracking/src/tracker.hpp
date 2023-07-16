#pragma once
#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include <chrono>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "track_prediction.hpp"

#include "../../sky360_shared/include/tracking_state.hpp"

class Tracker
{
public:
    Tracker(const std::map<std::string, std::string> &settings, int id, const cv::Mat &frame, const cv::Rect &bbox, rclcpp::Logger logger)
        : settings(settings), id(id), track_predictor(id, bbox), logger_(logger)
    {
        cv2_tracker = Select(settings);
        cv2_tracker->init(frame, bbox);
        bboxes.push_back(bbox);
        stationary_track_counter = 0;
        active_track_counter = 0;
        tracking_state = TrackingStateEnum::ProvisionaryTarget;
        bbox_to_check = bbox;
        start = std::chrono::system_clock::now();
        second_counter = 0;
        tracked_boxes.push_back(bbox);
        is_init = false;

        track_path_plotting_enabled = true; // settings["track_path_plotting_enabled"]
        track_prediction_enabled = true; // settings["track_prediction_enabled"]
        track_validation_enable = true; // settings["track_validation_enable"]
        track_stationary_threshold = 5; // settings["track_stationary_threshold"]
        track_orphaned_threshold = 20; // settings["track_orphaned_threshold"]
    }

    bool operator==(const Tracker &other) const
    {
        return this->id == other.id;
    }

    static cv::Ptr<cv::Tracker> Select(const std::map<std::string, std::string> &settings)
    {
        (void)settings;
        std::string tracker_type = "KCF";//settings["tracker_type"];

        // if (tracker_type == "BOOSTING")
        // {
        //     return cv::TrackerBoosting::create();
        // }
        // else 
        if (tracker_type == "MIL")
        {
            return cv::TrackerMIL::create();
        }
        else if (tracker_type == "KCF")
        {
            return cv::TrackerKCF::create();
        }
        // else if (tracker_type == "TLD")
        // {
        //     return cv::TrackerTLD::create();
        // }
        // else if (tracker_type == "MEDIANFLOW")
        // {
        //     return cv::TrackerMedianFlow::create();
        // }
        else if (tracker_type == "GOTURN")
        {
            return cv::TrackerGOTURN::create();
        }
        // else if (tracker_type == "MOSSE")
        // {
        //      return cv::TrackerMOSSE::create();
        // }
        else if (tracker_type == "DASIAMRPN")
        {
            return cv::TrackerDaSiamRPN::create();
        }

        cv::TrackerCSRT::Params params;
        params.use_gray = true;
        params.psr_threshold = 0.06;
        return cv::TrackerCSRT::create(params);
    }

    const cv::Rect& get_bbox() const
    {
        return bboxes.back();
    }

    cv::Point get_center() const
    {
        cv::Rect bbox = get_bbox();
        return cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    }

    std::pair<bool, cv::Rect> update(const cv::Mat &frame)
    {
        cv::Rect bbox;
        bool ok = cv2_tracker->update(frame, bbox);
        if (ok)
        {
            bboxes.push_back(bbox);

            if (track_path_plotting_enabled)
            {
                center_points.push_back(std::make_pair(get_center(), tracking_state));
            }

            if (track_prediction_enabled)
            {
                predictor_center_points.clear();
                predictor_center_points.push_back(track_predictor.update(bbox));
            }

            if (track_validation_enable)
            {
                bool validate_bbox = false;
                auto now = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed = now - start;
                if (std::floor(elapsed.count()) > second_counter)
                {
                    tracked_boxes.push_back(bbox);
                    second_counter++;
                    validate_bbox = true;
                }

                int stationary_scavanage_threshold = (int)std::floor((double)track_stationary_threshold * 1.5);

                if (tracked_boxes.size() > 1)
                {
                    if (bbox_overlap(bbox_to_check, bbox) == 0.0)
                    {
                        if (tracking_state != TrackingStateEnum::ActiveTarget)
                        {
                            tracking_state = TrackingStateEnum::ActiveTarget;
                            bbox_to_check = bbox;
                            stationary_track_counter = 0;
                        }
                    }

                    if (validate_bbox)
                    {
                        cv::Rect previous_tracked_bbox = tracked_boxes.back();
                        if (bbox_overlap(bbox_to_check, previous_tracked_bbox) > 0)
                        {
                            stationary_track_counter++;
                        }
                        else
                        {
                            stationary_track_counter = 0;
                        }
                    }
                }

                if (track_stationary_threshold <= stationary_track_counter && stationary_track_counter < stationary_scavanage_threshold)
                {
                    tracking_state = TrackingStateEnum::LostTarget;
                }
                else if (stationary_track_counter >= stationary_scavanage_threshold)
                {
                    ok = false;
                }

                if (tracking_state == TrackingStateEnum::ActiveTarget)
                {
                    active_track_counter++;
                    if (active_track_counter > track_orphaned_threshold)
                    {
                        bbox_to_check = bbox;
                        active_track_counter = 0;
                    }
                }
            }
        }
        return {ok, bbox};
    }

    bool is_tracking() const
    {
        return tracking_state == TrackingStateEnum::ActiveTarget;
    }

    static inline bool bbox_overlap(const cv::Rect &r1, const cv::Rect &r2)
    {
        if ((r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0) ||
            (r1.x > (r2.x + r2.width) || r2.x > (r1.x + r1.width)) ||
            (r1.y > (r2.y + r2.height) || r2.y > (r1.y + r1.height)))
        {
            return false;
        }

        return true;
    }

    static inline bool bbox1_contain_bbox2(const cv::Rect &bbox1, const cv::Rect &bbox2)
    {
        return (bbox2.x > bbox1.x) && (bbox2.y > bbox1.y) && (bbox2.x + bbox2.width < bbox1.x + bbox1.width) && (bbox2.y + bbox2.height < bbox1.y + bbox1.height);
    }

    bool does_bbox_overlap(const cv::Rect& bbox) const
    {
        return bbox_overlap(bboxes.back(), bbox) > 0;
    }

    bool is_bbox_contained(const cv::Rect& bbox) const
    {
        return bbox1_contain_bbox2(bboxes.back(), bbox);
    }

    int get_id() const
    {
        return id;
    }

    TrackingStateEnum get_tracking_state() const
    {
        return tracking_state; 
    }

    const std::vector<std::pair<cv::Point, TrackingStateEnum>>& get_center_points() const
    {
        return center_points;
    }

    const std::vector<cv::Point>& get_predictor_center_points() const
    {
        return predictor_center_points;
    }

private:
    std::map<std::string, std::string> settings;
    int id;
    cv::Ptr<cv::Tracker> cv2_tracker;
    std::vector<cv::Rect> bboxes;
    int stationary_track_counter;
    int active_track_counter;
    TrackingStateEnum tracking_state;
    cv::Rect2d bbox_to_check;
    std::chrono::system_clock::time_point start;
    int second_counter;
    std::vector<cv::Rect> tracked_boxes;
    std::vector<std::pair<cv::Point, TrackingStateEnum>> center_points;
    TrackPrediction track_predictor;
    std::vector<cv::Point> predictor_center_points;
    rclcpp::Logger logger_;
    bool is_init;

    bool track_path_plotting_enabled;
    bool track_prediction_enabled;
    bool track_validation_enable;
    int track_stationary_threshold;
    int track_orphaned_threshold;

};

#endif