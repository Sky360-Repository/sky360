#pragma once

#include <vector>
#include <opencv2/core.hpp>

namespace sky360lib::tracking
{
    using namespace cv;

    class TrackerTargetState
    {
    public:
        virtual ~TrackerTargetState(){};
        /** @brief Get the position
         * @return The position
         */
        Point2f getTargetPosition() const;

        /** @brief Set the position
         * @param position The position
         */
        void setTargetPosition(const Point2f &position);
        /** @brief Get the width of the target
         * @return The width of the target
         */
        int getTargetWidth() const;

        /** @brief Set the width of the target
         * @param width The width of the target
         */
        void setTargetWidth(int width);
        /** @brief Get the height of the target
         * @return The height of the target
         */
        int getTargetHeight() const;

        /** @brief Set the height of the target
         * @param height The height of the target
         */
        void setTargetHeight(int height);

    protected:
        Point2f targetPosition;
        int targetWidth;
        int targetHeight;
    };

    typedef std::vector<std::pair<Ptr<TrackerTargetState>, float>> ConfidenceMap;
    typedef std::vector<Ptr<TrackerTargetState>> Trajectory;

    class TrackerStateEstimator
    {
    public:
        virtual ~TrackerStateEstimator();

        /** @brief Estimate the most likely target state, return the estimated state
        @param confidenceMaps The overall appearance model as a list of :cConfidenceMap
        */
        Ptr<TrackerTargetState> estimate(const std::vector<ConfidenceMap> &confidenceMaps);

        /** @brief Update the ConfidenceMap with the scores
        @param confidenceMaps The overall appearance model as a list of :cConfidenceMap
        */
        void update(std::vector<ConfidenceMap> &confidenceMaps);

        /** @brief Create TrackerStateEstimator by tracker state estimator type
        @param trackeStateEstimatorType The TrackerStateEstimator name

        The modes available now:

        -   "BOOSTING" -- Boosting-based discriminative appearance models. See @cite AMVOT section 4.4

        The modes available soon:

        -   "SVM" -- SVM-based discriminative appearance models. See @cite AMVOT section 4.5
        */
        static Ptr<TrackerStateEstimator> create(const String &trackeStateEstimatorType);

        /** @brief Get the name of the specific TrackerStateEstimator
         */
        String getClassName() const;

    protected:
        virtual Ptr<TrackerTargetState> estimateImpl(const std::vector<ConfidenceMap> &confidenceMaps) = 0;
        virtual void updateImpl(std::vector<ConfidenceMap> &confidenceMaps) = 0;
        String className;
    };

    class TrackerModel
    {
    public:
        TrackerModel();

        virtual ~TrackerModel();

        /** @brief Set TrackerEstimator, return true if the tracker state estimator is added, false otherwise
        @param trackerStateEstimator The TrackerStateEstimator
        @note You can add only one TrackerStateEstimator
        */
        bool setTrackerStateEstimator(Ptr<TrackerStateEstimator> trackerStateEstimator);

        /** @brief Estimate the most likely target location

        @cite AAM ME, Model Estimation table I
        @param responses Features extracted from TrackerFeatureSet
        */
        void modelEstimation(const std::vector<Mat> &responses);

        /** @brief Update the model

        @cite AAM MU, Model Update table I
        */
        void modelUpdate();

        /** @brief Run the TrackerStateEstimator, return true if is possible to estimate a new state, false otherwise
         */
        bool runStateEstimator();

        /** @brief Set the current TrackerTargetState in the Trajectory
        @param lastTargetState The current TrackerTargetState
        */
        void setLastTargetState(const Ptr<TrackerTargetState> &lastTargetState);

        /** @brief Get the last TrackerTargetState from Trajectory
         */
        Ptr<TrackerTargetState> getLastTargetState() const;

        /** @brief Get the list of the ConfidenceMap
         */
        const std::vector<ConfidenceMap> &getConfidenceMaps() const;

        /** @brief Get the last ConfidenceMap for the current frame
         */
        const ConfidenceMap &getLastConfidenceMap() const;

        /** @brief Get the TrackerStateEstimator
         */
        Ptr<TrackerStateEstimator> getTrackerStateEstimator() const;

    private:
        void clearCurrentConfidenceMap();

    protected:
        std::vector<ConfidenceMap> confidenceMaps;
        Ptr<TrackerStateEstimator> stateEstimator;
        ConfidenceMap currentConfidenceMap;
        Trajectory trajectory;
        int maxCMLength;

        virtual void modelEstimationImpl(const std::vector<Mat> &responses) = 0;
        virtual void modelUpdateImpl() = 0;
    };
}
