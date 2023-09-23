#pragma once

#include "mrosuam/gui/AtomicVariable.h"

#include <ros/ros.h>

#include <chrono>

namespace mrosuam::gui
{
    
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds>;
    
    template <class MsgType>
    class ConnectionStatus
    {
    public:
        ConnectionStatus(const std::string& topicName, float timeoutDurationSeconds) : m_topicName(topicName), m_timeoutDurationSeconds(timeoutDurationSeconds) {}
        
        void initialize(ros::NodeHandle& nodeHandle)
        {
            subscriber = nodeHandle.subscribe<MsgType>(m_topicName, 10, &ConnectionStatus<MsgType>::callback, this);
        }
        
        bool checkStatus()
        {
            const TimePoint timeNow = std::chrono::high_resolution_clock::now();
            bool status = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - m_lastMessageTime.get()).count() / 1.0e3f < m_timeoutDurationSeconds;
            return status;
        }
        
        void callback(const typename MsgType::ConstPtr& data)
        {
            m_lastMessageTime = std::chrono::high_resolution_clock::now();
        }
        
    private:
        
        std::string m_topicName;
        
        ros::Subscriber subscriber;
        
        float m_timeoutDurationSeconds;
        
        AtomicVariable<TimePoint> m_lastMessageTime;
        
    };
    
}
