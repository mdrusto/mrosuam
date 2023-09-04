#pragma once

#include "mrosuam/gui/AtomicVariable.h"

#include <ros/ros.h>


namespace mrosuam::gui
{
    
    template <class Type, class MsgType>
    class ROSTopicVariable
    {
    public:
        
        ROSTopicVariable(std::function<Type(typename MsgType::ConstPtr)> cb, const Type& initValue = Type()) : customCallback(cb), atomicVar(initValue)  {}
        
        void initialize(ros::NodeHandle& nodeHandle, const std::string& topic, uint32_t queueSize)
        {
            
            auto callback = 
                [this] (const boost::shared_ptr<const MsgType>& msg) -> void
                {
                    atomicVar = customCallback(msg);
                };
            
            subscriber = nodeHandle.subscribe<MsgType>(topic, queueSize, callback);
        }
        
        Type get() { return atomicVar.get(); }
        
    private:
        
        AtomicVariable<Type> atomicVar;
        ros::Subscriber subscriber;
        std::function<Type(typename MsgType::ConstPtr)> customCallback;
    };
    
}
