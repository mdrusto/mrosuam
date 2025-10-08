#pragma once

#include "mrosuam/gui/AtomicVariable.h"

#include <rclcpp/rclcpp.hpp>


namespace mrosuam::gui
{
    
    template <class Type, class MsgType>
    class ROSTopicVariable
    {
    public:
        
        ROSTopicVariable(std::function<Type(typename MsgType::ConstPtr)> cb, const Type& initValue = Type()) : customCallback(cb), atomicVar(initValue)  {}
        
        void initialize(const rclcpp::Subscriber<MsgType>::SharedPtr sub)
        {
            
            auto callback = 
                [this] (const boost::shared_ptr<const MsgType>& msg) -> void
                {
                    atomicVar = customCallback(msg);
                };
            
            subscriber = sub;
        }
        
        Type get() { return atomicVar.get(); }
        
    private:
        
        AtomicVariable<Type> atomicVar;
        rclcpp::Subscriber<MsgType>::SharedPtr subscriber;
        std::function<Type(typename MsgType::ConstPtr)> customCallback;
    };
    
}
