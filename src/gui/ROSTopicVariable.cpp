#include "mrosuam/gui/ROSTopicVariable.h"

namespace mrosuam::gui
{
    template <class Type, class MsgType>
    void ROSTopicVariable<Type, MsgType>::initialize(const ros::NodeHandle& nodeHandle, const std::string& topic, uint32_t queueSize, Type (*cb) (typename MsgType::ConstPtr))
    {
        
    }
}
