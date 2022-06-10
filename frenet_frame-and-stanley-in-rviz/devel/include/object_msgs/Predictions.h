// Generated by gencpp from file object_msgs/Predictions.msg
// DO NOT EDIT!


#ifndef OBJECT_MSGS_MESSAGE_PREDICTIONS_H
#define OBJECT_MSGS_MESSAGE_PREDICTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <object_msgs/Prediction.h>

namespace object_msgs
{
template <class ContainerAllocator>
struct Predictions_
{
  typedef Predictions_<ContainerAllocator> Type;

  Predictions_()
    : id(0)
    , n_prediction(0)
    , predictions()
    , prob()  {
    }
  Predictions_(const ContainerAllocator& _alloc)
    : id(0)
    , n_prediction(0)
    , predictions(_alloc)
    , prob(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef int32_t _n_prediction_type;
  _n_prediction_type n_prediction;

   typedef std::vector< ::object_msgs::Prediction_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::object_msgs::Prediction_<ContainerAllocator> >::other >  _predictions_type;
  _predictions_type predictions;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _prob_type;
  _prob_type prob;





  typedef boost::shared_ptr< ::object_msgs::Predictions_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::object_msgs::Predictions_<ContainerAllocator> const> ConstPtr;

}; // struct Predictions_

typedef ::object_msgs::Predictions_<std::allocator<void> > Predictions;

typedef boost::shared_ptr< ::object_msgs::Predictions > PredictionsPtr;
typedef boost::shared_ptr< ::object_msgs::Predictions const> PredictionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::object_msgs::Predictions_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::object_msgs::Predictions_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::object_msgs::Predictions_<ContainerAllocator1> & lhs, const ::object_msgs::Predictions_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.n_prediction == rhs.n_prediction &&
    lhs.predictions == rhs.predictions &&
    lhs.prob == rhs.prob;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::object_msgs::Predictions_<ContainerAllocator1> & lhs, const ::object_msgs::Predictions_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace object_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::object_msgs::Predictions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::object_msgs::Predictions_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::object_msgs::Predictions_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::object_msgs::Predictions_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::object_msgs::Predictions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::object_msgs::Predictions_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::object_msgs::Predictions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f8937a5011813d4b881807b890c93552";
  }

  static const char* value(const ::object_msgs::Predictions_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf8937a5011813d4bULL;
  static const uint64_t static_value2 = 0x881807b890c93552ULL;
};

template<class ContainerAllocator>
struct DataType< ::object_msgs::Predictions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "object_msgs/Predictions";
  }

  static const char* value(const ::object_msgs::Predictions_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::object_msgs::Predictions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n"
"int32 n_prediction\n"
"object_msgs/Prediction[] predictions\n"
"float32[] prob\n"
"\n"
"================================================================================\n"
"MSG: object_msgs/Prediction\n"
"std_msgs/Header header\n"
"uint32 id\n"
"\n"
"uint32 n_predictions\n"
"float32 dt  # s\n"
"object_msgs/Object[] predictions\n"
"\n"
"float32[] sigx\n"
"float32[] sigy\n"
"float32[] rho\n"
"time t0  # time corresponding prediction[0]\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: object_msgs/Object\n"
"std_msgs/Header header\n"
"uint32 id\n"
"\n"
"# The type of classification given to this object.\n"
"uint8 classification\n"
"uint8 CLASSIFICATION_UNKNOWN=0\n"
"uint8 CLASSIFICATION_CAR=1\n"
"uint8 CLASSIFICATION_PEDESTRIAN=2\n"
"uint8 CLASSIFICATION_CYCLIST=3\n"
"\n"
"# The detected position and orientation of the object.\n"
"float32 x       # m\n"
"float32 y       # m\n"
"float32 yaw     # rad\n"
"\n"
"float32 v       # m/s\n"
"float32 yawrate # rad/s\n"
"\n"
"float32 a      # m/ss\n"
"float32 delta  # radian\n"
"\n"
"# size\n"
"float32 L     # m\n"
"float32 W     # m\n"
;
  }

  static const char* value(const ::object_msgs::Predictions_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::object_msgs::Predictions_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.n_prediction);
      stream.next(m.predictions);
      stream.next(m.prob);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Predictions_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::object_msgs::Predictions_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::object_msgs::Predictions_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "n_prediction: ";
    Printer<int32_t>::stream(s, indent + "  ", v.n_prediction);
    s << indent << "predictions[]" << std::endl;
    for (size_t i = 0; i < v.predictions.size(); ++i)
    {
      s << indent << "  predictions[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::object_msgs::Prediction_<ContainerAllocator> >::stream(s, indent + "    ", v.predictions[i]);
    }
    s << indent << "prob[]" << std::endl;
    for (size_t i = 0; i < v.prob.size(); ++i)
    {
      s << indent << "  prob[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.prob[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBJECT_MSGS_MESSAGE_PREDICTIONS_H
