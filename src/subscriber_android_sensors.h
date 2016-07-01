#ifndef SUBSCRIBER_ANDROID_SENSORS_H
#define SUBSCRIBER_ANDROID_SENSORS_H

#include <string>
#include "sensor_msgs/Imu.h"

#include <fstream>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
//
#include <boost/archive/xml_oarchive.hpp>
//
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>


#define MACRO_XSTR(s) MACRO_STR(s)
#define MACRO_STR(s) #s

#define GET_ROS_PARAM( _param_name_, _param_member_, _default_value_, _priv_nh_, _nh_ )                             \
    if ( _priv_nh_.getParam ( MACRO_XSTR(_param_name_), _param_member_ ) )                                                \
{                                                                                                               \
    ROS_INFO_STREAM ( MACRO_XSTR(_param_name_) << " is: " << _param_member_);                                         \
    }                                                                                                               \
    else if( _nh_.getParam ( MACRO_XSTR(_param_name_), _param_member_ ) )                                                 \
{                                                                                                               \
    ROS_WARN_STREAM ("Non-private " << MACRO_XSTR(_param_name_) << " parameter is DEPRECATED: " << _param_member_);   \
    }                                                                                                               \
    else {                                                                                                          \
    _param_member_ = _default_value_;                                                                           \
    ROS_INFO_STREAM ( MACRO_XSTR(_param_name_) << " is: " << _param_member_ << "\t[default-value]");                  \
    }

// exemple : T = ::sensor_msgs::Imu
template<typename T>
class vector3_ros_serialize_type
{
    friend class boost::serialization::access;
    double x, y, z;

protected:
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(x) & \
                BOOST_SERIALIZATION_NVP(y) & \
                BOOST_SERIALIZATION_NVP(z);
    }

public:
    typedef boost::shared_ptr< T const> TConstPtr;

    vector3_ros_serialize_type(const TConstPtr & msg_) :
        x(msg_->orientation.x), y(msg_->orientation.y), z(msg_->orientation.z)
    {}
};

template<typename T>
class vector4_ros_serialize_type :
        public vector3_ros_serialize_type<T>
{
    friend class boost::serialization::access;

    double w;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        vector3_ros_serialize_type<T>::serialize(ar, version);
        ar & BOOST_SERIALIZATION_NVP(w);
    }

public:
    typedef boost::shared_ptr< T const> TConstPtr;

    vector4_ros_serialize_type(const TConstPtr & msg_) :
        vector3_ros_serialize_type<T>(msg_),
        w(msg_->orientation.w)
    {}
};

typedef vector3_ros_serialize_type<sensor_msgs::Imu> angular_velocity_imu_serialize_type;
typedef vector3_ros_serialize_type<sensor_msgs::Imu> linear_acceleration_imu_serialize_type;
typedef vector4_ros_serialize_type<sensor_msgs::Imu> orientation_imu_serialize_type;

class stamp_imu_serialize_type
{
    friend class boost::serialization::access;

    const boost::posix_time::ptime stamp;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        boost::serialization::save<Archive>(ar, stamp.time_of_day(), version);
    }

public:
    stamp_imu_serialize_type(const sensor_msgs::ImuConstPtr & msg_) :
         stamp(msg_->header.stamp.toBoost())
    {}
};

BOOST_CLASS_VERSION(vector3_ros_serialize_type<sensor_msgs::Imu>, 1)
BOOST_CLASS_VERSION(vector4_ros_serialize_type<sensor_msgs::Imu>, 1)
BOOST_CLASS_VERSION(stamp_imu_serialize_type, 1)

class _sensor_msgs_serialize_type
{
    friend class boost::serialization::access;
    //    friend std::ostream & operator<<(std::ostream &os, const _sensor_msgs_serialize_type &msg_);

    sensor_msgs::ImuConstPtr msg_;

    orientation_imu_serialize_type orientation;
    angular_velocity_imu_serialize_type angular_velocity;
    linear_acceleration_imu_serialize_type linear_velocity;
    stamp_imu_serialize_type stamp_imu;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // ------------------------
        // Informations temporelles
        // ------------------------
        ar << BOOST_SERIALIZATION_NVP(stamp_imu);

        // ------------------------
        // Informations IMU
        // ------------------------
        ar <<  BOOST_SERIALIZATION_NVP(orientation);
        ar <<  BOOST_SERIALIZATION_NVP(angular_velocity);
        ar <<  BOOST_SERIALIZATION_NVP(linear_velocity);
    }


public:
    // ------------------------
    // CONSTRUCTOR
    // ------------------------
    _sensor_msgs_serialize_type(const sensor_msgs::ImuConstPtr& _msg) :
        msg_(_msg)
      , orientation(_msg), angular_velocity(_msg), linear_velocity(_msg)
      , stamp_imu(_msg)
    {}

    // ------------------------
    // GETTERS
    // ------------------------
    inline const ros::Time&                 stamp() const { return msg_->header.stamp; }
    inline const sensor_msgs::ImuConstPtr&  msg()   const { return msg_; }
};

BOOST_CLASS_VERSION(_sensor_msgs_serialize_type, 1)


std::ostream & operator<<(std::ostream &os, const _sensor_msgs_serialize_type &msg_)
{
    // ostream of ROS msg
    return os << *(msg_.msg());
}

/**
 * @brief The SensorsToFile class
 */
class SensorsToFile {
protected:
    ros::NodeHandle nh_;

private:
    std::string prefix_;

    bool binary_;
    bool show_msg_;
    bool xml_;

    std::vector<_sensor_msgs_serialize_type> v_sensor_msgs_;

public:
    std::string sensors_topic_;

    ros::Subscriber sub_;

    inline std::string get_extension() const {
        // Si XML export => extension du fichier = "xml"
        // Sinon si export binaire => extension = "dat"
        // Sinon => export text => extension = "txt"
        return xml_ ? ".xml" : binary_ ? ".dat" : ".txt";
    }

    std::string build_filename() const
    {
        // On recupere le timestamp du message
        const ros::Time& stamp_begin = v_sensor_msgs_.front().stamp();
        const ros::Time& stamp_end = v_sensor_msgs_.back().stamp();
        // On construit le nom du fichier image par rapport a ce timestamp
        std::stringstream ss_sensors_filename;
        ss_sensors_filename << prefix_ <<                               \
                               stamp_begin << "_to_" << stamp_end <<    \
                               get_extension();
        return ss_sensors_filename.str();
    }

    template<typename T, class Archive>
    static
    void dump(const std::vector<T> & _vector, Archive & _ar)
    {
        // dump vector to archive
        for (typename std::vector<T>::const_iterator it = _vector.begin() ; it != _vector.end(); ++it)
            _ar << *it;
    }
    // Les fonctions ne peuvent pas etre partiellement specialisees (template) !
    // -> template<typename T> static void dump<T, boost::archive::xml_oarchive>(const std::vector<T> & _vector, boost::archive::xml_oarchive & _ar) { ... }
    //
    inline void dump(boost::archive::xml_oarchive & _ar) const {
        _ar & BOOST_SERIALIZATION_NVP(v_sensor_msgs_);
    }

    inline void dump(boost::archive::binary_oarchive & _ar) const {
        dump<_sensor_msgs_serialize_type, boost::archive::binary_oarchive>(v_sensor_msgs_, _ar);
    }

    inline void dump(boost::archive::text_oarchive & _ar) const {
        dump<_sensor_msgs_serialize_type, boost::archive::text_oarchive>(v_sensor_msgs_, _ar);
    }


    bool dump(const std::string & _filename)
    {
        bool retour = true;

        std::ofstream ofs(_filename, std::ios::out | std::ofstream::binary);
        if(ofs.is_open()) {
            // save data to archive
            try {
                if(xml_) {
                    boost::archive::xml_oarchive oa(ofs);
                    dump(oa);
                }
                else {
                    if(binary_) {
                        boost::archive::binary_oarchive oa(ofs);
                        // write vector to archive
                        dump(oa);
                        // archive and stream closed when destructors are called
                    }
                    else {
                        boost::archive::text_oarchive oa(ofs);
                        // write vector to archive
                        dump(oa);
                        // archive and stream closed when destructors are called
                    }
                }
            }
            catch(std::exception &exc){
                ROS_ERROR_STREAM("Exception lors de la serialization ! -> " << exc.what());
                retour = false;
            }

            ROS_INFO_STREAM("Write sensors data into: " << _filename);
        }
        else {
            ROS_ERROR_STREAM("Probleme d'ouverture du stream: " << _filename);
            retour = false;
        }

        return retour;
    }

    void sensors_cb(const sensor_msgs::ImuConstPtr& _msg)
    {
        if (show_msg_)
            ROS_INFO_STREAM("Orientation\n" << _msg->orientation);

        // Sauvegarde en memoire du message (les informations qui nous interessent du message)
        _sensor_msgs_serialize_type sensor_msg(_msg);
        v_sensor_msgs_.push_back(sensor_msg);
//        ROS_INFO_STREAM("sensor_msg : " << sensor_msg);

        // Est ce qu'on a atteint la capacite max. du vecteur ?
        if (v_sensor_msgs_.size() == v_sensor_msgs_.capacity())
        {
            // Si oui, on enregistre (sur fichier) le vecteur
            dump(build_filename());
            // et on le vide
            v_sensor_msgs_.clear();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    SensorsToFile (ros::NodeHandle _priv_nh = ros::NodeHandle("~"),
                   const uint32_t& _size_buffer=50 )
    {
        v_sensor_msgs_.reserve(_size_buffer);

        // On recupere les parametres ROS
        GET_ROS_PARAM(prefix, prefix_, "SENSORS_", _priv_nh, nh_);
        GET_ROS_PARAM(binary, binary_, true, _priv_nh, nh_);
        GET_ROS_PARAM(xml, xml_, false, _priv_nh, nh_);
        GET_ROS_PARAM(sensors_topic, sensors_topic_, "/android/imu", _priv_nh, nh_);
        GET_ROS_PARAM(show_msg, show_msg_, false, _priv_nh, nh_);

        // On lance le subscriber
        sub_ = nh_.subscribe(sensors_topic_, 1, &SensorsToFile::sensors_cb, this);

        ROS_INFO ("Listening for incoming data on topic %s",
                  nh_.resolveName (sensors_topic_).c_str ());
    }
};

#endif // SUBSCRIBER_ANDROID_SENSORS_H
