// urls:
// - http://www.boost.org/doc/libs/1_61_0/libs/serialization/doc/tutorial.html
#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Imu.h"

//#include <iostream>
#include <fstream>
//#include <ostream>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>


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


    struct _orientation_stamp_type
    {
        _orientation_stamp_type(const sensor_msgs::ImuConstPtr& _msg) :
            _stamp(_msg->header.stamp), _orientation(_msg->orientation)
        {}

        ros::Time _stamp;
        sensor_msgs::Imu::_orientation_type _orientation;

        friend class boost::serialization::access;
        // When the class Archive corresponds to an output archive, the
        // & operator is defined similar to <<.  Likewise, when the class Archive
        // is a type of input archive the & operator is defined similar to >>.
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            //            ar & _stamp.toSec() & _stamp.toNSec();
            const double _stamp_sec = _stamp.toSec();
            const double _stamp_nsec = _stamp.toNSec();
            ar << _stamp_sec & _stamp_nsec;
            ar << _orientation.x & _orientation.y & _orientation.z & _orientation.w;
        }
    };

    std::vector<_orientation_stamp_type> v_orientations_;

public:
    std::string sensors_topic_;

    ros::Subscriber sub_;

    std::string build_filename() const
    {
        // On recupere le timestamp du message
        const ros::Time& stamp_begin = v_orientations_.front()._stamp;
        const ros::Time& stamp_end = v_orientations_.back()._stamp;
        // On construit le nom du fichier image par rapport a ce timestamp
        std::stringstream ss_sensors_filename;
        ss_sensors_filename << prefix_ <<                               \
                               stamp_begin << "_to_" << stamp_end <<    \
                               ".dat";
        return ss_sensors_filename.str();
    }

    template<typename T, class Archive>
    static
    void write_vector_on_archive(const std::vector<T> & _vector, Archive & _ar)
    {
        // write vector to archive
        for (typename std::vector<T>::const_iterator it = _vector.begin() ; it != _vector.end(); ++it)
            _ar << *it;
    }

    inline void write_on_archive(boost::archive::binary_oarchive & _ar) const {
        write_vector_on_archive<_orientation_stamp_type, boost::archive::binary_oarchive>(v_orientations_, _ar);
    }

    inline void write_on_archive(boost::archive::text_oarchive & _ar) const {
        write_vector_on_archive<_orientation_stamp_type, boost::archive::text_oarchive>(v_orientations_, _ar);
    }

    bool write_orientations(const sensor_msgs::ImuConstPtr& _msg)
    {
        bool retour = true;

        const std::string filename = build_filename();

        std::ofstream ofs(filename, std::ios::out | std::ofstream::binary);
        if(ofs.is_open()) {
            // save data to archive
            try {
                if(binary_) {
                    boost::archive::binary_oarchive oa(ofs);
                    // write vector to archive
                    write_on_archive(oa);
                    // archive and stream closed when destructors are called
                }
                else {
                    boost::archive::text_oarchive oa(ofs);
                    // write vector to archive
                    write_on_archive(oa);
                    // archive and stream closed when destructors are called
                }
            }
            catch(std::exception &exc){
                ROS_ERROR_STREAM("Exception lors de la serialization ! -> " << exc.what());
                retour = false;
            }

            ROS_INFO_STREAM("Write sensors data into: " << filename);
        }
        else {
            ROS_ERROR_STREAM("Probleme d'ouverture du stream: " << filename);
            retour = false;
        }

        return retour;
    }

    void sensors_cb(const sensor_msgs::ImuConstPtr& _msg)
    {
        if (show_msg_)
            ROS_INFO_STREAM("Orientation\n" << _msg->orientation);

        // Sauvegarde en memoire du message (les informations qui nous interessent du message)
        v_orientations_.push_back(_msg);

        // Est ce qu'on a atteint la capacite max. du vecteur ?
        if (v_orientations_.size() == v_orientations_.capacity())
        {
            // Si oui, on enregistre (sur fichier) le vecteur
            write_orientations(_msg);
            // et on le vide
            v_orientations_.clear();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    SensorsToFile (ros::NodeHandle _priv_nh = ros::NodeHandle("~"),
                   const uint32_t& _size_buffer=50 )
    {
        v_orientations_.reserve(_size_buffer);

        // On recupere les parametres ROS
        GET_ROS_PARAM(prefix, prefix_, "SENSORS_", _priv_nh, nh_);
        GET_ROS_PARAM(binary, binary_, true, _priv_nh, nh_);
        GET_ROS_PARAM(sensors_topic, sensors_topic_, "/android/imu", _priv_nh, nh_);
        GET_ROS_PARAM(show_msg, show_msg_, false, _priv_nh, nh_);

        // On lance le subscriber
        sub_ = nh_.subscribe(sensors_topic_, 1, &SensorsToFile::sensors_cb, this);

        ROS_INFO ("Listening for incoming data on topic %s",
                  nh_.resolveName (sensors_topic_).c_str ());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");

    ros::NodeHandle priv_nh("~");

    SensorsToFile stof(priv_nh);

    ros::spin();
}
