// urls:
// - http://www.boost.org/doc/libs/1_61_0/libs/serialization/doc/tutorial.html
#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Imu.h"

#include <queue>
#include <algorithm>
#include <iostream>
#include <fstream>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

// TODO: rajouter l'option binaire/text pour l'enregistrement
class SensorsToFile {
protected:
    ros::NodeHandle nh_;

private:
    std::string prefix_;

    struct _orientation_stamp_type
    {
        _orientation_stamp_type(const sensor_msgs::ImuConstPtr& _msg) :
            _stamp(ros::Time::now()), _orientation(_msg->orientation)
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
            ar << _stamp.toSec() & _stamp.toNSec();
            ar << _orientation.x & _orientation.y & _orientation.z & _orientation.w;
        }
    };

    std::vector<_orientation_stamp_type> v_orientations;

public:
    std::string sensors_topic_;

    ros::Subscriber sub_;

    void sensors_cb(const sensor_msgs::ImuConstPtr& _msg)
    {
//        ROS_INFO_STREAM("Orientation\n" << _msg->orientation);
        v_orientations.push_back(_msg);
        if (v_orientations.size() == v_orientations.capacity())
        {
            // TODO: faire une méthode (séparée) pour écrire le vector
            // On récupère le timestamp du message
            const ros::Time& stamp_begin = v_orientations.at(0)._stamp;
            const ros::Time& stamp_end = v_orientations.back()._stamp;
            // On construit le nom du fichier image par rapport à ce timestamp
            std::stringstream ss_sensors_filename;
            ss_sensors_filename << prefix_ << \
                                   stamp_begin << "_to_" << stamp_end << \
                                   ".dat";
            std::ofstream ofs(ss_sensors_filename.str(), std::ios::out | std::ofstream::binary);
            // save data to archive
                {
//                    boost::archive::text_oarchive oa(ofs);
                boost::archive::binary_oarchive oa(ofs);
                    // write class instance to archive
                    for (std::vector<_orientation_stamp_type>::iterator it = v_orientations.begin() ; it != v_orientations.end(); ++it)
                        oa << *it;
                    // archive and stream closed when destructors are called
                }
            ROS_INFO_STREAM("Write sensors data into: " << ss_sensors_filename.str());
            //
            v_orientations.clear();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    SensorsToFile (ros::NodeHandle _priv_nh = ros::NodeHandle("~"),
                   const uint32_t& _size_buffer=50 )
    {
        v_orientations.reserve(_size_buffer);

        // Check if a prefix parameter is defined for output file names.
        if (_priv_nh.getParam ("prefix", prefix_))
        {
            ROS_INFO_STREAM (" file prefix is: " << prefix_);
        }
        else if (nh_.getParam ("prefix", prefix_))
        {
            ROS_WARN_STREAM ("Non-private prefix parameter is DEPRECATED: "
                             << prefix_);
        }
        else {
            prefix_ = "SENSORS_";
        }

        // TODO: permettre de spécifier en option/argument le topic à écouter
        sensors_topic_ = "/android/imu";


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
