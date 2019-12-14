/**
 * \file swarm_behavior.cpp
 * \brief A node to randomly move the virtual leader of a swarm
 * \author Michele Tartari
 * Based on an origina file made by Gaetan Garcia.
 * 
 * \param[in] ...
 * 
 * Subscribes to: <BR>
 * 
 * Publishes to: <BR>
 *
 *    The swarm behavior randomly sets the position of the virtual leader
 *  of the swarm. No particular individual of the swarm is the leader, 
 *  instead the swarm moves always close to the virtual leader set by this 
 *  node.
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// By default, the key that makes the swarm move it the space bar.
#define DEFAULT_MOVE_KEY 32

// Callback functions to handle keystroke messages.

bool keyAvailable = false ;
int  lastKey ; 
void keystrokeCallback(std_msgs::Int16 key_msg){
    lastKey      = key_msg.data ;
    keyAvailable = true         ;
}

ros::Publisher pubMarker ;
visualization_msgs::Marker marker;


// Our marker message has many fields that remain constant and
// are initialized only once by the following function.

void initializeMarker(){
    // Fetch node name. Markers will be blue if the word "blue" is in the name, red otherwise.
    std::string nodeName ;
    nodeName = ros::this_node::getName();
    // Create a marker for this node. Only timestamp and position will be later updated.
    marker.header.frame_id = "/map";
    marker.ns = nodeName;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    if( nodeName.find("blue") != std::string::npos ){
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
    }else{
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
    }
}


// Function to publish a marke at a given (x,y) position.

void publishMarkerAt( geometry_msgs::Point markerPos) {    
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = markerPos.x ;
    marker.pose.position.y = markerPos.y ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}


int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "swarm_behavior");
    
// ------------- 
// Your modifications should be within the two commented dashed lines.    

    // Node handles
    ros::NodeHandle nh, nh_loc("~") ;

    // Read the node parameters.
    int moveKey ;
    double worldSize ;
    nh.param("/world_size", worldSize, 1.0  );   // A global parameter. Others are local.
    nh_loc.param("move_key"  , moveKey , DEFAULT_MOVE_KEY);
    ROS_INFO("move_key: %d",moveKey) ;
    double step = worldSize/10.0 ;

    // Topics to which the node subscribes.
    ros::Subscriber subKeystrokes = 
        nh.subscribe<std_msgs::Int16>("/key_typed",1,keystrokeCallback);

    // Topics to which the node publishes.
    ros::Publisher pubGoal = nh.advertise<geometry_msgs::Point>("leader_pos",1);
    
// Your modifications should not go beyond this line.
// -------------
    
    pubMarker = nh_loc.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

    // Set a random initial virtual leader. Will then be randomly reset on demand.
    geometry_msgs::Point virtualLeader ;
    timeval t1;
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec * t1.tv_sec);
    virtualLeader.x = (1.0*rand()/RAND_MAX)*worldSize ;
    virtualLeader.y = (1.0*rand()/RAND_MAX)*worldSize ;
    virtualLeader.z = 0.0 ;
    
    initializeMarker() ;

    ros::Rate rate(100);
    while (ros::ok()){

        ros::spinOnce();

        // If no keystroke has been received, do nothing.
        if( keyAvailable ){ 
            keyAvailable = false ;
            if( lastKey == moveKey ){
                // Move the swarm.
                virtualLeader.x = (1.0*rand()/RAND_MAX)*worldSize ;
                virtualLeader.y = (1.0*rand()/RAND_MAX)*worldSize ;
                virtualLeader.z = 0.0 ; 
            }
        }
        pubGoal.publish(virtualLeader) ;
        publishMarkerAt(virtualLeader) ;

        rate.sleep();
    }
}
