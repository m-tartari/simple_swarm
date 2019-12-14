/**
 * \file swarm_node
 * \brief The node implements the behavior of a swarm member
 * \author Michele Tartari
 * Based on an origina file made by Gaetan Garcia.
 * 
 * \param[in] /world_size, global parameter. Positive real. World 
 *               is a square with coordinates between 0 and world_size.
 * \param[in] Kp, private parameter. Positive real. Proportional gain
 *               of the swarm member control, defaults to 0.1.
 *               Should be less than and not close to 1.0.
 * 
 * Subscribes to: <BR>
 *    ° "virtual_leader_pos", group topic name, type geometry_msgs::Pos,
 *         position of the virtual leader. <BR>
 * 
 * Publishes to: <BR>
 *    ° "pos", local topic name, type geometry_msgs::Point, position
 *         of this swarm member. <BR>
 *    ° "/visualization_marker", absolute topic name, visualization_msgs::Marker,
 *         to publish to rviz a marker at the position of the swarm member. <BR>
 *
 * The behavior of the swarm member is to get to a random point close 
 * to the virtual leader.
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// You may have a number of globals here.

geometry_msgs::Point virtualLeader, previousLeaderPos ;

// Callback functions...

geometry_msgs::Point goal_pos_msg ;
bool virtualLeaderInfoReceived     = false ;
void virtualLeaderCallback(geometry_msgs::Point goal_pos_msg){
    // Copy goal information into global variables
    virtualLeader.x = goal_pos_msg.x ;
    virtualLeader.y = goal_pos_msg.y ;
    virtualLeader.z = goal_pos_msg.z ;
    virtualLeaderInfoReceived = true ;
}

ros::Publisher pubMarker ;
visualization_msgs::Marker marker;

void initializeMarker( ){
    // Fetch node name. Markers will be blue if the word "blue" is in the name, red otherwise.
    std::string nodeName ;
    nodeName = ros::this_node::getName();
    // Create a marker for this node. Only timestamp and position will be later updated.
    marker.header.frame_id = "/map";
    marker.ns = nodeName;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    if( nodeName.find("blue") != std::string::npos ){
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5;
    }else{
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5;
    }
}

void publishMarkerAt( geometry_msgs::Point markerPos) {    
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = markerPos.x ;
    marker.pose.position.y = markerPos.y ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}

#define DIST  0.05
#define FREQ 10.0

int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "swarm_node");
   
// ------------- 
// Your modifications should be within the two commented dashed lines.

    // Define your node handles
    ros::NodeHandle nh_loc("~"), nh_glob ;

    // A global parameter can be read with a local namespace node handler.
    double worldSize ;
    nh_glob.param("/world_size", worldSize, 1.0  );   
    double Kp ;
    nh_loc.param("Kp",Kp,0.1); 

    // What the node subscribes to.
    ros::Subscriber subToGoal = 
        nh_glob.subscribe<geometry_msgs::Point>("leader_pos",1,virtualLeaderCallback);

    // What the node publishes to.
    ros::Publisher pubPos = nh_loc.advertise<geometry_msgs::Point>("pos",1);
    
// Your modifications should not go beyond this line.
// -------------
    
    pubMarker = nh_loc.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

    // Random initial position swarm member.
    geometry_msgs::Point pos ;
    timeval t1;
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec * t1.tv_sec);
    pos.x = (1.0*rand())/RAND_MAX*worldSize ;  pos.y = (1.0*rand())/RAND_MAX*worldSize ; pos.z = 0.0 ; 
    publishMarkerAt( pos ) ; 
    
    initializeMarker() ;
    previousLeaderPos.x = -1.0 ;  previousLeaderPos.y = -1.0 ;  previousLeaderPos.z = -1.0 ;
    geometry_msgs::Point goal = pos ;  // Birds won't move
        
    ros::Rate rate(FREQ);
    while (ros::ok()){

        ros::spinOnce();

        if( virtualLeaderInfoReceived ){
            if( (virtualLeader.x!=previousLeaderPos.x) || (virtualLeader.y!=previousLeaderPos.y) ){
                // Set a goal close to virtual leader position 
                goal.x = virtualLeader.x + ((1.0*rand())/RAND_MAX-0.5)*0.3*worldSize ;
                goal.y = virtualLeader.y + ((1.0*rand())/RAND_MAX-0.5)*0.3*worldSize ;
                goal.z = 0.0 ;
                virtualLeaderInfoReceived = false ;
                previousLeaderPos = virtualLeader ;
            }
        }

        // Control swarm member.
        pos.x += Kp*(goal.x-pos.x) ;
        pos.y += Kp*(goal.y-pos.y) ;

        pubPos.publish(pos) ;

        // Publish a marker to visualize under Rviz
        publishMarkerAt( pos ) ;
            
        rate.sleep();
    }
}
