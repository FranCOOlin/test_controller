int main(int argc, char** argv) {
    // ros::init(argc, argv, "simu");
    // ros::NodeHandle nh;

    // ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    // ros::Rate loop_rate(100);

    // UAVState uav_state;
    // uav_state.position << 0, 0, 0;
    // uav_state.velocity << 0, 0, 0;
    // uav_state.orientation << 0, 0, 0;
    // uav_state.angular_velocity << 0, 0, 0;

    // while (ros::ok()) {
    //     uav_state.position += uav_state.velocity * 0.01;
    //     uav_state.orientation += uav_state.angular_velocity * 0.01;

    //     nav_msgs::Odometry odom;
    //     odom.header.stamp = ros::Time::now();
    //     odom.header.frame_id = "world";
    //     odom.pose.pose.position.x = uav_state.position(0);
    //     odom.pose.pose.position.y = uav_state.position(1);
    //     odom.pose.pose.position.z = uav_state.position(2);
    //     odom.twist.twist.linear.x = uav_state.velocity(0);
    //     odom.twist.twist.linear.y = uav_state.velocity(1);
    //     odom.twist.twist.linear.z = uav_state.velocity(2);
    //     odom.pose.pose.orientation.x = uav_state.orientation(0);
    //     odom.pose.pose.orientation.y = uav_state.orientation(1);
    //     odom.pose.pose.orientation.z = uav_state.orientation(2);
    //     pub.publish(odom);

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}