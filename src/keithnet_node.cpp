/**
 * @file keithnet_node.cpp
 * @brief Assumes a UESMANN net has been trained, with
 * NUMSONARS (3) inputs, HNODES hidden layers and 2 outputs.
 * The inputs are sonars, the outputs are left and right motors.
 * 
 * Reads the network from the file in ~netfile. No topology checks
 * are done to make sure the read network is the same!
 *
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "backpropNoBiasHormone.h"

#define HNODES 8
#define NUM_SONARS 3

static float sonars[NUM_SONARS];
void sonarCallback(const std_msgs::Float32MultiArray::ConstPtr &msg){
    for(int i=0;i<NUM_SONARS;i++){
        ROS_INFO("Sonar received");
        sonars[i] = msg->data[i];
    }
}

// convert NN output (0=reverse,1=forward) to motor speed
inline float net2motor(float f){
    f -= 0.5f; // recentre
    f *= 15.0f; // scale for speed
    return f;
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"keithnet_node");
    ros::NodeHandle n;
    
    // set up the network - we assume a certain geometry
    // which must match up!
    
    // 3 inputs, some hidden nodes, 2 outputs
    int layers[]={NUM_SONARS,8,2};
    BackpropNetNoBiasHormone net(1); // eta is irrelevant
    net.init(3,layers);
    
    // try to read the genome file
    std::string file;
    if(!ros::param::get("~netfile",file)){
        ROS_ERROR("~netfile not specified");
        exit(0);
    }
    
    FILE *a;
    if(!(a=fopen(file.c_str(),"rb"))){
        ROS_ERROR("cannot open netfile %s",file.c_str());
        exit(0);
    }
    
    int gs = net.getGenomeSize();
    double *qq = new double[gs];
    size_t r = fread(qq,sizeof(double),gs,a);
    fclose(a);
    if(r!=gs){
        ROS_ERROR("Netfile %s too short: %d wanted, %ld read",file.c_str(),gs,r);
        exit(0);
    }
    net.setFromGenome(qq);
    delete [] qq;
    
    
    // subscribe to sonar topics
    ros::Subscriber sonarSub =
          n.subscribe<std_msgs::Float32MultiArray>("sonar",100,sonarCallback);
    
    // publish to motor topics
    ros::Publisher leftMotorPub =
          n.advertise<std_msgs::Float32>("leftmotors",100);
    ros::Publisher rightMotorPub =
          n.advertise<std_msgs::Float32>("rightmotors",100);
    
    
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        
        // feed the network
        double ins[NUM_SONARS];
        for(int i=0;i<NUM_SONARS;i++)
            ins[i]=sonars[i];
        
        net.setH(0);
        net.setInputs(ins);
        net.update();
        
        // update the robot
        double *outs = net.getOutputs();
        std_msgs::Float32 msg;
        msg.data=net2motor(outs[0]);
        leftMotorPub.publish(msg);
        msg.data=net2motor(outs[1]);
        rightMotorPub.publish(msg);
        
        rate.sleep();
    }
    
    return 0;
}
