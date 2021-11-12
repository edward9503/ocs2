#include "ocs2_raisim/RaisimRollout.h"
#include "ocs2_raisim/RaisimRolloutSettings.h"
#include <raisim/OgreVis.hpp>
#include <raisimBasicImguiPanel.hpp>
#include "helper.hpp"
#include <ocs2_msgs/raisim_PD_timeStampAndJoints.h>
#include <ros/topic.h>
#include "ros/ros.h"

#include <iostream>

ocs2_msgs::raisim_PD_timeStampAndJoints raisim_PD;

class PDJointReceiver
{
public:
    PDJointReceiver(){
        joints_sub = nm.subscribe("/legged_robot/PDJoints", 1, &PDJointReceiver::PDJointsCallback, this);
    }

    void PDJointsCallback(const ocs2_msgs::raisim_PD_timeStampAndJoints& msg)
    {
        //   ROS_INFO("I heard time: [%s]", msg.timeStamp);
        // std::cout << "I heard time: " << msg.timeStamp << "\n" << std::endl;
        //   ROS_INFO("I heard joints: [%s]", msg.joints);
        // std::cout << "I heard joints 1: " << msg.joints[0] << "\n" << std::endl;
        // std::cout << "I heard joints 2: " << msg.joints[1] << "\n" << std::endl;
        // std::cout << "I heard joints 3: " << msg.joints[2] << "\n" << std::endl;

        raisim_PD.timeStamp = msg.timeStamp;
        raisim_PD.joints = msg.joints;
    
    }

private:
    ros::NodeHandle nm;
    ros::Subscriber joints_sub;
};


void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(1., 1, -1);
  lightdir.normalise();
  vis->getLight()->setDirection(lightdir);
  vis->setContactVisObjectSize(0.04, 0.3);

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shadow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(10);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.01, 0.4);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}

int main(int argc, char **argv) {

    ROS_WARN("Make sure ocs2 is already runing.");
    ROS_WARN("Press Enter to continue...");
    std::cin.ignore();     

    ros::init(argc, argv, "raisim_PDOnly");
    PDJointReceiver PDJoints_obj;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    raisim::World::setActivationKey(raisim::loadResource("activation.raisim"));

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.0125);
    auto vis = raisim::OgreVis::get();
    world.setERP(0,0);

    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(1800, 1200);
    vis->setImguiSetupCallback(imguiSetupCallback);
    vis->setImguiRenderCallback(imguiRenderCallBack);
    vis->setSetUpCallback(setupCallback);
    vis->setAntiAliasing(8);
    raisim::gui::manualStepping = false;

    /// starts visualizer thread
    vis->initApp();

    /// create raisim objects
    auto ground = world.addGround();

    std::vector<raisim::ArticulatedSystem*> anymal;
    std::vector<std::vector<raisim::GraphicObject>*> anymal_visual;

    /// create visualizer objects
    vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");  

    /// create raisim objects
    anymal.push_back(world.addArticulatedSystem(raisim::loadResource("anymalC/anymal.urdf")));

    /// ANYmal joint PD controller
    Eigen::VectorXd jointNominalConfig(19), jointsPTarget(19), jointVelocityTarget(18);
    Eigen::VectorXd jointState(18), jointVel(18), jointForce(18), jointPgain(18), jointDgain(18);
    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointPgain.tail(12).setConstant(100.0);
    jointDgain.tail(12).setConstant(10.0);

    // std::cout << "I heard joints 1: " << raisim_PD.joints[0] << "\n" << std::endl;
    // std::cout << "I heard joints 2: " << raisim_PD.joints[1] << "\n" << std::endl;
    // std::cout << "I heard joints 3: " << raisim_PD.joints[2] << "\n" << std::endl;

    jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 
                          raisim_PD.joints[0], raisim_PD.joints[1], raisim_PD.joints[2], 
                          raisim_PD.joints[6], raisim_PD.joints[7], raisim_PD.joints[8], 
                          raisim_PD.joints[3], raisim_PD.joints[4], raisim_PD.joints[5], 
                          raisim_PD.joints[9], raisim_PD.joints[10], raisim_PD.joints[11];
    jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    anymal_visual.push_back(vis->createGraphicalObject(anymal.back(), "ANYmal"));  

    anymal.back()->setGeneralizedCoordinate({0, 0, 0.5, 1.0, 0.0, 0.0, 0.0, 
                                             raisim_PD.joints[0], raisim_PD.joints[1], raisim_PD.joints[2], 
                                             raisim_PD.joints[6], raisim_PD.joints[7], raisim_PD.joints[8], 
                                             raisim_PD.joints[3], raisim_PD.joints[4], raisim_PD.joints[5], 
                                             raisim_PD.joints[9], raisim_PD.joints[10], raisim_PD.joints[11]});
    Eigen::Vector4d quat; quat <<1., 0., 0., 0.;

    /// lambda function for the controller
    // double time=0.;
    // auto controller = [vis,
    //                     &time,
    //                     &world,
    //                     anymal]() {
    //     // time += world.getTimeStep();
    //     raisim::Vec<3> camera_position;
    //     raisim::Mat<3, 3> camera_orientation;
    //     Eigen::Matrix3f camera_oriEigen;

    //     anymal.back()->getFramePosition("face_front_to_wide_angle_camera_front_camera", camera_position);
    //     anymal.back()->getFrameOrientation("face_front_to_wide_angle_camera_front_camera", camera_orientation);
    //     for (int i=0; i<3; i++) {
    //     for (int j=0; j<3; j++) {
    //         camera_oriEigen(i,j) = camera_orientation(i,j);
    //     }
    //     }
    //     Eigen::Matrix3f preR_camera;
    //     preR_camera << 0, 0, -1,
    //                 -1, 0, 0,
    //                 0, 1, 0;
    //     Eigen::Quaternionf camera_quatEigen(camera_oriEigen*preR_camera);
    //     camera_quatEigen.normalize();
    //     vis->getCameraMan()->setQuatPos(Ogre::Real(camera_quatEigen.w()), Ogre::Real(camera_quatEigen.x()), 
    //                                             Ogre::Real(camera_quatEigen.y()), Ogre::Real(camera_quatEigen.z()), 
    //                                             Ogre::Vector3(camera_position(0), camera_position(1), camera_position(2)));
    //     std::cout<<"w: "<<camera_quatEigen.w()<<"x: "<<camera_quatEigen.x()<<"y: "<<camera_quatEigen.y()<<"z: "<<camera_quatEigen.z() << "\n";
    //     // Camera->setPosition(1,1,1);
    //     // mCamera->setOrientation(Ogre::Quaternion{1.,0.,0.,0.});
    //     // vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);
    //     // vis->getCameraMan()->setCamera(mCamera);
    //     // RSINFO(Camera->getPosition);
    //     // vis->getCameraMan()->setCamera(Camera);
    //     // RSINFO(camera_position);
    //     // RSINFO(camera_oriEigen);
    //     // RSINFO(camera_orientation);
    //     // RSINFO(anymal.back()-> getFrameIdxByName("face_front_to_depth_camera_front_camera"));
    // };

    // vis->setControlCallback(controller); 
    raisim::Vec<3> camera_position;
    anymal.back()->getFramePosition("face_front_to_wide_angle_camera_front_camera", camera_position); 

    vis->select(anymal_visual.back()->at(0));
    // vis->getCameraMan()->setYawPitchDist(Ogre::Radian(M_PI_2), -Ogre::Radian(0), 10);  
    vis->getCameraMan()->setQuatPos(Ogre::Real(0.5), Ogre::Real(0.5), Ogre::Real(-0.5), Ogre::Real(-0.5), 
                                            Ogre::Vector3(camera_position(0), camera_position(1), camera_position(2)));
    anymal.back()->setGeneralizedForce(Eigen::VectorXd::Zero(anymal.back()->getDOF()));
    anymal.back()->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    anymal.back()->setPdGains(jointPgain, jointDgain);
    anymal.back()->setPdTarget(jointNominalConfig, jointVelocityTarget);

    // Set the variable for the last time stamp
    ros::Time lastTimeStampt(raisim_PD.timeStamp);

    while (ros::ok()){
        ros::Time begin = ros::Time::now();
        std::cout << "time step: " << (raisim_PD.timeStamp - lastTimeStampt).toSec() << "\n" << std::endl;
        world.setTimeStep((raisim_PD.timeStamp - lastTimeStampt).toSec());
        lastTimeStampt = raisim_PD.timeStamp;


        // prepares all kinematic and dynamic quantities for the current time step
        world.integrate1();  

        jointsPTarget << 0, 0, 0, 0, 0, 0, 0, 
                         raisim_PD.joints[0], raisim_PD.joints[1], raisim_PD.joints[2], 
                         raisim_PD.joints[6], raisim_PD.joints[7], raisim_PD.joints[8], 
                         raisim_PD.joints[3], raisim_PD.joints[4], raisim_PD.joints[5], 
                         raisim_PD.joints[9], raisim_PD.joints[10], raisim_PD.joints[11];
        anymal.back()->setPdTarget(jointsPTarget, jointVelocityTarget);

        // actually move time foward and change the state
        world.integrate2();  

        vis->renderOneFrame();
        ros::Time end = ros::Time::now();

        // world.setTimeStep(0.0125);
        
        ros::spinOnce();
        
        std::cout << "time elapsed: " << (end - begin).toSec() << "\n" << std::endl;
    }
    return 0;
}