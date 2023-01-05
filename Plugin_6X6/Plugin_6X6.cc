#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <cmath>


namespace gazebo
{
    class Plugin_6X6 : public ModelPlugin  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if(_model->GetJointCount()==0){
                std::cerr<<"Invalid joint count, Plugin not loaded\n";
                return;
            }else{
                std::cout<<"Load the Model Successed!"<<std::endl;
            }

            this->model=_model;
            this->world=model->GetWorld();

            //Link
            this->Link_chassis=model->GetLink("link_0");
            physics::LinkState link_state(this->Link_chassis);
            this->Link_chassis_State=link_state;
            this->Link_chassis_Pose=this->Link_chassis_State.Pose();
            this->Link_chassis_Vel=this->Link_chassis_State.Velocity();

            //Joint
            this->Joint_All.resize(6);
            this->Joint_All[0]=this->model->GetJoint("JOINT_0_Leg2Wheel");
            this->Joint_All[1]=this->model->GetJoint("JOINT_1_Leg2Wheel");
            this->Joint_All[2]=this->model->GetJoint("JOINT_2_Leg2Wheel");
            this->Joint_All[3]=this->model->GetJoint("JOINT_3_Leg2Wheel");
            this->Joint_All[4]=this->model->GetJoint("JOINT_4_Leg2Wheel");
            this->Joint_All[5]=this->model->GetJoint("JOINT_5_Leg2Wheel");

            //Joint Controller: PID
            this->para_pid.SetPGain(1.0);
            this->para_pid.SetIGain(0.0);
            this->para_pid.SetDGain(0.0);
            this->para_pid=common::PID(1.0,0,0);
//            Joint_Ctl=this->model->GetJointController();
//            Joint_Ctl->SetVelocityPID("JOINT_0_Leg2Wheel",this->para_pid);
//            Joint_Ctl->SetVelocityPID(this->Joint_All[0]->GetScopedName(),this->para_pid);

            //Joint Lidar Controller
            const double fmax = 10000;
            double vel_Lidar = 100;
            this->Joint_Lidar=this->model->GetJoint("JOINT_Lidar");

            if (_sdf->HasElement("Lidar_Velcity"))
                vel_Lidar = _sdf->Get<double>("Lidar_Velcity");
            this->Joint_Lidar->SetParam("fmax",0,fmax);
            this->Joint_Lidar->SetParam("vel",0,vel_Lidar);

            //Update
            this->connections.push_back(
                    event::Events::ConnectWorldUpdateBegin(
                            std::bind(&Plugin_6X6::Update,this)));


            //ROS Communication
            if(!ros::isInitialized()){
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client_0", ros::init_options::NoSigintHandler);
            }

            this->rosNode_0.reset(new ros::NodeHandle("gazebo_client_0"));
            ros::SubscribeOptions so_0 = ros::SubscribeOptions::create<std_msgs::Float64>(
                    "/Data_toGazebo",
                    1, boost::bind(&Plugin_6X6::OnRosMsg_0, this, _1),
                    ros::VoidPtr(), &this->rosQueue_0);

            this->rosSub_0 = this->rosNode_0->subscribe(so_0);
            this->rosQueueThread_0 = std::thread(std::bind(&Plugin_6X6::QueueThread_0, this));
            this->updateConnection_0 = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&Plugin_6X6::Update, this));
        }

    public:
        void Update(){
//            this->TractionControl_2(-10);
//            this->model->GetJointController()->SetVelocityTarget("JOINT_Lidar",100.0);
//            this->model->GetJointController()->SetVelocityTarget("link_imu",10.0);

//            this->Link_chassis=model->GetLink("link_0");
//            physics::LinkState link_state(this->Link_chassis);
//            std::cout<<"Link_chassis_Pose: "<<link_state.Pose().Pos().Y()<<std::endl;
        }

        void OnRosMsg_0(const std_msgs::Float64ConstPtr &_msg){
            std::cout<<"OnRosMsg_0_ros_data:"<<_msg->data<<std::endl;
            this->TractionControl_2(_msg->data);
        }

        void QueueThread_0(){
            static const double timeout = 0.01;
            while (this->rosNode_0->ok()){
                this->rosQueue_0.callAvailable(ros::WallDuration(timeout));
            }
        }

        void TractionControl_2(const double &Val){
            const double fmax = 10000;
            for(size_t i=0;i<6;i++){
                this->Joint_All[i]->SetParam("fmax",0,fmax);
                this->Joint_All[i]->SetParam("vel",0,Val);
            }
        }
        void TractionControl_1(const double &Val){
            //Attention: Left Right heading
            const double fmax = 10000;
            this->model->GetJoint("JOINT_0_Leg2Wheel")->SetParam("fmax",0, fmax);
            this->model->GetJoint("JOINT_1_Leg2Wheel")->SetParam("fmax",0, fmax);
            this->model->GetJoint("JOINT_2_Leg2Wheel")->SetParam("fmax",0, fmax);
            this->model->GetJoint("JOINT_3_Leg2Wheel")->SetParam("fmax",0, fmax);
            this->model->GetJoint("JOINT_4_Leg2Wheel")->SetParam("fmax",0, fmax);
            this->model->GetJoint("JOINT_5_Leg2Wheel")->SetParam("fmax",0, fmax);

            this->model->GetJoint("JOINT_0_Leg2Wheel")->SetParam("vel",0, Val);
            this->model->GetJoint("JOINT_1_Leg2Wheel")->SetParam("vel",0, Val);
            this->model->GetJoint("JOINT_2_Leg2Wheel")->SetParam("vel",0, Val);
            this->model->GetJoint("JOINT_3_Leg2Wheel")->SetParam("vel",0, Val);
            this->model->GetJoint("JOINT_4_Leg2Wheel")->SetParam("vel",0, Val);
            this->model->GetJoint("JOINT_5_Leg2Wheel")->SetParam("vel",0, Val);
        }


    private:
        physics::ModelPtr model;
        physics::WorldPtr world;

        physics::LinkPtr Link_chassis;
        physics::LinkState Link_chassis_State;
        ignition::math::Pose3d Link_chassis_Pose;
        ignition::math::Pose3d Link_chassis_Vel;

        std::vector<physics::JointPtr> Joint_All;
        physics::JointPtr Joint_Wheel_LF;

        physics::JointPtr Joint_Lidar;
        physics::JointControllerPtr Joint_Lidar_Ctl;

    private:
        common::PID para_pid;
        physics::JointControllerPtr Joint_Ctl;

    private:
        std::vector<event::ConnectionPtr> connections;

    private:
        std::unique_ptr<ros::NodeHandle> rosNode_0;
        ros::Subscriber rosSub_0;
        ros::CallbackQueue rosQueue_0;
        std::thread rosQueueThread_0;
        event::ConnectionPtr updateConnection_0;
    };


    GZ_REGISTER_MODEL_PLUGIN(Plugin_6X6)
}
