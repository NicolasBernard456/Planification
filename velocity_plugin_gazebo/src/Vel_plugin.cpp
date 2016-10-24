#include "Vel_plugin.hh"
#include <gazebo/physics/Model.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(Vel_plugin)

/////////////////////////////////////////////////
Vel_plugin::Vel_plugin() : ModelPlugin(){
	char* argv[1];
	int argc = 0;
	ros::init(argc, argv,"Vel_plugin");
	ros::NodeHandle n;
	pub = n.advertise<std_msgs::Float64>("/Robot_mobile/x",1);
	pub2 = n.advertise<std_msgs::Float64>("/Robot_mobile/y",1);
	pub3 = n.advertise<std_msgs::Float64>("/Robot_mobile/theta",1);
}

/////////////////////////////////////////////////
Vel_plugin::~Vel_plugin()
{
}
/////////////////////////////////////////////////
void Vel_plugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/){
//   // Get the parent sensor.
  this->model = boost::dynamic_pointer_cast<physics::Model>(_model);
//   // Make sure the parent sensor is valid.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Vel_plugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void Vel_plugin::OnUpdate(const common::UpdateInfo & /*_info*/){
	std_msgs::Float64 rot_z, vel_x, vel_y;
	rot_z.data = this->model->GetWorldAngularVel().z;
	vel_x.data = this->model->GetWorldLinearVel().x;
	vel_y.data = this->model->GetWorldLinearVel().y;
	pub.publish(vel_x);
	pub2.publish(vel_y);
	pub3.publish(rot_z);
}