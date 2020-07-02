#include <moveit_msgs/DisplayTrajectory.h>

#include <apriltags/AprilTagDetections.h>

#include <itia_futils/itia_futils.h>
#include <itia_rutils/itia_rutils.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>

class rosPS
{
private:
    
    ros::NodeHandle&                                                     nh;
    ros::Publisher&                                                      planning_scene_diff_publisher;
    ros::Subscriber                                                      ar_sub;
    itia::rutils::MsgReceiver<apriltags::AprilTagDetections>             msg_receiver;
    moveit::planning_interface::MoveGroupInterface                       group;
    tf::TransformBroadcaster                                             br;
    tf::Transform                                                        transform;
    tf::TransformListener                                                listener;
    ros::Publisher                                                       obj_pos_pub;
    
    geometry_msgs::PoseArray                                             object_poses;
    std::vector<int>                                                     obj_tag_array;
    int stop;
    int k;
public:
    rosPS ( ros::NodeHandle& nh,
            ros::Publisher& planning_scene_diff_publisher, 
            const std::string& ar_pose_marker_topic )
    : nh                ( nh )
    , msg_receiver      ( "marker_info_Receiver" )
    , planning_scene_diff_publisher (planning_scene_diff_publisher)
    , group             ( "arm" )
    {
      stop = 0;
      k=0;
      ar_sub = nh.subscribe(ar_pose_marker_topic, 1,&itia::rutils::MsgReceiver<apriltags::AprilTagDetections>::callback, &msg_receiver);
      obj_pos_pub = nh.advertise<geometry_msgs::PoseArray>("/env_obj/pos", 1000);
    }

    ~rosPS() 
    {};
    
 void object_tracking (const apriltags::AprilTagDetections& received_marker_info)
   {
    
    if (stop == 0)
    {
            obj_tag_array.clear();
            std::vector<int> obj_type_array;
            std::vector<std::vector<double>> obj_dimensions_array;
            std::vector<std::vector<double>> tag_to_object_center_array;
              
            for (int i=0;i<=10;i++)
            {
                int obj_tag;
                std::stringstream string_tag_id;
                string_tag_id << "/object_info/object_"<<i+1 <<"/tag_id";
                std::string tag_id_param = string_tag_id.str();
                if (nh.getParam(tag_id_param,obj_tag))
                obj_tag_array.push_back(obj_tag);

                if (!nh.getParam(tag_id_param,obj_tag))
                break;
                
                int obj_type;
                std::stringstream string_obj_tpye;
                string_obj_tpye << "/object_info/object_"<<i+1 <<"/type";
                std::string obj_type_param = string_obj_tpye.str();
                if (nh.getParam(obj_type_param,obj_type))
                obj_type_array.push_back(obj_type);
                
                std::vector<double> obj_dimension;
                std::stringstream string_obj_dimension;
                string_obj_dimension << "/object_info/object_"<<i+1 <<"/dimensions";
                std::string obj_dimension_param = string_obj_dimension.str();
                if (nh.getParam(obj_dimension_param,obj_dimension))
                obj_dimensions_array.push_back(obj_dimension);
                
                std::vector<double> tag_to_object_center;
                std::stringstream string_tag_to_object_center;
                string_tag_to_object_center << "/object_info/object_"<<i+1 <<"/tag_to_object_center";
                std::string tag_to_object_center_param = string_tag_to_object_center.str();
                if (nh.getParam(tag_to_object_center_param,tag_to_object_center))
                tag_to_object_center_array.push_back(tag_to_object_center);
                
            }

            itia::support::printvectorint("obj_tag_array",obj_tag_array);

            geometry_msgs::Pose object_pose;
            
            for ( int iCurM=0; iCurM<received_marker_info.detections.size(); iCurM++ )
              {   
                  int tagID =received_marker_info.detections[iCurM].id;
                  
                  if(tagID == obj_tag_array[k])
                    {
                          std::stringstream ss;
                          ss << "april_tag_" << tagID;
                          std::string markerFrameID = ss.str();

                          itia::support::creattf(received_marker_info.detections[iCurM].pose,"/kinect2_rgb_optical_frame",markerFrameID);
                          
                          tf::StampedTransform mapTransform;
                          
                          try
                          {
                              listener.waitForTransform ( "/world",markerFrameID, ros::Time ( 0 ),ros::Duration ( 0.1 ) );
                              listener.lookupTransform ( "/world", markerFrameID,  ros::Time ( 0 ), mapTransform );
                          }
                          
                          catch ( tf::TransformException &ex )
                          {
                              ROS_ERROR ( "%s",ex.what() );
                              return;
                          }
                  
                          std::vector<double> tag_to_object_center_i = tag_to_object_center_array[k];
                          
                          std::vector<double> obj_pose = { mapTransform.getOrigin().getX() + tag_to_object_center_i[0],
                                                           mapTransform.getOrigin().getY() + tag_to_object_center_i[1],
                                                           mapTransform.getOrigin().getZ() + tag_to_object_center_i[2]};
                          std::vector<double> obj_orient = {1,0,0,0};
                          std::vector<double> obj_dim = obj_dimensions_array[k];
                          
                          if(obj_type_array[k]== 1)
                             itia::moveit_utils::addBox(planning_scene_diff_publisher, obj_pose, obj_orient,obj_dim,"root","world",markerFrameID);
                          else if (obj_type_array[k]== 2)
                             itia::moveit_utils::addCylinder(planning_scene_diff_publisher, obj_pose, obj_orient,obj_dim,"root","world",markerFrameID);
                          
                          itia::support::fromVecToPose(obj_pose,obj_orient,object_pose);
                          
                          object_poses.poses.push_back(object_pose);
                          k=k+1;
                    }
                  
              } // for april_tags_detection_size
            
    } // if stop
    
    if (object_poses.poses.size() == obj_tag_array.size())
    {
    stop = 1;
    }
    } // void
  
  void obj_pos_pub_fun()
  {
     ros::Rate lp(100);
        while (ros::ok())
        {
            if (object_poses.poses.size() == obj_tag_array.size() & object_poses.poses.size() != 0)
            {;
                obj_pos_pub.publish(object_poses);
                std::cout << object_poses << std::endl;
            }
            lp.sleep();
        }
  }
  
  void run( )
    {
        apriltags::AprilTagDetections msg;  
        ros::Rate lp(100);
        while (ros::ok())
        {
            if ( msg_receiver.isANewDataAvailable() )
            {
                msg = msg_receiver.getData();
                object_tracking ( msg);
            }
            lp.sleep();
           
            if (object_poses.poses.size() == obj_tag_array.size() & object_poses.poses.size() != 0)
              break;
        }
    }

};

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "object_tracking" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    sleep (4.0);
    
    itia::support::printString("Adding the environment");

    ROS_INFO("%sAdding the table", BOLDCYAN);
    std::vector<double> table_pose;
    std::vector<double> table_orient;
    std::vector<double> table_dim;

    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    sleep(0.5);

    if (!node_handle.getParam("/scene_configuration/table/position",table_pose))
    ROS_ERROR("Error in reading table pose");
    if (!node_handle.getParam("/scene_configuration/table/orientation",table_orient))
    ROS_ERROR("Error in reading table orientation");    
    if (!node_handle.getParam("/scene_configuration/table/dimensions",table_dim))
    ROS_ERROR("Error in reading table dim");
    
    itia::moveit_utils::addBox(planning_scene_diff_publisher, table_pose, table_orient,table_dim,"root","world","table");
        
    rosPS topics ( node_handle, planning_scene_diff_publisher,"/apriltags/detections" );

    topics.run();
    
    topics.obj_pos_pub_fun();
    
    itia::support::end();
    return 0;
}
