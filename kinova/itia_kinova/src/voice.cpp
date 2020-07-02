#include <moveit_msgs/DisplayTrajectory.h> // all moveit_operations

#include <sound_play/SoundRequest.h>

#include <itia_futils/itia_futils.h>
#include <itia_tutils/itia_tutils.h>
#include <itia_rutils/itia_rutils.h>

#include <itia_kinova/support.h>
#include <itia_kinova/itia_kinova_utils.h>
#include <itia_kinova/itia_moveit_utils.h>
#include <itia_kinova/itia_grasp_utils.h>

class rosPS
{
private:
    
    ros::NodeHandle&                                        nh;
    ros::Subscriber                                         voice_sub;
    ros::Subscriber                                         tool_pose_sub;
    ros::Publisher                                          sound_pub;
    itia::rutils::MsgReceiver<std_msgs::String>             msg_receiver;
    moveit::planning_interface::MoveGroupInterface          group;
    
    int command,once;
    double qw,qx,qy,qz,distance;
    geometry_msgs::Pose current_eep;
    std::string command_,side_grasp,top_grasp,long_mode,short_mode,medium_mode,home,danger,stop_,first_pos,second_pos,third_pos,face_pos,
                command_voice,side_grasp_voice,top_grasp_voice,long_mode_voice,short_mode_voice,medium_mode_voice,home_voice,stop_voice,
                up_voice,down_voice,left_voice,right_voice,front_voice,back_voice,open_voice,close_voice,clockwise_voice,anti_clockwise_voice,
                first_pos_voice,second_pos_voice,third_pos_voice,face_pos_voice;
                
public:
    rosPS ( ros::NodeHandle& nh, 
            const std::string& voice,
            const std::string& tool_pose)
    : nh                ( nh )
    , msg_receiver      ( "NextPoseReceiver" )
    , group                   ( "arm" )
    {
        once = 0;
        qw = 0.498322844505;
        qx = -0.503124177456;
        qy = 0.499893963337;
        qz = -0.498644709587;
        voice_sub = nh.subscribe(voice, 1,&itia::rutils::MsgReceiver<std_msgs::String>::callback, &msg_receiver);
        sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 1000);
        tool_pose_sub = nh.subscribe(tool_pose, 1000, &rosPS::tool_pose_Callback, this);
    }

    ~rosPS() 
    {};
    
    
    void tool_pose_Callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
    { 
     current_eep.position.x = msg-> pose.position.x;
     current_eep.position.y = msg-> pose.position.y;
     current_eep.position.z = msg-> pose.position.z;
     current_eep.orientation.w = msg-> pose.orientation.w;
     current_eep.orientation.x = msg-> pose.orientation.x;     
     current_eep.orientation.y = msg-> pose.orientation.y;
     current_eep.orientation.z = msg-> pose.orientation.z;
    }
    
    void voice_execute ( const std_msgs::String& recieved_voice )
    {
      std::string voice_rec =  recieved_voice.data.c_str();
      itia::support::printString(voice_rec);
      
      kinova_msgs::ArmPoseGoal goal;
      std::vector<double> joint_values;
      group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), joint_values);
      
      double human_from_robot;
      bool read_human_from_robot =nh.getParam("/voice/human_from_robot",human_from_robot);
      double human_from_robot_y_axis = -1 * human_from_robot;

      bool english_voice;
      bool english_voice_param =nh.getParam("/voice/english_voice_output",english_voice);
      
      double z_below_table_constraint = -0.248;
      
      double long_distance;
      if (current_eep.position.z > 0.075)
      long_distance = 0.25;
      else if (current_eep.position.z > -0.02  && current_eep.position.x > -0.13)
      long_distance = 0.07;
      else if (current_eep.position.z < -0.02  && current_eep.position.x < -0.13)
      long_distance = 0.07;
      
      double medium_distance = 0.07;
      double short_distance = 0.03;
      
      
      
      
      if (once == 0)
      {
            if(!english_voice)
            {
              //italian_voice_output
              command_ = " BENVENUTO, BUONGIORNO ";
// //               side_grasp = " ";
// //               top_grasp = " ";
              long_mode = " LONG MOVEMENT MODE ACTIVATED ";
              short_mode = " SHORT MOVEMENT MODE ACTIVATED ";
              medium_mode = "MEDIUM MOVEMENT MODE ACTIVATED";
              danger = " DANGER, FINGERS MIGHT HIT THE TABLE ";
              stop_ = " GRAZIE ";
// //               home = " ";
// //               first_pos = " ";
// //               second_pos = " ";
// //               third_pos = " ";
// //               face_pos = " ";
              
              //italian_voice_input
              command_voice = "pronto";
              side_grasp_voice="lato afferra";
              top_grasp_voice="sopra afferra";
              long_mode_voice="grande";
              short_mode_voice="piccolo";
              medium_mode_voice ="medio";
              up_voice="solleva";
              down_voice="abbassa";
              left_voice="indietro";
              right_voice="avanti";
              front_voice="vicino";
              back_voice="lontano";
              open_voice="apri";
              close_voice="stringi";
              home_voice="posizione casa";
              clockwise_voice="ruota";
              anti_clockwise_voice="antiruota";
              stop_voice="stop";
              first_pos_voice = "posizione prima";
              second_pos_voice = "posizione seconda";
              third_pos_voice = "posizione terza";
              face_pos_voice = "posizione faccia";
            }
            else
            {
              //english_voice_output
              command_ = " I AM AT YOUR SERVICE, PLEASE GIVE A COMMAND ";
              side_grasp = " SIDE GRASP MOVEMENT MODE ACTIVATED ";
              top_grasp = " TOP GRASP MODE ACTIVATED ";
              long_mode = " LONG MOVEMENT MODE ACTIVATED ";
              short_mode = " SHORT MOVEMENT MODE ACTIVATED ";
              medium_mode = "MEDIUM MOVEMENT MODE ACTIVATED";
              home = " GOING, TO, HOME, POSITION";
              danger = " DANGER ";
              stop_ = " THANK YOU, HOPE I DID WELL ";
              first_pos = "GOING, TO, FIRST, POSITION";
              second_pos = "GOING, TO, SECOND, POSITION ";
              third_pos = "GOING, TO, THIRD, POSITION";
              
              //english_voice_input
              command_voice = "command";
              side_grasp_voice="side grasp";
              top_grasp_voice="top grasp";
              long_mode_voice="long";
              short_mode_voice="small";
              medium_mode_voice ="medium";
              up_voice="up";
              down_voice="down";
              left_voice="left";
              right_voice="right";
              front_voice="front";
              back_voice="back";
              open_voice="open";
              close_voice="close";
              home_voice="home position";
              first_pos_voice="first position";
              second_pos_voice="second position";
              third_pos_voice="third position";
              clockwise_voice="clockwise";
              anti_clockwise_voice="anticlockwise";
              stop_voice="finish";
            }
            distance = long_distance;
            once = once + 1;     
      }
      
  /////////////////////
 // start command   //        
/////////////////////      
      
      if (voice_rec == command_voice )
      {
          itia::support::play_sound(nh,sound_pub,command_);
          command = 1;
      }
      
      if (command == 1)
      {   

  //////////////////
 // grasp mode   //        
//////////////////

        if (voice_rec == side_grasp_voice)
        {
            itia::support::play_sound(nh,sound_pub,side_grasp);
            qw = 0.498322844505;
            qx = -0.503124177456;
            qy = 0.499893963337;
            qz = -0.498644709587;
            itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,current_eep.position.z,qw,qx,qy,qz);            
        }

        else if (voice_rec == top_grasp_voice)
        {
            itia::support::play_sound(nh,sound_pub,top_grasp);
            qw = 0.0164206009358;
            qx = 0.781286716461;
            qy = 0.623637676239;
            qz = -0.0199354216456;
            itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,current_eep.position.z,qw,qx,qy,qz);  
        }

  /////////////////////
 // distance mode   //        
/////////////////////

        else if (voice_rec == long_mode_voice)
        {  
            distance = long_distance;
            itia::support::play_sound(nh,sound_pub,long_mode);
        }
        
        else if (voice_rec == medium_mode_voice)
        {  
            distance = medium_distance;
            itia::support::play_sound(nh,sound_pub,medium_mode);
        }

        else if (voice_rec == short_mode_voice)
        {
            distance = short_distance;
            itia::support::play_sound(nh,sound_pub,short_mode);
        }
        
  /////////////////////
 // finger command  //        
/////////////////////

        else if (voice_rec == close_voice )
              itia::kinova_utils::closehand(false,false);
        
        else if (voice_rec == open_voice)
              itia::kinova_utils::openhand(false,false);

  ///////////////////////
 // position command  //        
///////////////////////
        
        else if (voice_rec == home_voice) //reset to default
        {
            if (current_eep.position.z > 0.05)
            {
            itia::support::play_sound(nh,sound_pub,home);
            itia::kinova_utils::gohome();
            distance = long_distance;
            qw = 0.498322844505;
            qx = -0.503124177456;
            qy = 0.499893963337;
            qz = -0.498644709587;
            }
            else if (current_eep.position.z < 0.05)
            {
            itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,0.0,qw,qx,qy,qz);
            sleep(2.0);
            itia::support::play_sound(nh,sound_pub,home);
            itia::kinova_utils::gohome();
            distance = long_distance;
            qw = 0.498322844505;
            qx = -0.503124177456;
            qy = 0.499893963337;
            qz = -0.498644709587;
            }
        }

        else if (voice_rec == first_pos_voice)
        {
            if (current_eep.position.z > 0.05)
            {
              itia::support::play_sound(nh,sound_pub,first_pos);
              itia::kinova_utils::move(voice_rec,0.63,-0.3,0.4,qw,qx,qy,qz);
            }
            else if (current_eep.position.z < 0.05)
            {
            itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,0.0,qw,qx,qy,qz);
            sleep(2.0);
            itia::support::play_sound(nh,sound_pub,first_pos);
            itia::kinova_utils::move(voice_rec,0.63,-0.3,0.4,qw,qx,qy,qz);
            }
        }
        
        else if (voice_rec == second_pos_voice)
        {
            if (current_eep.position.z > 0.05)
            {
              itia::support::play_sound(nh,sound_pub,second_pos);
              itia::kinova_utils::move(voice_rec,0.63,-0.3,0.06,qw,qx,qy,qz);
            }
            else if (current_eep.position.z < 0.05)
            {
            itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,0.0,qw,qx,qy,qz);
            sleep(2.0);
            itia::support::play_sound(nh,sound_pub,second_pos);
            itia::kinova_utils::move(voice_rec,0.63,-0.3,0.06,qw,qx,qy,qz);
            }
        }
        
        else if (voice_rec == third_pos_voice)
        {
            if (current_eep.position.z > 0.05)
            {
              itia::support::play_sound(nh,sound_pub,third_pos);
              itia::kinova_utils::move(voice_rec,0.3,-0.3,0.06,qw,qx,qy,qz);
            }
            else if (current_eep.position.z < 0.05)
            {
            itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,0.0,qw,qx,qy,qz);
            sleep(2.0);
            itia::support::play_sound(nh,sound_pub,third_pos);
            itia::kinova_utils::move(voice_rec,0.3,-0.3,0.06,qw,qx,qy,qz);
            }
        }
        
        else if (voice_rec == face_pos_voice)
        {
            if (current_eep.position.z > 0.05)
            {
              itia::support::play_sound(nh,sound_pub,face_pos);
              itia::kinova_utils::move(voice_rec,0.3,-0.664,0.2,qw,qx,qy,qz);
            }
            else if (current_eep.position.z < 0.05)
            {
            itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,0.0,qw,qx,qy,qz);
            sleep(2.0);
            itia::support::play_sound(nh,sound_pub,face_pos);
            itia::kinova_utils::move(voice_rec,0.3,-0.664,0.2,qw,qx,qy,qz);
            }
        }
        
  /////////////////////
 // rotate command  //        
/////////////////////

        else if (voice_rec == clockwise_voice )
        {
           if (current_eep.position.z > 0.05)
           {
             itia::kinova_utils::move_joint(voice_rec,joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]-0.5);
             sleep(2.0);
             qw = current_eep.orientation.w;
             qx = current_eep.orientation.x;
             qy = current_eep.orientation.y;
             qz = current_eep.orientation.z;
           }
           else if (current_eep.position.z < 0.05 && current_eep.position.x < -0.13)
           {
             itia::kinova_utils::move_joint(voice_rec,joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]-0.5);
             sleep(2.0);
             qw = current_eep.orientation.w;
             qx = current_eep.orientation.x;
             qy = current_eep.orientation.y;
             qz = current_eep.orientation.z;
           }
           else
            itia::support::play_sound(nh,sound_pub,danger);
        }
        
        else if (voice_rec == anti_clockwise_voice )
        {
           if (current_eep.position.z > 0.05)
           {
             itia::kinova_utils::move_joint(voice_rec,joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]+0.5);
             sleep(2.0);
             qw = current_eep.orientation.w;
             qx = current_eep.orientation.x;
             qy = current_eep.orientation.y;
             qz = current_eep.orientation.z;
          }
           else if (current_eep.position.z < 0.05 && current_eep.position.x < -0.13)
           {
             itia::kinova_utils::move_joint(voice_rec,joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]+0.5);
             sleep(2.0);
             qw = current_eep.orientation.w;
             qx = current_eep.orientation.x;
             qy = current_eep.orientation.y;
             qz = current_eep.orientation.z;
          }
          else
            itia::support::play_sound(nh,sound_pub,danger);
        }
        
  ///////////////////
 // move command  //        
///////////////////        
        
         else if (voice_rec == right_voice)
         {
           if (current_eep.position.z > -0.5 )
                  itia::kinova_utils::move(voice_rec,current_eep.position.x + distance,current_eep.position.y,current_eep.position.z,qw,qx,qy,qz);
           if (current_eep.position.z < 0.0 && current_eep.position.x < -0.13 )
                  itia::support::play_sound(nh,sound_pub,danger);
         }
         
         else if (voice_rec == left_voice)
              itia::kinova_utils::move(voice_rec,current_eep.position.x - distance,current_eep.position.y,current_eep.position.z,qw,qx,qy,qz);
        
         else if (voice_rec == front_voice)
          {
            if (current_eep.position.y - distance > human_from_robot_y_axis)
              itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y - distance,current_eep.position.z,qw,qx,qy,qz);
            
            else if (current_eep.position.y - distance < human_from_robot_y_axis & current_eep.position.y - short_distance > human_from_robot_y_axis)
            {
              itia::support::play_sound(nh,sound_pub,danger);
              itia::kinova_utils::move(voice_rec,current_eep.position.x,human_from_robot_y_axis,current_eep.position.z,qw,qx,qy,qz);
            }
            else if (current_eep.position.y - short_distance < human_from_robot_y_axis)
            {
              itia::support::play_sound(nh,sound_pub,danger);
              itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y - short_distance,current_eep.position.z,qw,qx,qy,qz);
            }
          }
        
        else if (voice_rec == back_voice)
              itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y + distance,current_eep.position.z,qw,qx,qy,qz);
        
        else if (voice_rec == up_voice)
              itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,current_eep.position.z + distance,qw,qx,qy,qz);

        
        else if (voice_rec == down_voice)
        {
            if (current_eep.position.z > distance && current_eep.position.x > -0.13 )
                  itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,current_eep.position.z - distance,qw,qx,qy,qz);
            else if (current_eep.position.z > 0.01 && current_eep.position.x > -0.13 )
            {
              itia::support::play_sound(nh,sound_pub,danger);
              itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,0.0,qw,qx,qy,qz); 
            }
            else if (current_eep.position.z > distance && current_eep.position.x < -0.13 )
              itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,current_eep.position.z - distance,qw,qx,qy,qz);
            else if (current_eep.position.z > z_below_table_constraint && current_eep.position.x < -0.13 )
            {
              itia::support::play_sound(nh,sound_pub,danger);
              itia::kinova_utils::move(voice_rec,current_eep.position.x,current_eep.position.y,z_below_table_constraint,qw,qx,qy,qz);
            }
            else
              itia::support::play_sound(nh,sound_pub,danger);
        }
        
        
  ///////////////////
 // stop command  //        
///////////////////  
        
        else if (voice_rec == stop_voice)
        {
            itia::support::play_sound(nh,sound_pub,stop_);
            command = 0;
        }
        
      } // if command
      

    } // callback
    
  void run( )
    {
        std_msgs::String msg;
    
        ros::Rate lp(10);
        while (ros::ok())
        {
            if ( msg_receiver.isANewDataAvailable() )
            {
                msg = msg_receiver.getData();
                voice_execute( msg);
            }
            lp.sleep();
        }
    }

};

int main ( int argc, char **argv )
{
    itia::support::printString ( "intializing" );

    ros::init ( argc, argv, "voice" );
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner ( 1 );
    spinner.start();
    sleep (4.0);
    
    bool gazebo = false;  bool fake = false;
    itia::kinova_utils::active_controller(node_handle,"/voice/gazebo","/voice/fake",fake,gazebo);
    
    ros::Publisher sound_pub = node_handle.advertise<sound_play::SoundRequest>("/robotsound", 1000);
    bool english_voice;
    bool english_voice_param =node_handle.getParam("/voice/english_voice_output",english_voice);
    
    if(english_voice)
    {
    itia::support::play_sound(node_handle,sound_pub,"SWITCHED, ON A");
    sleep(2.0);
    itia::support::play_sound(node_handle,sound_pub,"ENGLISH, VOICE, OUTPUT ");
    }
    else
    {
    itia::support::play_sound(node_handle,sound_pub," ACCESO ");
    sleep(2.0);
    itia::support::play_sound(node_handle,sound_pub,"ITALIAN, VOICE, OUTPUT ");
    }
        
    rosPS topics ( node_handle,
                   "/recognizer/output",
                   "/j2n6s300_driver/out/tool_pose");

    topics.run();

    itia::support::end();
    return 0;
}