
#include <pwd.h>
#include <unistd.h>
#include <sys/types.h>

#include <cmath>
#include <functional>   // std::minus
#include <numeric>      // std::accumulate
#include <dirent.h>
#include <random>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h> 

#include <rosdyn_identification/rosdyn_par_estim_interface.h>
#include <tsupport/basic.h>

// #include<Eigen/StdVector>

PLUGINLIB_EXPORT_CLASS(rosdyn::MetoParEstimInterfaceNodelet, nodelet::Nodelet) 

namespace rosdyn
{
      
  inline int getdir (const std::string& dir, std::vector<std::string>& files)
  {
    DIR *dp;
    struct dirent *dirp;
    
    if((dp  = opendir(dir.c_str())) == NULL) 
    {
      std::cout << "Error(" << errno << ") opening " << dir << std::endl;
      return errno;
    }
    
    while ((dirp = readdir(dp)) != NULL) 
      files.push_back( std::string(dirp->d_name) );    
    
    closedir(dp);
    return 0;
  }
  
  void MetoParEstimInterfaceNodelet::onInit()
  {
    std::vector<std::string> args = getMyArgv();
  
    m_stop = false;
    
    m_meto_par_estim_as.reset(new actionlib::SimpleActionServer<rosdyn_identification_msgs::MetoParEstimAction>( getNodeHandle(), "meto_param_estimation", 
                              boost::bind( &MetoParEstimInterfaceNodelet::metoParEstimCB,  this, _1 ), false   ));
    
    m_meto_par_estim_as->start();
    
    m_save_model_server = getNodeHandle().advertiseService("meto_save_model",&rosdyn::MetoParEstimInterfaceNodelet::saveXmlCallback,this);
    
    m_main_thread  = std::thread(&rosdyn::MetoParEstimInterfaceNodelet::main, this);
    
  };


  MetoParEstimInterfaceNodelet::~MetoParEstimInterfaceNodelet()
  {
    m_stop = true;
    if (m_main_thread.joinable())
      m_main_thread.join(); 
  };

  
  void MetoParEstimInterfaceNodelet::metoParEstimCB( const rosdyn_identification_msgs::MetoParEstimGoalConstPtr& goal )
  { 
    
    std::string trj_name_           = goal->trj_namespace;
    
//     std::string add_info_namespace_ = goal->add_info_namespace;
//     std::string add_info_name_      = goal->add_info_name;
//     std::string add_info_path_      = goal->add_info_path;

    rosdyn_identification_msgs::MetoParEstimResult m_result_;
    rosdyn_identification_msgs::MetoParEstimFeedback m_feedback_;
    
    bool verbose_ = true;
    if ( !getNodeHandle().getParam( std::string(std::string(m_namespace)+"/verbose").c_str(), verbose_) )
    {
      ROS_DEBUG("Impossible to find %s/verbose, set false\n", m_namespace.c_str() );
      verbose_=false;
    }
    
    std::string topic_type_ = "JointState";
    std::string log_topic_name_ = goal->trj_namespace;
    
    std::string topic_name_tmp_ = log_topic_name_;
    std::replace( topic_name_tmp_.begin(), topic_name_tmp_.end(), '/', '_');
        
    std::string robot_description;
    if (!getNodeHandle().getParam("robot_description", robot_description) )
    {
      ROS_ERROR("Impossible to find robot_description\n");
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }
    
    
    
    std::vector<std::string> controller_joint_names_;
    if ( !getNodeHandle().getParam(std::string(m_namespace+"/controller_joint_names"), controller_joint_names_) )
    {
      ROS_ERROR("Impossible to find %s\n", std::string(m_namespace+"/controller_joint_names").c_str() );
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }
    
    m_estimator.reset(new rosdyn::MetoParEstim ( getNodeHandle(), robot_description));
    
    m_model_name = m_estimator->getRobotName();
    std::vector<std::string> joints_name = m_estimator->getRobotJointName();
    unsigned int number_of_joint_from_xml_ = joints_name.size();
    
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string test_path_( homedir + std::string("/.ros") );
    
    // Find all the files that need to be loaded
    std::string dir = test_path_ + "/";
    std::vector<std::string> files = std::vector<std::string>();

    if (getdir( dir, files ))
    {
      ROS_ERROR("No valid file to be loaded %s\n", test_path_.c_str() );
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }

    std::vector<std::string> file_to_be_loaded_;
    for ( auto singleFile : files ) 
    {
      std::size_t pos_file_       = singleFile.find( trj_name_ );
      std::size_t pos_topic_      = singleFile.find( topic_name_tmp_ );
      std::size_t pos_is_fake_    = singleFile.find( "fake_controller" );    
      std::size_t pos_extension_  = singleFile.find( ".bin" );   
      
      if( (pos_file_ != std::string::npos) && 
          ( pos_topic_ != std::string::npos ) &&
          ( pos_is_fake_ == std::string::npos) && 
          ( pos_extension_ == singleFile.size()-4 ) )
        file_to_be_loaded_.push_back( singleFile.substr( pos_file_ ) );
    }
    
    if ( file_to_be_loaded_.size() == 0 )
    {
      ROS_ERROR("No valid file to be loaded %s\n", test_path_.c_str() );
      m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
      m_meto_par_estim_as->setAborted(m_result_);
      return;
    }
    
    
    std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd>> vctFileData;
    
    for ( const std::string& fileToLoad : file_to_be_loaded_ )
    {
      ROS_INFO("Loading a new binary file, please wait a moment...");
    
      m_file_name = test_path_ + "/" + fileToLoad;
      
      std::ifstream is;
      is.open (m_file_name, std::ios::in | std::ios::binary );
      
      // get length of file:
      is.seekg ( 0, std::ios::end );
      unsigned int length = is.tellg(); 
      is.seekg ( 0, std::ios::beg );
      
      // read data as a block:
      int number_of_sample = length / sizeof(double) / ( 1+3*number_of_joint_from_xml_ );
      
      Eigen::MatrixXd singleFileData( (1+3*number_of_joint_from_xml_), number_of_sample );
      singleFileData.setZero();
      
      if( length != ( singleFileData.rows() * singleFileData.cols() * sizeof(double) ) )
      {
        ROS_ERROR("Dimensions mismatch between the number of bytes in the binary file and the reconstructed matrix! (file name: %s)",fileToLoad.c_str());
        ROS_ERROR("length: %d", length);
        ROS_ERROR("rows(): %zu", singleFileData.rows());
        ROS_ERROR("cols(): %zu", singleFileData.cols());
        ROS_ERROR("sizeof(double): %zu", sizeof(double));
        
        m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
        m_meto_par_estim_as->setAborted(m_result_);
        return;
      }  
      
      is.read ( (char*)singleFileData.data(), length );
      is.close();
      
      vctFileData.push_back( singleFileData );
      
      ROS_INFO("File: %s  loaded!", fileToLoad.c_str() );  
    }
    
    int nCols = 0;
    int nRows = vctFileData.front().rows();
    for ( auto singFile : vctFileData )
    {
      nCols += singFile.cols();
      if ( nRows != singFile.rows() )
      {
        ROS_ERROR("Dimensions mismatch: data extracted from two different files have a different number of rows.");
        m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::GENERIC_ERROR;
        m_meto_par_estim_as->setAborted(m_result_);
        return;
      }
    }
    
    Eigen::MatrixXd qf_full( nCols, number_of_joint_from_xml_ ), 
                    Dqf_full( nCols, number_of_joint_from_xml_ ), 
                    DDqf_full( nCols, number_of_joint_from_xml_ ), 
                    efff_full( nCols, number_of_joint_from_xml_ );

    qf_full.setZero(); Dqf_full.setZero(); DDqf_full.setZero(); efff_full.setZero();
    
    ROS_INFO("Starting to compute dynamics parameters...");
    
    int st_row_ = 0;
    int size_row_ = 0;
    for ( auto singFile : vctFileData )
    {        
      Eigen::MatrixXd fileData = singFile;
      
      if (verbose_)
        ROS_INFO("Loaded new data file with %zu rows and %zu columns", fileData.rows(), fileData.cols() );
      
      Eigen::MatrixXd q( singFile.cols(), number_of_joint_from_xml_ ),
                      Dq( singFile.cols(), number_of_joint_from_xml_ ), 
                      DDq( singFile.cols(), number_of_joint_from_xml_ ), 
                      eff( singFile.cols(), number_of_joint_from_xml_ );
      
      Eigen::MatrixXd qf( singFile.cols(), number_of_joint_from_xml_ ), 
                      Dqf( singFile.cols(), number_of_joint_from_xml_ ), 
                      DDqf( singFile.cols(), number_of_joint_from_xml_ ), 
                      efff( singFile.cols(), number_of_joint_from_xml_ );                
                      
      q.setZero(); Dq.setZero(); DDq.setZero(); eff.setZero();      
      qf.setZero(); Dqf.setZero(); DDqf.setZero(); efff.setZero();
            
      for ( unsigned int idxJnt=0; idxJnt<number_of_joint_from_xml_; idxJnt++ )
      {      
        q.col(idxJnt) = fileData.row(   idxJnt + 1 ).transpose();
        Dq.col(idxJnt) = fileData.row(  idxJnt + 1 + number_of_joint_from_xml_  ).transpose();
        eff.col(idxJnt) = fileData.row( idxJnt + 1 + number_of_joint_from_xml_ * 2 ).transpose();
        
        eigen_control_toolbox::FirstOrderLowPass filt_pos;   
        filt_pos.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_pos.setStateFromLastIO(      q.col(idxJnt).head(1),   q.col(idxJnt).head(1));
        
        eigen_control_toolbox::FirstOrderLowPass filt_vel;
        filt_vel.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_vel.setStateFromLastIO(     Dq.col(idxJnt).head(1),  Dq.col(idxJnt).head(1));
     
        Eigen::VectorXd init_acc(1);
        init_acc.setZero();
        eigen_control_toolbox::FirstOrderHighPass filt_acc;
        filt_acc.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_acc.setStateFromLastIO(    init_acc,                Dq.col(idxJnt).head(1));
        
        eigen_control_toolbox::FirstOrderLowPass filt_eff;
        filt_eff.setStateFromLastIO( eff.col(idxJnt).head(1), eff.col(idxJnt).head(1));
        filt_eff.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_eff.setStateFromLastIO( eff.col(idxJnt).head(1), eff.col(idxJnt).head(1));
        
        for (unsigned int iStep=0; iStep<q.rows(); iStep++)  
        {
          
          qf(iStep, idxJnt)     = filt_pos.update(q(iStep, idxJnt));
          Dqf(iStep, idxJnt)    = filt_vel.update(Dq(iStep, idxJnt));
          DDqf(iStep, idxJnt)   = filt_acc.update(Dq(iStep, idxJnt));
          efff(iStep, idxJnt)   = filt_eff.update(eff(iStep, idxJnt));
          
        }
        
      }
      
//////////////////////////////
//                          //                          
//                          // 
// terrin addition start    //
//                          //                          
//                          //  
//////////////////////////////

      qf.setZero(); Dqf.setZero(); DDqf.setZero(); efff.setZero();
      
        tsupport::basic::print_green("Reading the file info \n");
        
        int samples = singFile.cols();
        int dof = number_of_joint_from_xml_;

        Eigen::VectorXd org_time(samples);
        
        Eigen::MatrixXd org_q( samples, dof ),
                        org_Dq( samples, dof ),
                        org_DDq( samples, dof ),
                        org_eff( samples, dof );

        org_q.setZero(); org_Dq.setZero(); org_DDq.setZero(); org_eff.setZero();  
        org_q = q;
        org_Dq = Dq;
        org_eff = eff;
        
        double org_start_time = fileData(0,0);
        
        org_time = fileData.row(0).transpose() - (org_start_time * Eigen::VectorXd::Ones(samples));
        
//         std::ofstream org_eff_file;
//         org_eff_file.open ("/home/terrin/projects/virtual_force_sensor/files/log_filtering/zero_phase_filter/PI_org_eff.txt");
//             org_eff_file << org_eff << "\n";
//         org_eff_file.close();
//   
// // // //  acceleration estimation
    
      tsupport::basic::print_green("Acceleration \n");

      double rate_hz = 125;

      Eigen::VectorXd Dq_6(dof),Dq_last_6(dof),DDq_6(dof);
      Dq_last_6.setZero();
      DDq_6.setZero();

      for (unsigned int iStep=0; iStep<q.rows(); iStep++)  
        {
            Dq_6  = Dq.row(iStep);
            DDq_6=(Dq_6-Dq_last_6)*rate_hz;
            Dq_last_6=Dq_6;

            for (int idxJnt = 0;idxJnt<dof;idxJnt++)
                org_DDq(iStep, idxJnt)   = DDq_6(idxJnt);
        }
     
     // Band Pass Filtering
      
      tsupport::basic::print_green("Start Band Pass Filtering \n");
      
      
      Eigen::MatrixXd bpf_q( samples, dof ),
                      bpf_Dq( samples, dof ),
                      bpf_DDq( samples, dof ),
                      bpf_eff( samples, dof );
                        
      for ( unsigned int idxJnt=0; idxJnt<dof; idxJnt++ )
      {      

        eigen_control_toolbox::FirstOrderLowPass filt_pos;   
        filt_pos.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_pos.setStateFromLastIO(      q.col(idxJnt).head(1),   q.col(idxJnt).head(1));
        
        eigen_control_toolbox::FirstOrderLowPass filt_vel;
        filt_vel.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_vel.setStateFromLastIO(     Dq.col(idxJnt).head(1),  Dq.col(idxJnt).head(1));
     
        Eigen::VectorXd init_acc(1);
        init_acc.setZero();
        eigen_control_toolbox::FirstOrderHighPass filt_acc;
        filt_acc.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_acc.setStateFromLastIO(    init_acc,                Dq.col(idxJnt).head(1));
        
        eigen_control_toolbox::FirstOrderLowPass filt_eff;
        filt_eff.setStateFromLastIO( eff.col(idxJnt).head(1), eff.col(idxJnt).head(1));
        filt_eff.importMatricesFromParam(      getNodeHandle(),m_namespace + "/filter");
        filt_eff.setStateFromLastIO( eff.col(idxJnt).head(1), eff.col(idxJnt).head(1));
        
        for (unsigned int iStep=0; iStep<q.rows(); iStep++)  
        {
          
          bpf_q(iStep, idxJnt)     = filt_pos.update(q(iStep, idxJnt));
          bpf_Dq(iStep, idxJnt)    = filt_vel.update(Dq(iStep, idxJnt));
          bpf_DDq(iStep, idxJnt)   = filt_acc.update(Dq(iStep, idxJnt));
          bpf_eff(iStep, idxJnt)   = filt_eff.update(eff(iStep, idxJnt));
        }
        
      }
      
        
// // // // Zero phase Filtering
    
    tsupport::basic::print_green("Start Zero Phase Filtering \n");

    int N_Dq = 15;
    int N_DDq = 10;
    int N_eff = 15;

    Eigen::MatrixXd zpf_Dq(samples,dof), 
                    zpf_DDq(samples,dof), 
                    zpf_eff(samples,dof);

    Eigen::VectorXd  Input_Dq(samples),Output_Dq(samples),
                     Input_DDq(samples),Output_DDq(samples),
                     Input_eff(samples),Output_eff(samples);

    for ( unsigned int idxJnt=0; idxJnt<dof; idxJnt++ )
    {   
        Input_Dq = Dq.col(idxJnt);
        Input_DDq = DDq.col(idxJnt);
        Input_eff = eff.col(idxJnt);
        
        tsupport::basic::zero_phase_MA_filter (Input_Dq,N_Dq,Output_Dq);
        tsupport::basic::zero_phase_MA_filter (Input_DDq,N_DDq,Output_DDq);
        tsupport::basic::zero_phase_MA_filter (Input_eff,N_eff,Output_eff);
        
        zpf_Dq.col(idxJnt) = Output_Dq;
        zpf_DDq.col(idxJnt) = Output_DDq;
        zpf_eff.col(idxJnt) = Output_eff;
    }
        
// // // Kalman Filtering
    
    tsupport::basic::print_green("Start Kalman Filtering \n");
    
    Eigen::MatrixXd kf_Dq(samples,dof), 
                    kf_DDq(samples,dof), 
                    kf_eff(samples,dof);

    double t = 1/rate_hz;
    ros::Rate rate(rate_hz);
    double P_gain,D_gain,I_gain,sd_q,sd_Dq,sd_DDq,sd_tau,error_zero_tolerance;
    double P_gain_tau,D_gain_tau,I_gain_tau,error_zero_tolerance_tau;
    ros::NodeHandle nh;
    
    nh.getParam("/std_dev/q",sd_q);
    nh.getParam("/std_dev/Dq",sd_Dq);
    nh.getParam("/std_dev/DDq",sd_DDq);
    nh.getParam("/std_dev/tau",sd_tau);
    
    nh.getParam("/motion_KF/error_zero_tolerance",error_zero_tolerance);
    nh.getParam("/motion_KF/P_gain",P_gain);
    nh.getParam("/motion_KF/I_gain",I_gain);
    nh.getParam("/motion_KF/D_gain",D_gain);
    
    nh.getParam("/tau_KF/error_zero_tolerance",error_zero_tolerance_tau);
    nh.getParam("/tau_KF/P_gain",P_gain_tau);
    nh.getParam("/tau_KF/I_gain",I_gain_tau);
    nh.getParam("/tau_KF/D_gain",D_gain_tau);

    std::cout << "/motion_KF/error_zero_tolerance : " << error_zero_tolerance << std::endl;
    std::cout << "/motion_KF/P_gain : " << P_gain << std::endl;
    std::cout << "/motion_KF/I_gain : " << I_gain << std::endl;
    std::cout << "/motion_KF/D_gain : " << D_gain << std::endl;
    
    std::cout << "/tau_KF/error_zero_tolerance : " << error_zero_tolerance_tau << std::endl;
    std::cout << "/tau_KF/P_gain : " << P_gain_tau << std::endl;
    std::cout << "/tau_KF/I_gain : " << I_gain_tau << std::endl;
    std::cout << "/tau_KF/D_gain : " << D_gain_tau << std::endl;

    double sd3_q = 3*sd_q;
    double sd3_Dq = 3*sd_Dq;
    double sd3_DDq = 3*sd_DDq;
    double sd3_tau = 3*sd_tau;

    Eigen::VectorXd q6(dof),Dq6(dof),tau6(dof),Dq_last6(dof),DDq6(dof);
    Dq_last6.setZero(); DDq6.setZero();

// motion_KF Initialization 
    
    Eigen::Matrix3d A,P_k_minus1,R,P_kp, K,P_k,I;
    Eigen::Vector3d B((pow(t,3))/6,(pow(t,2))/2, t);
    Eigen::MatrixXd X_k_minus1(3,dof),Y_k(3,dof),X_k(3,dof),X_kp(3,dof);
    Eigen::VectorXd J(dof), EF(dof), diff_EF(dof), inter_EF(dof), EF_last(dof),P(dof),D(dof);;
    
    J.setZero(); EF.setZero(); EF_last.setZero(); diff_EF.setZero(); inter_EF.setZero();
    
    A << 1, t, (pow(t,2))/2,
        0, 1, t,
        0, 0, 1;

    P_k_minus1 << sd3_q,    0,      0,
                    0,        sd3_Dq, 0,
                    0,        0,      sd3_DDq;
                    
    R = P_k_minus1;
    
    I = I.Identity();

// tau_KF Initialization 

    double P_k_minus1_tau = sd3_tau;  
    double R_tau = P_k_minus1_tau;
    Eigen::MatrixXd X_k_minus1_tau(1,dof),Y_k_tau(1,dof), X_k_tau(1,dof),X_kp_tau(1,dof);
    double P_kp_tau,K_tau,P_k_tau;
    Eigen::VectorXd delta(dof), EF_tau(dof), diff_EF_tau(dof), inter_EF_tau(dof), EF_last_tau(dof); 
    delta.setZero();EF_tau.setZero();diff_EF_tau.setZero();inter_EF_tau.setZero();EF_last_tau.setZero();
    int count = 1;

for (unsigned int iStep=0; iStep<q.rows(); iStep++)  
    {
        
        q6   = q.row(iStep);
        Dq6  = Dq.row(iStep);
        tau6 = eff.row(iStep);
    
        DDq6=(Dq6-Dq_last6)*rate_hz;
        Dq_last6=Dq6;

        // motion_KF 
        
        if (count == 1)
        {
            for (int i = 0;i<dof;i++)
            {
                X_k_minus1(0,i)=q6(i);
                X_k_minus1(1,i) = Dq6(i);
                X_k_minus1(2,i) = DDq6(i);
            }
        }

        X_kp = A*X_k_minus1 + B*J.transpose();
        P_kp = A*P_k_minus1*A.transpose();
        
        P_kp(1,2)=0;    P_kp(1,3)=0;
        P_kp(2,1)=0;    P_kp(2,3)=0;
        P_kp(3,1)=0;    P_kp(3,2)=0;
        
        K = P_kp*((P_kp+R).inverse());
        
        for (int i = 0;i<dof;i++)
        {
            Y_k(0,i) =q6(i);
            Y_k(1,i) = Dq6(i);
            Y_k(2,i) = DDq6(i);
        }

        X_k = X_kp + K*(Y_k-X_kp);
        P_k = (I-K)*P_kp; 
        
        X_k_minus1 = X_k;
        P_k_minus1 = P_k;

        // motion_KF - jerk estimation with PID controller
        
        for (int i = 0;i<dof;i++)
        {
            EF(i) = Y_k(1,i) - X_k(1,i);
            if (abs(EF(i)) < error_zero_tolerance)
                EF(i) = 0;
        }

        diff_EF = (EF - EF_last)/t;
        inter_EF = ((EF - EF_last)/2)*t;
        EF_last = EF;

        // gain sheduling
        
        for (unsigned int i=0;i<dof;i++)
        {
            if (abs(Dq6(i))<0.01 )
            {
                P(i)=5;D(i)=5;    
            }
            else if(abs(Dq6(i))<0.1)
            {
                P(i)=50;D(i)=20;
            }
            else if(abs(Dq6(i))<0.3)
            {
                P(i)=100;D(i)=30;
            }
            else if(abs(Dq6(i))<0.5)
            {
                P(i)=200;D(i)=45;
            }
            else if(abs(Dq6(i))<0.8)
            {
                P(i)=400;D(i)=90;
            }
            else
            {
                P(i)=600;D(i)=100;
            }
            
            J(i) = (P(i)*EF(i)) + (D(i)*diff_EF(i));
        }
        
        // tau_KF
        
        if (count == 1)
        {
            for (int i = 0;i<dof;i++)
                X_k_minus1_tau(0,i)=tau6(i);
        }
        
        X_kp_tau = X_k_minus1_tau + t*delta.transpose();
        P_kp_tau = P_k_minus1_tau;
        K_tau = P_kp_tau/(P_kp_tau+R_tau);
    
        for (int i = 0;i<dof;i++)
            Y_k_tau(0,i) =tau6(i);
    
        X_k_tau = X_kp_tau + K_tau*(Y_k_tau-X_kp_tau);
        P_k_tau = (1-K_tau)*P_kp_tau; 
    
        X_k_minus1_tau = X_k_tau;
        P_k_minus1_tau = P_k_tau;

        // tau_KF - delta estimation with PID controller
        
        for (int i = 0;i<dof;i++)
        {
            EF_tau(i) = Y_k_tau(0,i) - X_k_tau(0,i);
            if (abs(EF_tau(i)) < error_zero_tolerance_tau)
                EF_tau(i) = 0;
        }
    
        diff_EF_tau = (EF_tau - EF_last_tau)/t;
        inter_EF_tau = ((EF_tau - EF_last_tau)/2)*t;
        EF_last_tau = EF_tau;

        delta = (P_gain_tau*EF_tau) + (I_gain_tau*inter_EF_tau) + (D_gain_tau*diff_EF_tau);
        count = count + 1;
        
        // adjusting Dq and DDq at zero velocity
        
        for (unsigned int i=0;i<dof;i++)
        {
            if (abs(Dq6(i))<0.02 )
            {
                X_k(1,i) = Dq6(i);
                X_k(2,i) = 0;
            }
        }
        
        // Inputing the filtered values
        
        for (int idxJnt = 0;idxJnt<dof;idxJnt++)
        {
            kf_Dq(iStep, idxJnt)    = X_k(1,idxJnt);
            kf_DDq(iStep, idxJnt)   = X_k(2,idxJnt);
            kf_eff(iStep, idxJnt)   = X_k_tau(0,idxJnt);
        }
}

//     tsupport::basic::print_yellow("We are using Unfiltered value \n ");
//      qf = org_q; Dqf = org_Dq; DDqf = org_DDq; efff = org_eff;
     
//     tsupport::basic::print_yellow("We are using Band Pass Filtered value \n ");
//      qf = bpf_q; Dqf = bpf_Dq; DDqf = bpf_DDq; efff = bpf_eff;

//     tsupport::basic::print_yellow("We are using Kalman Filtered value \n ");
//      qf = bpf_q; Dqf = kf_Dq; DDqf = kf_DDq; efff = kf_eff;
        
    tsupport::basic::print_yellow("We are using ZPF + BPF value \n ");
     qf = bpf_q; Dqf = zpf_Dq; DDqf = bpf_DDq; efff = zpf_eff;
     
//////////////////////////////
//                          //                          
//                          // 
// terrin addition end      //
//                          //                          
//                          //  
//////////////////////////////
    
      size_row_ = qf.rows();
      
      if ( verbose_ )
        ROS_INFO("Writing %d rows and %zu columns, from row = %d and col = %d", size_row_, qf.cols(), st_row_, 0 );
      
      
      qf_full.block( st_row_,0,size_row_,qf.cols() )      = qf;
      Dqf_full.block( st_row_,0,size_row_,Dqf.cols() )    = Dqf;
      DDqf_full.block( st_row_,0,size_row_,DDqf.cols() )  = DDqf;
      efff_full.block( st_row_,0,size_row_,efff.cols() )  = efff;
      
      st_row_ += qf.rows();
    }
    
    std::mt19937 eng{ std::random_device{}()};
    Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(qf_full.rows(), 0, qf_full.rows());
    
    
    std::shuffle(indices.data(), indices.data() + qf_full.rows(),eng);
    
    qf_full= indices.asPermutation() * qf_full;
    Dqf_full= indices.asPermutation() * Dqf_full;
    DDqf_full= indices.asPermutation() * DDqf_full;
    efff_full= indices.asPermutation() * efff_full;
    
    unsigned int identification_part = std::min((unsigned int)1e5,((unsigned int)qf_full.rows())/2);
    
    
    // IDENTIFICATION 
    Eigen::MatrixXd qf_ident   = qf_full  .topRows(identification_part);
    Eigen::MatrixXd Dqf_ident  = Dqf_full .topRows(identification_part);
    Eigen::MatrixXd DDqf_ident = DDqf_full.topRows(identification_part);
    Eigen::MatrixXd efff_ident = efff_full.topRows(identification_part);
    
    ROS_DEBUG("Waits for parameters estimation...");
    Eigen::MatrixXd PhiR  = m_estimator->getTrajectoryRegressor(qf_ident, Dqf_ident, DDqf_ident);
    Eigen::VectorXd PiR   = m_estimator->getEstimatedParameters(efff_ident);
    
    ROS_DEBUG_STREAM("Parameters:\n"<<PiR);
    
    Eigen::MatrixXd Phi  = m_estimator->getTrajectoryFullRegressor(qf_ident, Dqf_ident, DDqf_ident);
    Eigen::VectorXd Pi    = m_estimator->getFullEstimatedParameters();
    ROS_DEBUG_STREAM("Full Parameters:\n"<<Pi);
    Eigen::VectorXd Tau_ident   = PhiR * PiR;
    Eigen::VectorXd Tau_ident_full   = Phi * Pi;
    ROS_DEBUG("error full base = %f",(Tau_ident-Tau_ident_full).norm());
    
    Eigen::Map<Eigen::MatrixXd> map_tau_ident(Tau_ident.data(), efff_ident.cols(),efff_ident.rows());
    Eigen::MatrixXd efff_ident_model=map_tau_ident.transpose();
    
    
    // VALIDATION
    Eigen::MatrixXd qf_vali    = qf_full.  bottomRows(qf_full.rows()-identification_part);
    Eigen::MatrixXd Dqf_vali   = Dqf_full. bottomRows(qf_full.rows()-identification_part);
    Eigen::MatrixXd DDqf_vali  = DDqf_full.bottomRows(qf_full.rows()-identification_part);
    Eigen::MatrixXd efff_vali  = efff_full.bottomRows(qf_full.rows()-identification_part);
    
    Eigen::MatrixXd PhiR_vali  = m_estimator->getTrajectoryRegressor(qf_vali, Dqf_vali, DDqf_vali);
    Eigen::VectorXd Tau_vali   = PhiR_vali * PiR;
    
    Eigen::Map<Eigen::MatrixXd> map_tau_vali(Tau_vali.data(), efff_vali.cols(),efff_vali.rows());
    Eigen::MatrixXd efff_vali_model=map_tau_vali.transpose();
    
    rosdyn_identification_msgs::TorqueFitting ident;
    rosdyn_identification_msgs::TorqueFitting vali;
    
    for ( std::size_t iJnt=0; iJnt<number_of_joint_from_xml_; iJnt++ )    
    {
      Eigen::VectorXd error= (efff_ident-efff_ident_model).col(iJnt);
      double rmse=eigen_utils::standard_deviation( error );
      double corr=eigen_utils::correlation( efff_ident.col(iJnt),efff_ident_model.col(iJnt) );
      ident.joint_names.push_back(controller_joint_names_.at(iJnt));
      ident.rms_error.push_back(rmse);
      ident.real_model_correlation.push_back(corr);
      
      error= (efff_vali-efff_vali_model).col(iJnt);
      rmse=eigen_utils::standard_deviation( error );
      corr=eigen_utils::correlation( efff_vali.col(iJnt), efff_vali_model.col(iJnt) );
      
      vali.joint_names.push_back(controller_joint_names_.at(iJnt));
      vali.rms_error.push_back(rmse);
      vali.real_model_correlation.push_back(corr);
    }
    m_result_.identification=ident;
    m_result_.validation=vali;
    
    m_meto_par_estim_as->publishFeedback(m_feedback_);
   ros::Duration(0.5).sleep();
   
    ROS_INFO("Estimation complete!");
    m_result_.status = rosdyn_identification_msgs::MetoParEstimResult::SUCCESSFUL;
    m_meto_par_estim_as->setSucceeded(m_result_);
    return;
  }


  void MetoParEstimInterfaceNodelet::main()
  {
    ROS_INFO("Starting THREAD MetoParEstimInterfaceNodelet");
     
    m_namespace = "meto_cfg";
    
    while( (ros::ok()) && (!m_stop) )
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
   
    ROS_INFO("End of MetoParEstimInterfaceNodelet");
    return;
  };


bool MetoParEstimInterfaceNodelet::saveXmlCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  if (!m_estimator)
    return false;
  
  return m_estimator->saveParXml();
  
}

}
