/*
 * StateMonitor.cpp
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#include "Low_level/StateMonitor.h"

StateMonitor::StateMonitor(Robot *robotModel, std::string nodeName) :
Node(nodeName), robotModel(robotModel)
{
    
    init_cmd();
    
    lowState_sub = this->create_subscription<unitree_go::msg::LowState>(
        joint_state_topic, 5, std::bind(&StateMonitor::stateUpdateCallback, this, std::placeholders::_1));
    params_sub = this->create_subscription<go2_gait_planner::msg::ParamsSet>(
        params_topic, 10, std::bind(&StateMonitor::paramsCallback, this, std::placeholders::_1));

    joints_sub = this->create_subscription<go2_gait_planner::msg::JointsSet>(
        joints_topic, 10, std::bind(&StateMonitor::jointsCallback, this, std::placeholders::_1));

    lowCmd_pub = this->create_publisher<unitree_go::msg::LowCmd>(joint_cmd_topic, 10);
    cmdPubTimer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&StateMonitor::publishLowCmd, this));

    qTarg = robotModel->getAngles();
    // Initialize Integral gains
    // KIs = Eigen::MatrixXd::Identity(12, 12);
    // KIs = 0.01 * KIs;

    /* std::ofstream imuFile;
    std::cout << "Gets here" << std::endl;
    imuFile.open(imuFileName);
    imuFile.close(); */
}

void StateMonitor::init_cmd()
{
    
    lowCmdMsg.head = {254, 239};
    lowCmdMsg.level_flag = 255;
    lowCmdMsg.gpio = 0;

    for (int i = 0; i < 20; i++)
    {
        lowCmdMsg.motor_cmd[i].mode = 0x01; //Set toque mode, 0x00 is passive mode
        lowCmdMsg.motor_cmd[i].q = PosStopF;
        lowCmdMsg.motor_cmd[i].kp = 0;
        lowCmdMsg.motor_cmd[i].dq = VelStopF;
        lowCmdMsg.motor_cmd[i].kd = 0;
        lowCmdMsg.motor_cmd[i].tau = 0;
    }
}

void StateMonitor::publishLowCmd()
{
    // std::cout << "Hi from StateMonitor" << std::endl;
    ++curTime;
    phaseIncr();
    
    /* std::ofstream imuFile;
    imuFile.open("positions" + std::to_string(startTime) + imuFileNameExt, std::ios_base::app);
    // std::cout << phase << std::endl; */
    /* for (int leg = 0; leg < LEG_NUM; ++leg)
    {
        auto pos = robotModel->legs[leg]->getPosition();
        for (int joint = 0; joint < JOINT_NUM; ++joint)
        {
            std::cout << pos[joint] << " ";
            // imuFile << pos[joint] << "\t";
        }
    }
    
    std::cout << std::endl; */
    /*
    imuFile << std::endl;
    imuFile.close(); */
    
    if (paramsNotSet)
        return;
    Eigen::Vector<double, 12> angles((double*) &(robotModel->getAngles()[0]));

    get_crc(lowCmdMsg); //Check motor cmd crc

    
        

    // limitTaus();
    // for (int i = 0; i < JOINT_NUM * LEG_NUM; i++)
        // lowCmdMsg.motor_cmd[i].tau = cntrlTau[i];
    for (int i = 0; i < JOINT_NUM * LEG_NUM; i++)
        lowCmdMsg.motor_cmd[i].q = qTarg[i];

    if (!paramsNotSet)
        lowCmd_pub->publish(lowCmdMsg);
}

void StateMonitor::phaseIncr()
{
    if (phase == MAX_PHASE_VALUE - 1)
        phase = 0;
    else
        ++phase;
}
 
void StateMonitor::stateUpdateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
    //std::cout << "Updating robot state" << std::endl;
    std::vector<double> jointAngles;
    std::vector<double> jointVels;
    std::vector<double> jointTaus;
    std::vector<double> jointKps;
    std::vector<double> jointKds;
    for (int leg = 0; leg < LEG_NUM; leg++)
    {
        int curLegPos = JOINT_NUM * leg;
        for (int joint = 0; joint < JOINT_NUM; joint++)
        {   
            double estAngle;
            try {
                estAngle = robotModel->legs[leg]->joints[joint]->enforceLim(msg->motor_state[curLegPos + joint].q);
            }
            catch (std::exception &e)
            {
                return;
            }
            jointAngles.push_back(estAngle);
            jointVels.push_back(msg->motor_state[curLegPos + joint].dq);
            jointTaus.push_back(msg->motor_state[curLegPos + joint].tau_est);
            // std::cout << jointAngles[i] << " ";
        }
    }
    // std::cout << std::endl;
    robotModel->setAngles(jointAngles);
    robotModel->setVels(jointVels);
    robotModel->setTaus(jointTaus);

    std::vector<float> quat({
                            msg->imu_state.quaternion[0],
                            msg->imu_state.quaternion[1],
                            msg->imu_state.quaternion[2],
                            msg->imu_state.quaternion[3]
                            });
                            
    auto eulerAngles = quatToEuler(quat);

    robotModel->setOrientation(
                               eulerAngles[0],
                               eulerAngles[1],
                               eulerAngles[2]
    );

    if (writeFile)
        writeToFile();
    
}

void StateMonitor::writeToFile()
{
    if ((curTime - startTime) > simTime)
        writeFile = false;

    int curLegPos = legType * JOINT_NUM;

    std::ofstream imuFile;
    imuFile.open(imuFileName + std::to_string(startTime) + imuFileNameExt, std::ios_base::app);

    auto jointAngles = robotModel->getAngles();
    auto jointVels = robotModel->getVels();
    auto jointTaus = robotModel->getTaus();
    auto orient = robotModel->getOrientation();

    for (int i = 0; i < 3; ++i)
    {
        imuFile << orient[i] << "\t";
    }

    // imuFile << (curTime - startTime) << " ";
    imuFile << (curTime - startTime);

    /* for (int i = 0; i < JOINT_NUM; ++i)
    {
        imuFile << lowCmdMsg.motor_cmd[curLegPos + i].q << " ";
        imuFile << lowCmdMsg.motor_cmd[curLegPos + i].dq << " ";
        imuFile << lowCmdMsg.motor_cmd[curLegPos + i].tau << " ";
        imuFile << lowCmdMsg.motor_cmd[curLegPos + i].kp << " ";
        imuFile << lowCmdMsg.motor_cmd[curLegPos + i].kd << " ";
        imuFile << jointAngles[curLegPos + i] << " ";
        imuFile << jointVels[curLegPos + i] << " ";
        imuFile << jointTaus[curLegPos + i] << " ";

    } */
    

    /* auto eulerAngles = robotModel->getOrientation();

    for (int i = 0; i < 3; ++i)
    {
        // std::cout << eulerAngles[i] << " ";
        imuFile << eulerAngles[i] << " ";
    }
    imuFile << cntrl1 << " ";
    imuFile << cntrl2 << " ";
    imuFile << orientTarg[1] << " ";
    imuFile << cntrlTau[cntrl1] << " ";
    imuFile << cntrlTau[cntrl2] << " ";
    imuFile << orientI_err[1] << " ";
    imuFile << qI_err[1] << " "; */
    // std::cout << std::endl;

    imuFile << std::endl;
    imuFile.close();
}

void StateMonitor::paramsCallback(go2_gait_planner::msg::ParamsSet::SharedPtr paramsMsg)
{
    std::vector<double> jointAngles = robotModel->getAngles();
    std::vector<double> jointVels = robotModel->getVels();
    std::vector<double> jointTaus = robotModel->getTaus();
    std::vector<double> jointKps = robotModel->getKps();
    std::vector<double> jointKds = robotModel->getKds();

    /* std::vector<double> jointAngles({
        -0.13000117242336273,
        1.2262111902236938,
        -2.724439859390259,
        0.12999489903450012,
        1.226211428642273,
        -2.724479913711548,
        -0.48181259632110596,
        1.2460269927978516,
        -2.7230396270751953,
        0.4821029305458069,
        1.245980143547058,
        -2.7229092121124268
});
    robotModel->setAngles(jointAngles); */

    /* std::vector<double> jointAngles({
        -0.136414, 1.22295, -2.72235,
        0.136565, 1.22285, -2.72225,
        -0.480054, 1.68357, -2.72429,
        0.479981, 1.68311, -2.72434
    });
    robotModel->setAngles(jointAngles); */

    for (int i = 0; i < JOINT_NUM * LEG_NUM; i++)
    {
        //if(paramsMsg->mask[i])jointAngles[i] = paramsMsg->q[i];
        //if(paramsMsg->mask[i])jointVels[i] = paramsMsg->dq[i];
       // if(paramsMsg->mask[i])jointTaus[i] = paramsMsg->tau[i];
        if(paramsMsg->mask[i])jointKps[i] = paramsMsg->kp[i];
        if(paramsMsg->mask[i])jointKds[i] = paramsMsg->kd[i]; 
    }

    robotModel->setKps(jointKps);
    robotModel->setKds(jointKds);
    for (int i = 0; i < JOINT_NUM * LEG_NUM; i++)
    {
        // lowCmdMsg.motor_cmd[i].q = jointAngles[i];
        lowCmdMsg.motor_cmd[i].dq = 0;
        lowCmdMsg.motor_cmd[i].tau = 0;
        lowCmdMsg.motor_cmd[i].kp = jointKps[i];
        lowCmdMsg.motor_cmd[i].kd = jointKds[i];
    }

    /* int curLegPos = JOINT_NUM * straightLegType;
    for (int i = curLegPos, j = 0; i < curLegPos + JOINT_NUM; i++, j++) */
    for(int i = 0; i < JOINT_NUM * LEG_NUM; i++)
        qTarg[i] = jointAngles[i];
    
    //std::cout << "Target Angles " << qTarg << std::endl;
    std::cout << "written params" << std::endl;
    paramsNotSet = false;
    //publishLowCmd();
}

void StateMonitor::jointsCallback(go2_gait_planner::msg::JointsSet::SharedPtr jointsMsg)
{
    startTime = curTime;
    // writeFile = true;
    legType = jointsMsg->leg_type;
    int curLegPos = legType * JOINT_NUM;

    std::lock_guard<std::mutex> guard(guard_mutex);

    for (int i = 0; i < JOINT_NUM; ++i)
    {
        // lowCmdMsg.motor_cmd[curLegPos + i].q = jointsMsg->q[i];
        qTarg[curLegPos + i] = jointsMsg->q[i];
        lowCmdMsg.motor_cmd[curLegPos + i].dq = jointsMsg->dq[i];
        lowCmdMsg.motor_cmd[curLegPos + i].tau = jointsMsg->tau[i];
    }
    std::cout << "gets here" << std::endl;
}


std::vector<float> StateMonitor::quatToEuler(std::vector<float> quat)
{
    if (quat.size() != 4)
        throw std::invalid_argument("Invalid number of quaternions");

    std::vector<float> eulerAngles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
    double cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
    eulerAngles.push_back(std::atan2(sinr_cosp, cosr_cosp)) ;

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (quat[0] * quat[2] - quat[1] * quat[3]));
    double cosp = std::sqrt(1 - 2 * (quat[0] * quat[2] - quat[1] * quat[3]));
    eulerAngles.push_back(2 * std::atan2(sinp, cosp) - M_PI / 2);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
    double cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
    eulerAngles.push_back(std::atan2(siny_cosp, cosy_cosp));

    return eulerAngles;
}


uint32_t StateMonitor::crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}