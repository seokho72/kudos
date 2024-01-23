#include "CKubot.h"
// #include "kinematics.h"

using namespace std;

CKubot::CKubot()
{
}

CKubot::CKubot(const CKubot &orig)
{
}

CKubot::~CKubot()
{
}

void CKubot::abc()
{
    printf("@@@@@@@@@@과연 결과는?@@@@@@@@@@\n");
}

int CKubot::sign(double a)
{
    if (a >= 0)
        return 1;
    return -1;
}

Matrix3d CKubot::transposeMat(Matrix3d mat)
{
    Matrix3d tmp_m;

    tmp_m << mat(0, 0), mat(1, 0), mat(2, 0),
        mat(0, 1), mat(1, 1), mat(2, 1),
        mat(0, 2), mat(1, 2), mat(2, 2);

    return tmp_m;
}

MatrixXd CKubot::rotMatX(double q)
{
    MatrixXd tmp_m(3, 3);
    tmp_m << 1, 0, 0,
        0, cos(q), -sin(q),
        0, sin(q), cos(q);
    return tmp_m;
}

MatrixXd CKubot::rotMatY(double q)
{
    MatrixXd tmp_m(3, 3);
    tmp_m << cos(q), 0, sin(q),
        0, 1, 0,
        -sin(q), 0, cos(q);
    return tmp_m;
}

MatrixXd CKubot::rotMatZ(double q)
{
    MatrixXd tmp_m(3, 3);
    tmp_m << cos(q), -sin(q), 0,
        sin(q), cos(q), 0,
        0, 0, 1;
    return tmp_m;
}

MatrixXd CKubot::getTransformI0()
{
    //  Frame 0 to Frame I

    MatrixXd tmp_m(4, 4);

    // Alternative representations
    // 1. tmp_m = MatrixXd::Identity(4,4);
    // 2. tmp_m(m-1,n-1) = m,n element where m,n>=1;

    tmp_m << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0.34,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::getTransform6E()
{
    //  Frame E to Frame 6

    MatrixXd tmp_m(4, 4);

    tmp_m << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -0.04,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToTransform01(VectorXd q)
{
    //  Frame 1 to Frame 0
    // q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]

    MatrixXd tmp_m(4, 4);
    double qq = q(0);

    tmp_m << cos(qq), -sin(qq), 0, 0,
        sin(qq), cos(qq), 0, 0.05,
        0, 0, 1, -0.10464,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToTransform01_R(VectorXd q)
{
    //  Frame 1 to Frame 0
    // q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]

    MatrixXd tmp_m(4, 4);
    double qq = q(0);

    tmp_m << cos(qq), -sin(qq), 0, 0,
        sin(qq), cos(qq), 0, -0.05,
        0, 0, 1, -0.10464,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToTransform12(VectorXd q)
{
    //  Frame 2 to Frame 1

    MatrixXd tmp_m(4, 4);
    double qq = q(1);

    tmp_m << 1, 0, 0, 0,
        0, cos(qq), -sin(qq), 0,
        0, sin(qq), cos(qq), 0,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToTransform23(VectorXd q)
{
    //  Frame 3 to Frame 2
    MatrixXd tmp_m(4, 4);
    double qq = q(2);

    tmp_m << cos(qq), 0, sin(qq), 0,
        0, 1, 0, 0,
        -sin(qq), 0, cos(qq), 0,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToTransform34(VectorXd q)
{
    //  Frame 4 to Frame 3
    MatrixXd tmp_m(4, 4);
    double qq = q(3);

    tmp_m << cos(qq), 0, sin(qq), 0,
        0, 1, 0, 0,
        -sin(qq), 0, cos(qq), -0.138,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToTransform45(VectorXd q)
{
    //  Frame 5 to Frame 4
    MatrixXd tmp_m(4, 4);
    double qq = q(4);

    tmp_m << cos(qq), 0, sin(qq), 0,
        0, 1, 0, 0,
        -sin(qq), 0, cos(qq), -0.143,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToTransform56(VectorXd q)
{
    //  Frame 5 to Frame 4
    MatrixXd tmp_m(4, 4);
    double qq = q(5);

    tmp_m << 1, 0, 0, 0,
        0, cos(qq), -sin(qq), 0,
        0, sin(qq), cos(qq), 0,
        0, 0, 0, 1;

    return tmp_m;
}

MatrixXd CKubot::jointToPosition(VectorXd q, double X, double Y, double Z)
{
    //  Extract position vector(3x1)

    VectorXd tmp_vL = VectorXd::Zero(3);
    VectorXd tmp_vR = VectorXd::Zero(3);
    VectorXd qL(6);
    qL << q.block(0, 0, 6, 1);
    VectorXd qR(6);
    qR << q.block(6, 0, 6, 1);
    VectorXd tmp_v = VectorXd::Zero(6);
    MatrixXd tmp_m(4, 4);
    MatrixXd tmg_b(4, 4);
    tmg_b << 1, 0, 0, X,
        0, 1, 0, Y,
        0, 0, 1, Z,
        0, 0, 0, 1;

    int left = 0;
    int right = 1;
    for (int leg = 0; leg < 2; leg++)
    {

        if (leg == 0)
        {
            tmp_m = tmg_b *
                    jointToTransform01(qL) *
                    jointToTransform12(qL) *
                    jointToTransform23(qL) *
                    jointToTransform34(qL) *
                    jointToTransform45(qL) *
                    jointToTransform56(qL) *
                    getTransform6E();

            //    tmp_v = tmp_m.block(0,3,3,1);

            tmp_vL(0) = tmp_m(0, 3);
            tmp_vL(1) = tmp_m(1, 3);
            tmp_vL(2) = tmp_m(2, 3);

            // std::cout << "\nFK_jointToPosition_L:\n" << tmp_vL << std::endl;
        }

        else if (leg == 1)
        {
            tmp_m = tmg_b *
                    jointToTransform01_R(qR) *
                    jointToTransform12(qR) *
                    jointToTransform23(qR) *
                    jointToTransform34(qR) *
                    jointToTransform45(qR) *
                    jointToTransform56(qR) *
                    getTransform6E();

            //    tmp_v = tmp_m.block(0,3,3,1);

            tmp_vR(0) = tmp_m(0, 3);
            tmp_vR(1) = tmp_m(1, 3);
            tmp_vR(2) = tmp_m(2, 3);

            // std::cout << "FK_jointToPosition_R:\n" << tmp_vR << std::endl;
        }
    }
    tmp_v << tmp_vL, tmp_vR;

    // std::cout << "FK_jointToPosition_tmp_vL+tmp_vR:\n" << tmp_vL << "\n \n" << tmp_vR << std::endl;

    // std::cout << "FK_jointToPosition_tmp_v:\n" << tmp_v  << std::endl;

    return tmp_v; // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;
}
MatrixXd CKubot::jointToPosition(VectorXd q)
{
    //  Extract position vector(3x1)

    VectorXd tmp_vL = VectorXd::Zero(3);
    VectorXd tmp_vR = VectorXd::Zero(3);
    VectorXd qL(6);
    qL << q.block(0, 0, 6, 1);
    VectorXd qR(6);
    qR << q.block(6, 0, 6, 1);
    VectorXd tmp_v = VectorXd::Zero(6);
    MatrixXd tmp_m(4, 4);

    int left = 0;
    int right = 1;
    for (int leg = 0; leg < 2; leg++)
    {

        if (leg == 0)
        {
            tmp_m = getTransformI0() *
                    jointToTransform01(qL) *
                    jointToTransform12(qL) *
                    jointToTransform23(qL) *
                    jointToTransform34(qL) *
                    jointToTransform45(qL) *
                    jointToTransform56(qL) *
                    getTransform6E();

            //    tmp_v = tmp_m.block(0,3,3,1);

            tmp_vL(0) = tmp_m(0, 3);
            tmp_vL(1) = tmp_m(1, 3);
            tmp_vL(2) = tmp_m(2, 3);

            // std::cout << "\nFK_jointToPosition_L:\n" << tmp_vL << std::endl;
        }

        else if (leg == 1)
        {
            tmp_m = getTransformI0() *
                    jointToTransform01_R(qR) *
                    jointToTransform12(qR) *
                    jointToTransform23(qR) *
                    jointToTransform34(qR) *
                    jointToTransform45(qR) *
                    jointToTransform56(qR) *
                    getTransform6E();

            //    tmp_v = tmp_m.block(0,3,3,1);

            tmp_vR(0) = tmp_m(0, 3);
            tmp_vR(1) = tmp_m(1, 3);
            tmp_vR(2) = tmp_m(2, 3);

            // std::cout << "FK_jointToPosition_R:\n" << tmp_vR << std::endl;
        }
    }
    tmp_v << tmp_vL, tmp_vR;

    // std::cout << "FK_jointToPosition_tmp_vL+tmp_vR:\n" << tmp_vL << "\n \n" << tmp_vR << std::endl;

    // std::cout << "FK_jointToPosition_tmp_v:\n" << tmp_v  << std::endl;

    return tmp_v; // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;
}

MatrixXd CKubot::jointToPosition_R(VectorXd q)
{
    //  Extract position vector(3x1)

    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4, 4);

    tmp_m = getTransformI0() *
            jointToTransform01_R(q) *
            jointToTransform12(q) *
            jointToTransform23(q) *
            jointToTransform34(q) *
            jointToTransform45(q) *
            jointToTransform56(q) *
            getTransform6E();

    //    tmp_v = tmp_m.block(0,3,3,1);

    tmp_v(0) = tmp_m(0, 3);
    tmp_v(1) = tmp_m(1, 3);
    tmp_v(2) = tmp_m(2, 3);

    return tmp_v;
    // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;
}

Matrix3d CKubot::jointToRotMat(VectorXd q)
{
    Matrix3d tmp_m;
    MatrixXd T_IE(4, 4);

    T_IE = getTransformI0() *
           jointToTransform01(q) *
           jointToTransform12(q) *
           jointToTransform23(q) *
           jointToTransform34(q) *
           jointToTransform45(q) *
           jointToTransform56(q) *
           getTransform6E();

    tmp_m = T_IE.block(0, 0, 3, 3);

    return tmp_m;
}

Matrix3d CKubot::jointToRotMat_R(VectorXd q)
{
    Matrix3d tmp_m;
    MatrixXd T_IE(4, 4);

    T_IE = getTransformI0() *
           jointToTransform01_R(q) *
           jointToTransform12(q) *
           jointToTransform23(q) *
           jointToTransform34(q) *
           jointToTransform45(q) *
           jointToTransform56(q) *
           getTransform6E();

    tmp_m = T_IE.block(0, 0, 3, 3);

    return tmp_m;
}

VectorXd CKubot::rotToEuler(MatrixXd rotMat)
{
    // ZYX Euler Angle - yaw-pitch-roll
    Vector3d tmp_v;

    tmp_v(0) = atan2(rotMat(1, 0), rotMat(0, 0));
    tmp_v(1) = atan2(-rotMat(2, 0), sqrt(pow(rotMat(2, 1), 2) + pow(rotMat(2, 2), 2)));
    tmp_v(2) = atan2(rotMat(2, 1), rotMat(2, 2));

    // std::cout << tmp_v << endl;

    return tmp_v;
}

VectorXd CKubot::Geometric_IK_L(VectorXd GB_PosOri, VectorXd GF_PosOri)
{                   // Global to Base,Foot position and orientaion
    double A, B, C; // A = thigh, B = calf
    double alpha, c5;
    VectorXd BH(3), p1(3), p2(3), p7(3), pf(3), FA(3), r(3);
    VectorXd q(6);
    MatrixXd R1(3, 3), R7(3, 3), RH(3, 3);

    MatrixXd GB = MatrixXd::Identity(4, 4);
    MatrixXd GF = MatrixXd::Identity(4, 4);

    GB.block(0, 0, 3, 3) = rotMatZ(GB_PosOri(5)) * rotMatY(GB_PosOri(4)) * rotMatX(GB_PosOri(3));
    GB.block(0, 3, 3, 1) << GB_PosOri(0), GB_PosOri(1), GB_PosOri(2);

    GF.block(0, 0, 3, 3) = rotMatZ(GF_PosOri(5)) * rotMatY(GF_PosOri(4)) * rotMatX(GF_PosOri(3));
    GF.block(0, 3, 3, 1) << GF_PosOri(0), GF_PosOri(1), GF_PosOri(2);

    BH << 0, 0.05, -0.105; // Base to Hip

    R1 = GB.block(0, 0, 3, 3);
    R7 = GF.block(0, 0, 3, 3);
    pf = GF.block(0, 3, 3, 1); // Foot position
    FA << 0, 0, 0.04;          // Foot to Ankle
    p7 = pf + FA;

    A = 0.138;
    B = 0.143;

    p1 << GB(0, 3), GB(1, 3), GB(2, 3);
    p2 = p1 + R1 * BH;

    r = R7.transpose() * (p2 - p7);
    C = r.norm();
    c5 = cos((A * A + B * B - C * C) / (2 * A * B));

    if (c5 >= 1)
        q(3) = 0.0;
    else if (c5 <= -1)
        q(3) = PI;
    else
        q(3) = -acos((A * A + B * B - C * C) / (2 * A * B)) + PI;

    alpha = asin((A * sin(PI - q(3))) / C);

    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2)) * sqrt(r(1) * r(1) + r(2) * r(2))) - alpha;

    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3) - q(4)); // Rotation Hip

    q(0) = atan2(-RH(0, 1), RH(1, 1));
    q(1) = atan2(RH(2, 1), -sin(q(0)) * RH(0, 1) + RH(1, 1) * cos(q(0)));
    q(2) = atan2(-RH(2, 0), RH(2, 2));

    // std::cout << "\nIK_joint_L:\n" << q << std::endl;

    return q;
}

VectorXd CKubot::Geometric_IK_R(VectorXd GB_cfg, VectorXd GF_cfg)
{
    double A, B, C; // thigh, calf
    double alpha, c5;
    VectorXd PH(3);
    VectorXd p1(3), p2(3), p7(3), pf(3), fa(3), r(3);
    VectorXd q(6);
    MatrixXd R1(3, 3), R7(3, 3), RH(3, 3);

    MatrixXd GB = MatrixXd::Identity(4, 4);
    MatrixXd GF = MatrixXd::Identity(4, 4);

    GB.block(0, 0, 3, 3) = rotMatZ(GB_cfg(5)) * rotMatY(GB_cfg(4)) * rotMatX(GB_cfg(3));
    GB.block(0, 3, 3, 1) << GB_cfg(0), GB_cfg(1), GB_cfg(2);

    GF.block(0, 0, 3, 3) = rotMatZ(GF_cfg(5)) * rotMatY(GF_cfg(4)) * rotMatX(GF_cfg(3));
    GF.block(0, 3, 3, 1) << GF_cfg(0), GF_cfg(1), GF_cfg(2);

    PH << 0, -0.05, -0.105;

    R1 = GB.block(0, 0, 3, 3);
    R7 = GF.block(0, 0, 3, 3);
    pf = GF.block(0, 3, 3, 1);
    fa << 0, 0, 0.04;
    p7 = pf + fa;

    A = 0.138;
    B = 0.143;

    p1 << GB(0, 3), GB(1, 3), GB(2, 3);
    p2 = p1 + R1 * PH;

    r = R7.transpose() * (p2 - p7);
    C = r.norm();
    c5 = cos((A * A + B * B - C * C) / (2 * A * B));

    if (c5 >= 1)
        q(3) = 0.0;

    else if (c5 <= -1)
        q(3) = PI;

    else
        q(3) = -acos((A * A + B * B - C * C) / (2 * A * B)) + PI;

    alpha = asin((A * sin(PI - q(3))) / C);

    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2)) * sqrt(r(1) * r(1) + r(2) * r(2))) - alpha;

    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3) - q(4));

    q(0) = atan2(-RH(0, 1), RH(1, 1));
    q(1) = atan2(RH(2, 1), -sin(q(0)) * RH(0, 1) + RH(1, 1) * cos(q(0)));
    q(2) = atan2(-RH(2, 0), RH(2, 2));

    // std::cout << "IK_joint_R:\n" << q << std::endl;

    return q;
}

void CKubot::IK()
{
    VectorXd GB_PosOri(6);
    VectorXd GF_PosOri_L(6), GF_PosOri_R(6);
    VectorXd GF_cfg_R(6);

    GB_PosOri << 0, 0, 0.5, 0, 0, 0;
    GF_PosOri_L << -0.00353553, 0.05, 0.156664, 0, 0, 0;
    GF_PosOri_R << -0.00353553, -0.05, 0.156664, 0, 0, 0;

    Geometric_IK_L(GB_PosOri, GF_PosOri_L);
    Geometric_IK_R(GB_PosOri, GF_PosOri_R);

    // GB_cfg << 0,0,0.5,0,0,0;
    // GF_cfg_L << 0,0.05,0.2,0,0,0;
    // GF_cfg_R << 0,-0.05,0.2,0,0,0;
    // Geometric_IK_L(GB_cfg, GF_cfg_L);
    // Geometric_IK_R(GB_cfg, GF_cfg_R);

    //    tmp_v = tmp_m.block(0,3,3,1);
}

double CKubot::cosWave(double amp, double period, double time, double int_pos)
{
    return (amp / 2) * (1 - cos(PI / period * time)) + int_pos;
}

void CKubot::HomePose(float return_time)
{
    /* HomePose, Return to HomePose(Humanoid Robots)
     * input : time while the robot acts Homepose posture,
     *
     * main processing:
     * 1. Homepose
     */
    static float re_time = 0;
    static float init_joint_angle[20];

    if (re_time == 0 && Move_current == false)
    {
        for (int j = 0; j < 20; j++)
        {
            init_joint_angle[j] = refAngle[j];
        }
    }

    if (re_time <= return_time && Move_current == false)
    {
        for (int j = 0; j < 20; j++)
        {
            refAngle[j] = cosWave(-init_joint_angle[j], return_time, re_time, init_joint_angle[j]); // 여기서 값을 다시 부여함 ㅇㅇ
                                                                                                    // -> init_joint_angle -> 0 까지(-init_joint_angle 만큼 이동하세요 명령)
        }
        re_time += tasktime;
    }
    else
    {
        re_time = 0;
        Move_current = true;
    }
} // base와 관절의 궤적제어가 동시에 들어가고있다.
//

void CKubot::walkingReady(float readytime)
{
    /* walkingReady
     * input : time while the robot acts ready posture,
     *
     * main processing:
     * 1. Walking Ready
     */

    static float time = 0;
    static float currentAngle[20]; // Now nDoF = 20

    if (time == 0 && Move_current == false)
    {
        for (int j = 0; j < 20; j++)
        {
            currentAngle[j] = refAngle[j];
        }
    }

    if (time < readytime && Move_current == false)
    {
        for (int j = 0; j < 20; j++)
        {
            refAngle[j] = cosWave(walkReadyAngle[j] - currentAngle[j], readytime, time, currentAngle[j]);
        }

        time += tasktime;
    }
    else
    {
        time = 0;
        Move_current = true;
    }
}

void CKubot::upperbodytest(float upperT)
{
    /* walkingReady
     * input : time while the robot acts ready posture,
     *
     * main processing:
     * 1. Walking Ready
     */

    static float upper_time = 0;
    static float uppperAngle[20]; // Now nDoF = 20
    static float uppertestAngle[20];

    VectorXd Base_pos(6);

    Base_pos(0) = 0;
    Base_pos(1) = 0;
    Base_pos(2) = 0.38;
    Base_pos(3) = 0;
    Base_pos(4) = 0;
    Base_pos(5) = 0;

    VectorXd init_LFoot(6), init_RFoot(6);
    init_LFoot << 0, 0.05, 0, 0, 0, 0;
    init_RFoot << 0, -0.05, 0, 0, 0, 0;

    VectorXd IK_walkreadyAngle(12);

    IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

    // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

    for (int j = 0; j < 12; j++)
    {

        uppperAngle[j] = IK_walkreadyAngle(j);
        refAngle[j] = uppperAngle[j];
        // printf(C_CYAN "The Robot's %d Joint for Walking Ready = %f\n" C_RESET, j, joint[j].walkReadyAngle * R2D);
    }

    uppertestAngle[12] = (-30 * D2R);  // L_shoulder_pitch_joint 플러스가 어깨 뒤로
    uppertestAngle[13] = (15 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
    uppertestAngle[14] = (-130 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽

    uppertestAngle[15] = (-30 * D2R);  // R_shoulder_pitch_joint
    uppertestAngle[16] = (-15 * D2R);  // R_elbow_roll_joint
    uppertestAngle[17] = (-130 * D2R); // R_hand_pitch_joint

    uppertestAngle[18] = (0 * D2R); // Neck_yaw_joint
    uppertestAngle[19] = (0 * D2R); // head_pitch_joint

    if (upper_time == 0 && Move_current == false)
    {
        for (int j = 12; j < 20; j++)
        {
            uppperAngle[j] = refAngle[j];
        }
    }

    if (upper_time < upperT && Move_current == false)
    {
        for (int j = 12; j < 20; j++)
        {
            refAngle[j] = cosWave(uppertestAngle[j] - uppperAngle[j], upperT, upper_time, uppperAngle[j]);
        }

        upper_time += tasktime;
    }
    else
    {
        upper_time = 0;
        Move_current = true;
    }
}

void CKubot::setWalkingReadyPos(double init_x, double init_y, double init_z)
{
    /* setWalkingReadyPos, Set CoM pos for Walking Ready(Humanoid Robots)
     * input : Init Positions [X, Y, Z][unit:m] of CoM for walking ready,
     *
     * main processing:
     * 1. Setting CoM Position when walking Ready
     */
    VectorXd Base_pos(6);

    Base_pos(0) = init_x;
    Base_pos(1) = init_y;
    Base_pos(2) = init_z;
    Base_pos(3) = 0;
    Base_pos(4) = 0;
    Base_pos(5) = 0;

    VectorXd init_LFoot(6), init_RFoot(6);
    init_LFoot << 0, 0.05, 0, 0, 0, 0;
    init_RFoot << 0, -0.05, 0, 0, 0, 0;

    VectorXd IK_walkreadyAngle(12);

    IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

    // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle  << std::endl;

    for (int j = 0; j < 12; j++)
    {

        walkReadyAngle[j] = IK_walkreadyAngle(j);
        // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
    }

    // walkReadyAngle[12] = (30 *D2R);     // L_shoulder_pitch_joint 플러스가 어깨 뒤로
    // walkReadyAngle[13] = (15 *D2R);     // L_elbow_roll_joint 플러스가 좌우로 나란히
    // walkReadyAngle[14] = (-130 *D2R);     // L_hand_pitch_joint  플러스가 안으로 굽
    // walkReadyAngle[15] = (30  *D2R);     // R_shoulder_pitch_joint
    // walkReadyAngle[16] = (-15 *D2R);     // R_elbow_roll_joint
    // walkReadyAngle[17] = (-130 *D2R);     // R_hand_pitch_joint
    // walkReadyAngle[18] = (0 *D2R);     // Neck_yaw_joint
    // walkReadyAngle[19] = (0 *D2R);     // head_pitch_joint

    walkReadyAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
    walkReadyAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
    walkReadyAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
    walkReadyAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
    walkReadyAngle[16] = (0 * D2R);   // R_elbow_roll_joint
    walkReadyAngle[17] = (-85 * D2R); // R_hand_pitch_joint
    walkReadyAngle[18] = (0 * D2R);   // Neck_yaw_joint
    walkReadyAngle[19] = (55 * D2R);  // head_pitch_joint

    // std::cout << "walkReadyAngle == \n" << walkReadyAngle << std::endl;
}

void CKubot::initializeBkubot()
{

    /* initialize CRobot class's variable
     * initializeFTSensorValue();
     * initializeIMUSensorValue();
     * initializeSystemID(38.9410,555,zmp.observer.disturbanceExist); // kg, mm
     * initializeZmpControl(zmp.observer.disturbanceExist);
     */

    Base.current = VectorXd::Zero(6);
    Base.vel = VectorXd::Zero(6);
    Base.acc = VectorXd::Zero(6);
    Base.pre = VectorXd::Zero(6);
    Base.prevel = VectorXd::Zero(6);
    Base.preacc = VectorXd::Zero(6);

    Base.refpos = VectorXd::Zero(6);
    Base.refvel = VectorXd::Zero(6);
    Base.refacc = VectorXd::Zero(6);
    Base.refquat = VectorXd::Zero(4);
    Base.refquat(0) = 1;

    Base.prerefpos = VectorXd::Zero(6);
    Base.prerefvel = VectorXd::Zero(6);
    Base.prerefacc = VectorXd::Zero(6);

    LFoot.current = VectorXd::Zero(6);
    LFoot.vel = VectorXd::Zero(6);
    LFoot.acc = VectorXd::Zero(6);
    LFoot.pre = VectorXd::Zero(6);
    LFoot.prevel = VectorXd::Zero(6);
    LFoot.preacc = VectorXd::Zero(6);

    LFoot.refpos = VectorXd::Zero(6);
    LFoot.refvel = VectorXd::Zero(6);
    LFoot.refacc = VectorXd::Zero(6);
    LFoot.refquat = VectorXd::Zero(4);
    LFoot.refquat(0) = 1;
    LFoot.prerefpos = VectorXd::Zero(6);
    LFoot.prerefvel = VectorXd::Zero(6);
    LFoot.prerefacc = VectorXd::Zero(6);

    RFoot.current = VectorXd::Zero(6);
    RFoot.vel = VectorXd::Zero(6);
    RFoot.acc = VectorXd::Zero(6);
    RFoot.pre = VectorXd::Zero(6);
    RFoot.prevel = VectorXd::Zero(6);
    RFoot.preacc = VectorXd::Zero(6);

    RFoot.refpos = VectorXd::Zero(6);
    RFoot.refvel = VectorXd::Zero(6);
    RFoot.refacc = VectorXd::Zero(6);
    RFoot.refquat = VectorXd::Zero(4);
    RFoot.refquat(0) = 1;
    RFoot.prerefpos = VectorXd::Zero(6);
    RFoot.prerefvel = VectorXd::Zero(6);
    RFoot.prerefacc = VectorXd::Zero(6);
    // // Base, EndPoint Data init

    // initposition = false;

    walking.step.LFoot = MatrixXd::Zero(PatternElement, PatternPlan / 2);
    walking.step.RFoot = MatrixXd::Zero(PatternElement, PatternPlan / 2);
    walking.step.preLfoot = VectorXd::Zero(PatternElement);
    walking.step.preRfoot = VectorXd::Zero(PatternElement);
    walking.step.prefootCenter = VectorXd::Zero(PatternElement);
    walking.step.LastLfoot = VectorXd::Zero(PatternElement);
    walking.step.LastRfoot = VectorXd::Zero(PatternElement);
    walking.step.LastfootCenter = VectorXd::Zero(PatternElement);
    // // variable for Pattern plan

    walking.change_FB_walking_flag = false;
    walking.change_LR_walking_flag = false;
    walking.change_LTurn_flag = false;
    walking.change_RTurn_flag = false;
    walking.change_Turn_flag = false;

    walking.Ready = false;
    walking.walk = false;
    walking.SP = walking.DSP;
    walking.old_SP = walking.DSP;
    walking.SP_check = walking.DSP;
    walking.SP_oldcheck = walking.DSP;
    walking.SP_checking = 0;

    walking.ft_SP = walking.DSP;
    walking.old_ft_SP = walking.DSP;
    walking.walk_SP = walking.DSP;
    walking.old_walk_SP = walking.DSP;

    walking.NewfootPlan = false;

    walking.time.readytime = 0;
    walking.time.sec = 0;
    walking.time.wsec = 0;

    walking.step.walking_f_or_b = walking.step.WalkinginPlace;
    walking.step.FBstepSize = 0;
    walking.step.oldFBstepSize = 0;
    walking.step.walking_l_or_r = walking.step.WalkinginPlace;
    walking.step.LRstepSize = 0;
    walking.step.oldLRstepSize = 0;
    walking.step.walking_lT_or_rT = walking.step.WalkinginPlace;
    walking.step.TurnSize = 0;
    walking.step.oldTurnSize = 0;
    walking.step.walking_Turn = walking.step.TurnModeOff;
    walking.step.turningPoint = 0;
    zmp.previewControl.setcount = 0;
    // // Walking init

    init_t = 0;

    // mode = POSITIONCONTROLMODE;
    // Mode

    zmp.controller.ON = true;
    zmp.controller.OFF = false;
    zmp.controller.gainchangeflag = false;
    // ZMP control, observer

    setZmpPreview(1.5);
    // initializeQP();

    body_kine.lower.BASE_TO_LEG_Y = 0.05;                                                                                   // mm // 50.0 mm
    body_kine.lower.BASE_TO_LEG_Z = 104.64;                                                                                 // mm
    body_kine.lower.THIGH_LENGTH = 138.0;                                                                                   //
    body_kine.lower.CALF_LENGTH = 143.0;                                                                                    // mm
    body_kine.lower.ANKLE_LENGTH = 40;                                                                                      // mm
    body_kine.lower.LEG_LENGTH = body_kine.lower.THIGH_LENGTH + body_kine.lower.CALF_LENGTH + body_kine.lower.ANKLE_LENGTH; // mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
    body_kine.lower.LEG_SIDE_OFFSET = body_kine.lower.BASE_TO_LEG_Y;                                                        // BASE_TO_LEG_Y

    body_kine.lower.foot_x_length = 150; // mm
    body_kine.lower.foot_y_length = 100; // mm

    VectorXd initLfoot = VectorXd::Zero(PatternElement);
    VectorXd initRfoot = VectorXd::Zero(PatternElement);

    initLfoot(Y) = body_kine.lower.LEG_SIDE_OFFSET;
    initRfoot(Y) = -body_kine.lower.LEG_SIDE_OFFSET;

    globalPositionUpdate(initLfoot, initRfoot);
    lastGlobalStepPositionUpdate();
    walking.initialize = false;

    // checkCRobot();
}

void CKubot::setZmpPreview(double previewTime)
{

    /* setZmpPreview
     * input : preview time for ZMP preview control
     * output : previewTime --> setZmpPreviewTime [Crobot class's Function]
     *          Setting variable for ZMP preview control
     *
     * Before run function
     * [First, Run once Crobot class's Function initializeSystemID when starting program]
     */

    std::cout << "===== Setting ZMP preview control =====" << std::endl;

    setZmpPreviewTime(previewTime);

    double T = (double)tasktime;

    zmp.previewControl.A = MatrixXd::Zero(3, 3);
    zmp.previewControl.A << 1, T, T * T / 2.0, 0, 1, T, 0, 0, 1;

    zmp.previewControl.B = MatrixXd::Zero(3, 1);
    zmp.previewControl.B << T * T * T / 6.0, T * T / 2.0, T;

    zmp.previewControl.C = MatrixXd::Zero(1, 3);
    // zmp.previewControl.C << 1 ,0 ,-(systemID.robotH*0.34)/9.81;
    zmp.previewControl.C << 1, 0, -(0.38 * 1.15) / 9.81;

    zmp.previewControl.B_Tilde = MatrixXd::Zero(4, 1);
    zmp.previewControl.B_Tilde.block(0, 0, 1, 1) = zmp.previewControl.C * zmp.previewControl.B;
    zmp.previewControl.B_Tilde.block(1, 0, 3, 1) = zmp.previewControl.B;

    zmp.previewControl.I_Tilde = MatrixXd::Zero(4, 1);
    zmp.previewControl.I_Tilde << 1, 0, 0, 0;

    zmp.previewControl.F_Tilde = MatrixXd::Zero(4, 3);
    zmp.previewControl.F_Tilde.block(0, 0, 1, 3) = zmp.previewControl.C * zmp.previewControl.A;
    zmp.previewControl.F_Tilde.block(1, 0, 3, 3) = zmp.previewControl.A;

    zmp.previewControl.Q_Tilde = MatrixXd::Zero(4, 4);

    zmp.previewControl.Q_Tilde.block(0, 0, 1, 1) = MatrixXd::Identity(1, 1);
    zmp.previewControl.Q_Tilde.block(1, 1, 3, 3) = MatrixXd::Zero(3, 3);

    zmp.previewControl.R = MatrixXd::Identity(1, 1) * 0.000001;

    zmp.previewControl.A_Tilde = MatrixXd::Zero(4, 4);
    zmp.previewControl.A_Tilde.block(0, 0, 4, 1) = zmp.previewControl.I_Tilde;
    zmp.previewControl.A_Tilde.block(0, 1, 4, 3) = zmp.previewControl.F_Tilde;

    zmp.previewControl.K_Tilde = MatrixXd::Zero(4, 4);
    zmp.previewControl.K_Tilde = ZMP_DARE(zmp.previewControl.A_Tilde, zmp.previewControl.B_Tilde, zmp.previewControl.Q_Tilde, zmp.previewControl.R);

    zmp.previewControl.G_I = MatrixXd::Zero(1, 1);
    zmp.previewControl.G_I = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.B_Tilde).inverse() * zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.I_Tilde;

    zmp.previewControl.G_P = MatrixXd::Zero(1, zmp.previewControl.PreviewTimeSize);

    zmp.previewControl.G_X = MatrixXd::Zero(1, 3);
    zmp.previewControl.G_X = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.B_Tilde).inverse() * zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.F_Tilde;

    zmp.previewControl.A_Tilde_c = MatrixXd::Zero(4, 4);
    zmp.previewControl.A_Tilde_c = zmp.previewControl.A_Tilde - zmp.previewControl.B_Tilde * (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.B_Tilde).inverse() * zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.A_Tilde;

    zmp.previewControl.X_Tilde = MatrixXd::Zero(4, 1);

    for (int pre_cnt = 0; pre_cnt < zmp.previewControl.PreviewTimeSize; pre_cnt++)
    {
        if (pre_cnt == 0)
        {
            zmp.previewControl.X_Tilde = -zmp.previewControl.A_Tilde_c.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.I_Tilde;
            zmp.previewControl.G_P(0, pre_cnt) = -zmp.previewControl.G_I(0, 0);
        }
        else
        {
            zmp.previewControl.X_Tilde = zmp.previewControl.A_Tilde_c.transpose() * zmp.previewControl.X_Tilde;
            zmp.previewControl.G_P(0, pre_cnt) = ((zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.B_Tilde).inverse() * zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.X_Tilde)(0, 0);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    zmp.previewControl.Y.ref = MatrixXd::Zero(1, (int)(zmp.previewControl.RefTotalTrajSize + 0.001));
    zmp.previewControl.Y.m_ref = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.state = MatrixXd::Zero(3, 1); // 3X1 matrix
    zmp.previewControl.Y.E = 0;

    if (CommandFlag == SWAYMOTION || CommandFlag == KICKTEST)
    {
        zmp.previewControl.Y.sum_E = zmp.previewControl.Y.sum_E;
    }

    else
    {
        zmp.previewControl.Y.sum_E = 0;
        zmp.previewControl.Y.sum_P = 0;
        zmp.previewControl.Y.U = 0;
    }

    zmp.previewControl.X.ref = MatrixXd::Zero(1, (int)(zmp.previewControl.RefTotalTrajSize + 0.001));
    zmp.previewControl.X.m_ref = walking.step.LastfootCenter(X);
    zmp.previewControl.X.state = MatrixXd::Zero(3, 1); // 3X1 matrix
    zmp.previewControl.X.E = 0;

    if (CommandFlag == SWAYMOTION || CommandFlag == KICKTEST)
    {
        zmp.previewControl.X.sum_E = zmp.previewControl.X.sum_E;
    }

    else
    {
        zmp.previewControl.X.sum_E = 0;
        zmp.previewControl.X.sum_P = 0;
        zmp.previewControl.X.U = 0;
    }

    zmp.previewControl.Y.new_state = MatrixXd::Zero(3, 1);
    zmp.previewControl.Y.old_zmp = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.CoM = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.new_state(0) = zmp.previewControl.Y.CoM;
    zmp.previewControl.Y.dCoM = 0;

    zmp.previewControl.X.new_state = MatrixXd::Zero(3, 1);
    zmp.previewControl.X.old_zmp = walking.step.LastfootCenter(X);
    zmp.previewControl.X.CoM = walking.step.LastfootCenter(X);
    zmp.previewControl.X.new_state(0) = zmp.previewControl.X.CoM;
    zmp.previewControl.X.dCoM = 0;

    std::cout << "===== Setting End ZMP preview control =====" << std::endl;

    // zmp.previewControl.count = 0;
}

void CKubot::setZmpPreviewTime(double previewTime)
{

    /* setZmpPreviewTime
     * input : preview time for ZMP preview control
     * output : preview time, array about preview time for ZMP preview control,
     *          total ZMP trajectory array and time of the trajectory considering zmp preview time
     *
     * Before run function
     * [First, Run Crobot class's Function setWalkingTime before running this function]
     */

    zmp.previewControl.PreviewTime = previewTime;
    zmp.previewControl.PreviewReadyTime = 2.0;

    zmp.previewControl.PreviewTimeSize = zmp.previewControl.PreviewTime * (double)onesecSize;
    zmp.previewControl.PeriodTimeSize = walking.time.periodTime * (double)onesecSize;

    zmp.previewControl.RefTotalTrajSize = (zmp.previewControl.PreviewReadyTime + walking.step.total_num * walking.time.periodTime + 3 * zmp.previewControl.PreviewTime) * (double)onesecSize;

    zmp.previewControl.RefTotalTime = zmp.previewControl.RefTotalTrajSize / (double)onesecSize;

    std::cout << "RefTotalTrajSize :" << zmp.previewControl.RefTotalTrajSize << std::endl;
    std::cout << "RefTotalTrajTime :" << zmp.previewControl.RefTotalTime << std::endl;

    printf("%lf\t%lf\t%lf\t%lf\n", zmp.previewControl.PreviewTime, zmp.previewControl.PreviewTimeSize, zmp.previewControl.RefTotalTrajSize, zmp.previewControl.RefTotalTime);
}

MatrixXd CKubot::ZMP_DARE(MatrixXd &A, MatrixXd &B, MatrixXd &Q, MatrixXd &R) // Kookmin.Univ Preview
{
    unsigned int nSize = A.rows();
    MatrixXd Z(nSize * 2, nSize * 2);

    Z.block(0, 0, nSize, nSize) = A + B * R.inverse() * B.transpose() * (A.inverse()).transpose() * Q;
    Z.block(0, nSize, nSize, nSize) = -B * R.inverse() * B.transpose() * (A.inverse()).transpose();
    Z.block(nSize, 0, nSize, nSize) = -(A.inverse()).transpose() * Q;
    Z.block(nSize, nSize, nSize, nSize) = (A.inverse()).transpose();

    eZMPSolver.compute(Z, true);

    Eigen::MatrixXcd U(nSize * 2, nSize);
    unsigned int j = 0;
    for (unsigned int i = 0; i < nSize * 2; i++)
    {
        std::complex<double> eigenvalue = eZMPSolver.eigenvalues()[i];
        double dReal = eigenvalue.real();
        double dImag = eigenvalue.imag();

        if (std::sqrt((dReal * dReal) + (dImag * dImag)) < 1.0)
        {
            U.block(0, j, nSize * 2, 1) = eZMPSolver.eigenvectors().col(i);
            j++;
        }
    }
    if (j != nSize)
    {
        printf("Warning! ******* Pelvis Planning *******\n");
    }

    Eigen::MatrixXcd U1 = U.block(0, 0, nSize, nSize);
    Eigen::MatrixXcd U2 = U.block(nSize, 0, nSize, nSize);

    Eigen::MatrixXcd X = U2 * U1.inverse();

    return X.real();
}

void CKubot::walkingPatternGenerator(int controlMODE, int CommandFLAG, double steps, double FBsize, double LRsize, double Turnsize, double footHeight, int stop)
{

    if (controlMODE == CTRLMODE_SWAY)
    { // printf("==walking generate start =====\n");
        if (CommandFLAG == SWAYMOTION_PATTERN)
        {
            // printf("=========walkingPatternGenerator check========\n");

            // initializeGlobalPosition();
            walking.initialize = true;
            // setWalkingStep(7,walking.step.S_L_F,0,0,0,50);
            // setWalkingTime(2.0,0.2);
            setZmpPreview(1.5);
            // FootStepsPlanner();
            // generateZmpTraj(walking.step.LFoot,walking.step.RFoot);
            walking.walk = true;
            CommandFlag = SWAYMOTION;
        }
        else if (CommandFLAG == SWAYMOTION)
        {
            walking.initialize = false;

            zmpPreviewControl();

            // generateFootTraj_ver2(walking.step.Lfoot,walking.step.Rfoot);

            // if(walking.NewfootPlan == true)
            // {
            //     // changeWalkingStep(FBsize,LRsize,Turnsize);
            //     // FootPlannerWindowMove();
            //     // generateZmpTraj(walking.step.Lfoot,walking.step.Rfoot);
            //     walking.NewfootPlan = false;
            // }

            if (zmp.previewControl.count > zmp.previewControl.RefTotalTrajSize)
            {
                SWAY_READY = false;
                CommandFlag = NONE_ACT;

                printf("=========SWAY End========\n");
                printf("=========SWAY Ready========\n");
            }
            //            printf("=========Walking========\n");
        }
    }

    else if (controlMODE == CTRLMODE_KICKTEST)
    {
        if (CommandFLAG == KICK_PATTERN)
        {
            setZmpPreview(1.5);

            CommandFlag = KICKTEST;
        }
        else if (CommandFLAG == KICKTEST)
        {
            zmpPreviewControl();

            if (zmp.previewControl.count > zmp.previewControl.RefTotalTrajSize)
            {
                KICK_READY = false;
                CommandFlag = NONE_ACT;

                printf("=========KICK End========\n");
                printf("=========KICK Ready========\n");
            }
        }
    }

    else if (controlMODE == CTRLMODE_WALKING_TEST)
    {
        if (CommandFLAG == ACT_PATTERN_TEST)
        {
            printf("=========walkingPatternGenerator check========\n");

            initializeGlobalStepPosition();
            walking.initialize = true;
            setWalkingStep(steps, walking.step.S_L_F, FBsize, LRsize, Turnsize, footHeight); //  void CKubot::setWalkingStep(int steps, int Startfoot, double FBsize, double LRsize, double Turnsize, double footHeight)
            setWalkingTime(1.0, 0.2);
            setZmpPreview(1.5);
            FootStepsPlanner();
            generateZmpTraj(walking.step.LFoot, walking.step.RFoot);
            walking.walk = true;
            CommandFlag = ACT_TEST_WALK;
        }
        else if (CommandFLAG == ACT_SIDE_PATTERN_TEST) // SIDE++
        {
            // src로봇으로 걸을 수
            // zmp가 충분히 갔나
            // 입력을 할 수 있는지
            // 실시간으로 토픽을 날릴것
            // 회전 포함
            // xyz direction zmp, CoM, 발 궤적 그래
            // xy, yz, zx평면의 //
            initializeGlobalStepPosition();
            walking.initialize = true;
            setWalkingStep(steps, walking.step.S_L_F, FBsize, LRsize, Turnsize, footHeight);
            setWalkingTime(1.0, 0.2);
            setZmpPreview(1.5);
            FootStepsPlanner();
            generateZmpTraj(walking.step.LFoot, walking.step.RFoot);
            walking.walk = true;
            CommandFlag = ACT_TEST_WALK;
        }
        else if (CommandFLAG == ACT_TURN_PATTERN_TEST) // Turn
        {
            printf("========= side walking PatternGenerator check ========\n");

            initializeGlobalStepPosition();
            walking.initialize = true;
            setWalkingStep(steps, walking.step.S_L_F, FBsize, LRsize, Turnsize, footHeight);
            setWalkingTime(1.0, 0.2);
            setZmpPreview(1.5);
            FootStepsPlanner();
            generateZmpTraj(walking.step.LFoot, walking.step.RFoot);
            walking.walk = true;
            CommandFlag = ACT_TEST_WALK;
        }
        else if (CommandFLAG == ACT_START_WALK) // TURN++
        {
            printf("========= INF walking PatternGenerator check ========\n");

            initializeGlobalStepPosition();
            walking.initialize = true;
            setWalkingStep(5, walking.step.S_L_F, 0, 0, 0, 0.03); // 0.05
            setWalkingTime(0.5, 0.01);
            setZmpPreview(1.5);
            FootStepsPlanner();
            // FootStepsPlanner(walking.step.LFoot, walking.step.RFoot, -0.01);
            generateZmpTraj(walking.step.LFoot, walking.step.RFoot);
            walking.walk = true;
            CommandFlag = ACT_INF_WALK;
        }

        else if (CommandFLAG == ACT_TEST_WALK || CommandFLAG == ACT_INF_WALK || CommandFLAG == ACT_STOPPING_WALK || CommandFLAG == ACT_STOP_WALK)
        {
            walking.initialize = false;
            zmpPreviewControl();
            generateFootTraj(walking.step.LFoot, walking.step.RFoot);
            // zmp.previewControl.count++;
            walking.time.sec = (float)(zmp.previewControl.count) * (float)(tasktime);

            if (walking.NewfootPlan == true)
            {
                changeWalkingStep(FBsize, LRsize, Turnsize);
                // FootPlannerWindowMove();
                FootPlannerWindowMove(-0.0);
                generateZmpTraj(walking.step.LFoot, walking.step.RFoot);
                walking.NewfootPlan = false;
            }

            if (zmp.previewControl.count > zmp.previewControl.RefTotalTrajSize)
            {
                zmp.previewControl.count = 0;
                walking.time.sec = 0;
                walking.time.wsec = 0;

                walking.walk = false;
                CommandFlag = NONE_ACT;

                printf("=========WALK End========\n");
                // printf("=========집가자========\n");
            }
        }
    }
}

void CKubot::zmpPreviewControl()
{

    /* zmpPreviewControl
     * output : trajectory by zmp preview control --> zmp.previewControl.X.CoM, zmp.previewControl.Y.CoM [Crobot class's Variable]
     *
     * Before run function
     * [First, Run once Crobot class's Function initializeSystemID when starting program]
     * [Second, Run Crobot class's Function setZmpPreview before running this function]
     * [Third, Run Crobot class's Function zmp plan(ex setZmpSwapPlan) before running this function]
     */

    // printf("=========zmpPreviewControl check========\n");

    if (zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize - zmp.previewControl.PreviewTimeSize)
    {
        // printf("\n if 1 ====== \n");
        zmp.previewControl.Y.state = zmp.previewControl.Y.new_state;
        zmp.previewControl.X.state = zmp.previewControl.X.new_state;

        zmp.previewControl.Y.E = zmp.previewControl.Y.old_zmp - zmp.previewControl.Y.ref(0, zmp.previewControl.count);
        zmp.previewControl.X.E = zmp.previewControl.X.old_zmp - zmp.previewControl.X.ref(0, zmp.previewControl.count);

        //        std::cout<<"zmp.previewControl.Y.E :"<<zmp.previewControl.Y.E<<std::endl;
        //        std::cout<<"zmp.previewControl.X.E :"<<zmp.previewControl.X.E<<std::endl;

        zmp.previewControl.Y.m_ref = zmp.previewControl.Y.ref(0, zmp.previewControl.count);
        zmp.previewControl.X.m_ref = zmp.previewControl.X.ref(0, zmp.previewControl.count);

        zmp.previewControl.Y.sum_E = zmp.previewControl.Y.sum_E + zmp.previewControl.Y.E;
        zmp.previewControl.X.sum_E = zmp.previewControl.X.sum_E + zmp.previewControl.X.E;
        // printf("\n if 2222 ====== \n");

        for (int pre_cnt = 0; pre_cnt < zmp.previewControl.PreviewTimeSize; pre_cnt++)
        {
            // printf("\n pre_cnt : %d\n", pre_cnt);
            zmp.previewControl.Y.sum_P = zmp.previewControl.Y.sum_P + zmp.previewControl.G_P(0, pre_cnt) * zmp.previewControl.Y.ref(0, zmp.previewControl.count + pre_cnt);
            zmp.previewControl.X.sum_P = zmp.previewControl.X.sum_P + zmp.previewControl.G_P(0, pre_cnt) * zmp.previewControl.X.ref(0, zmp.previewControl.count + pre_cnt);
        }
        // printf("\n if 33333333333 ====== \n");
        zmp.previewControl.X.U = -zmp.previewControl.G_I(0, 0) * zmp.previewControl.X.sum_E - (zmp.previewControl.G_X * zmp.previewControl.X.state)(0, 0) - zmp.previewControl.X.sum_P;
        zmp.previewControl.Y.U = -zmp.previewControl.G_I(0, 0) * zmp.previewControl.Y.sum_E - (zmp.previewControl.G_X * zmp.previewControl.Y.state)(0, 0) - zmp.previewControl.Y.sum_P;

        zmp.previewControl.Y.new_state = zmp.previewControl.A * zmp.previewControl.Y.state + zmp.previewControl.B * zmp.previewControl.Y.U;
        zmp.previewControl.X.new_state = zmp.previewControl.A * zmp.previewControl.X.state + zmp.previewControl.B * zmp.previewControl.X.U;

        zmp.previewControl.Y.old_zmp = (zmp.previewControl.C * zmp.previewControl.Y.state)(0, 0);
        zmp.previewControl.X.old_zmp = (zmp.previewControl.C * zmp.previewControl.X.state)(0, 0);

        zmp.previewControl.Y.CoM = zmp.previewControl.Y.new_state(0, 0);
        zmp.previewControl.Y.dCoM = zmp.previewControl.Y.new_state(1, 0);
        zmp.previewControl.X.CoM = zmp.previewControl.X.new_state(0, 0);
        zmp.previewControl.X.dCoM = zmp.previewControl.X.new_state(1, 0);
        // printf("\n XY_com : %lf \n", zmp.previewControl.Y.CoM);

        zmp.previewControl.Y.sum_P = 0;
        zmp.previewControl.X.sum_P = 0;

        // cout << "y_com_testprint: \n" << zmp.previewControl.Y.CoM << endl;
    }
    else
    {
    }
}
////////////////ZMPFK 계산할 때 사용///////////////

MatrixXd CKubot::ZMPFK(VectorXd q)
{
    //  Extract position vector(3x1)

    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4, 4);
    // MatrixXd tmp_fk(4,4);

    tmp_m = getTransform6E() *
            jointToTransform56(q).inverse() *
            jointToTransform45(q).inverse() *
            jointToTransform34(q).inverse() *
            jointToTransform23(q).inverse() *
            jointToTransform12(q).inverse() *
            jointToTransform01_zmp(q).inverse() *
            getTransformI0().inverse();

    //    tmp_v = tmp_m.block(0,3,3,1);
    // tmp_fk = tmp_m.inverse();

    tmp_v(0) = tmp_m(0, 3);
    tmp_v(1) = tmp_m(1, 3);
    tmp_v(2) = tmp_m(2, 3);

    return tmp_v;
    // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;
}

MatrixXd CKubot::ZMPFK(VectorXd q, VectorXd qq)
{
    //  Extract position vector(3x1)

    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4, 4);
    MatrixXd tmf_g(4, 4);
    tmf_g << 1, 0, 0, qq(0),
        0, 1, 0, qq(1),
        0, 0, 1, qq(2),
        0, 0, 0, 1;

    // MatrixXd tmp_fk(4,4);

    // Left_foot 기반 계산

    tmp_m = tmf_g *
            getTransform6E().inverse() *
            jointToTransform56(q).inverse() *
            jointToTransform45(q).inverse() *
            jointToTransform34(q).inverse() *
            jointToTransform23(q).inverse() *
            jointToTransform12(q).inverse() *
            jointToTransform01(q).inverse();
    // getTransformI0().inverse();

    //    tmp_v = tmp_m.block(0,3,3,1);
    // tmp_fk = tmp_m.inverse();

    tmp_v(0) = tmp_m(0, 3);
    tmp_v(1) = tmp_m(1, 3);
    tmp_v(2) = tmp_m(2, 3);

    return tmp_v;
    // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;
}

MatrixXd CKubot::jointToTransform01_zmp(VectorXd q)

{
    //  Frame 1 to Frame 0
    // q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]

    MatrixXd tmp_m(4, 4);
    double qq = q(0);

    tmp_m << cos(qq), -sin(qq), 0, 0,
        sin(qq), cos(qq), 0, 0,
        0, 0, 1, -0.10464,
        0, 0, 0, 1;

    return tmp_m;
}

//////////////////////////////////////////////

void CKubot::sway(/*float readytime*/)
{

    printf("==== ref_setting_start ====\n");
    // static float time = 0;
    // static float currentAngle[12]; // Now nDoF = 12s
    zmp.previewControl.X.ref.resize(3, zmp.previewControl.RefTotalTrajSize);
    zmp.previewControl.Y.ref.resize(3, zmp.previewControl.RefTotalTrajSize);
    // MatrixXd X_ref(3,1), Y_ref(3,1);
    // zmp.previewControl.X.ref;
    // zmp.previewControl.Y.ref;

    for (int i = 0; i < zmp.previewControl.RefTotalTrajSize; i++)
    {
        // double time = i * tasktime;  // time은 0초에서 (3001-1)*0.005 = 15초 까지
        if (i < 5000)
        {
            // X_ref << 0,0,0;
            // Y_ref << 0,0,0;
            zmp.previewControl.X.ref.col(i) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(i) << 0,
                0,
                0;
            // zmp.previewControl.count = zmp.previewControl.count + 1;
            // cout << "time :" << time << endl;
            // cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref << endl;
        }
        else if (i >= 5000 && i < 7000)
        {
            zmp.previewControl.X.ref.col(i) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(i) << 0.05,
                0,
                0;
            // zmp.previewControl.count ++;
            // cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref.col(i) << endl;
        }
        else if (i >= 7000 && i < 9000)
        {
            zmp.previewControl.X.ref.col(i) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(i) << -0.05,
                0,
                0;
            // zmp.previewControl.count ++;
            //  cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref << endl;
        }
        else if (i >= 9000 && i < 11000)
        {
            zmp.previewControl.X.ref.col(i) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(i) << 0.05,
                0,
                0;
            // zmp.previewControl.count ++;
            //  cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref << endl;
        }
        else if (i >= 11000 && i < 13000)
        {
            zmp.previewControl.X.ref.col(i) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(i) << -0.05,
                0,
                0;
            // zmp.previewControl.count ++;
            // cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref.col(i) << endl;
        }
        else if (i >= 13000 && i < 15000)
        {
            zmp.previewControl.X.ref.col(i) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(i) << 0.05,
                0,
                0;
            // zmp.previewControl.count ++;
            // cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref.col(i) << endl;
        }
        else
        {
            zmp.previewControl.X.ref.col(i) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(i) << 0,
                0,
                0;
            // zmp.previewControl.count ++;
            //  cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref << endl;
        }
        // zmp.previewControl.count = zmp.previewControl.count + 1;
    }
    printf("==== ref_setting_end ====\n");
}

void CKubot::DSP_baseRef(/*float readytime*/)
{

    printf("==== ref_setting_start ====\n");
    // static float time = 0;
    // static float currentAngle[12]; // Now nDoF = 12s
    zmp.previewControl.X.ref.resize(3, zmp.previewControl.RefTotalTrajSize);
    zmp.previewControl.Y.ref.resize(3, zmp.previewControl.RefTotalTrajSize);
    // MatrixXd X_ref(3,1), Y_ref(3,1);
    // zmp.previewControl.X.ref;
    // zmp.previewControl.Y.ref;

    for (DSP_count = 0; DSP_count < zmp.previewControl.RefTotalTrajSize; DSP_count++)
    {
        // double time = i * tasktime;  // time은 0초에서 (3001-1)*0.005 = 15초 까지
        if (DSP_count < 3000)
        {
            // X_ref << 0,0,0;
            // Y_ref << 0,0,0;
            zmp.previewControl.X.ref.col(DSP_count) << 0,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << 0,
                0,
                0;
            // zmp.previewControl.count = zmp.previewControl.count + 1;
            // cout << "time :" << time << endl;
            // cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref << endl;
        }
        else if (DSP_count >= 3000 && DSP_count < 9000)
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.05,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << 0.05,
                0,
                0;
            // zmp.previewControl.count ++;
            //  cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref.col(i) << endl;
        }

        else if (DSP_count >= 9000 && DSP_count < 10000)
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.05,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << 0,
                0,
                0;
        }

        else if (DSP_count >= 10000 && DSP_count < 16000)
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.05,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << -0.05,
                0,
                0;
        }

        else if (DSP_count >= 16000 && DSP_count < 17000)
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.05,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << 0,
                0,
                0;
        }

        else if (DSP_count >= 17000 && DSP_count < 23000)
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.1,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << 0.1,
                0,
                0;
            // zmp.previewControl.count ++;
            //  cout << "XY_ref_testprint: \n" << zmp.previewControl.Y.ref.col(i) << endl;
        }

        else if (DSP_count >= 23000 && DSP_count < 24000)
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.1,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << 0,
                0,
                0;
        }

        else if (DSP_count >= 24000 && DSP_count < 30000)
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.1,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << -0.1,
                0,
                0;
        }

        else
        {
            zmp.previewControl.X.ref.col(DSP_count) << 0.1,
                0,
                0;
            zmp.previewControl.Y.ref.col(DSP_count) << 0,
                0,
                0;
        }
    }
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
////////////////////////////05.20/////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
void CKubot::setWalkingStep(int steps, int Startfoot, double FBsize, double LRsize, double Turnsize, double footHeight)
{
    /* setWalkingStep, set variables related to [step] struct for walkin
     * input : steps, starting foot (select walking.step.S_L_F or walking.step.S_R_F), step size, foot height for walking
     *
     * main processing:
     * 1.
     */

    walking.step.total_num = steps;
    walking.step.total_footprints = steps + 1;
    walking.step.start_swing_foot = Startfoot; // hoooo //여기가 핵심. 시작발이 중요하다

    // printf(C_CYAN "total step num = %d\t total step footprints = %d\n" C_RESET, walking.step.total_num, walking.step.total_footprints);

    if (LRsize < 0 || Turnsize < 0)
    {
        walking.step.start_swing_foot = walking.step.S_R_F;
    }
    else
    {
        walking.step.start_swing_foot = walking.step.S_L_F;
    }

    walking.step.footHeight = footHeight;

    if (FBsize > 0)
    {
        walking.step.walking_f_or_b = walking.step.forwardWalking;
    }
    else if (FBsize < 0)
    {
        walking.step.walking_f_or_b = walking.step.backwardWalking;
    }
    else
    {
        walking.step.walking_f_or_b = 0;
    }

    walking.step.FBstepSize = FBsize;
    walking.step.oldFBstepSize = walking.step.FBstepSize;

    if (LRsize > 0)
    {
        walking.step.walking_l_or_r = walking.step.leftWalking;
    }
    else if (LRsize < 0)
    {
        walking.step.walking_l_or_r = walking.step.rightWalking;
    }
    else
    {
        walking.step.walking_l_or_r = 0;
    }

    walking.step.LRstepSize = LRsize;
    walking.step.oldLRstepSize = walking.step.LRstepSize;

    if (Turnsize != 0)
    {
        if (Turnsize > 0)
        {
            walking.step.walking_lT_or_rT = walking.step.rightTurning;
            walking.step.walking_Turn = walking.step.TurnModeOn;
        }
        else if (Turnsize < 0)
        {
            walking.step.walking_lT_or_rT = walking.step.leftTurning;
            walking.step.walking_Turn = walking.step.TurnModeOn;
        }
        walking.step.turningPoint = walking.step.FBstepSize / Turnsize;

        if (walking.step.total_num == 1)
        {
            walking.step.walking_Turn = walking.step.TurnModeOff;
        }
    }
    else
    {
        walking.step.walking_lT_or_rT = 0;
        walking.step.walking_Turn = walking.step.TurnModeOff;
    }

    walking.step.TurnSize = Turnsize;
    walking.step.oldTurnSize = walking.step.TurnSize;

    // printf(C_CYAN "FBstepSize =%f m\t LRstepSize =%f m\t TurnSize =%f degree\n" C_RESET, walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize * R2D);
    // printf(C_CYAN "footHeight =%f m\n" C_RESET, walking.step.footHeight);
    // printf(C_CYAN "turningPoint =%f m\n" C_RESET, walking.step.turningPoint);
}

void CKubot::setWalkingTime(double periodTime, double DSPratio)
{

    /* setWalkingTime, set variables related to [time] struct for walking
     * input : period time, DSP ratio for walking,
     * output : Setting walking period time, DSP ratio, SSP ratio
     *
     * main processing:
     * 1.
     */

    walking.time.periodTime = periodTime;
    walking.time.DSP_ratio = DSPratio;
    walking.time.SSP_ratio = 1 - walking.time.DSP_ratio; // hooooo // ratio:비율

    // printf(C_CYAN "Walking periodTime = %lf sec\t DSP %lf\t SSP %lf\n" C_RESET, periodTime, walking.time.DSP_ratio, walking.time.SSP_ratio);

    walking.time.SSP_start_time = walking.time.periodTime * walking.time.DSP_ratio / 2;
    walking.time.SSP_time = walking.time.periodTime * walking.time.SSP_ratio;
    walking.time.SSP_end_time = walking.time.SSP_start_time + walking.time.SSP_time; // hooooo // 우리가 봐야하는 구문 DSP일때 zmp궤적에 맞춰서 com 이동

    // walking.time.XYDSP_ratio = 0.6 * DSPratio;
    // walking.time.XYSSP_ratio = 1 - walking.time.XYDSP_ratio;

    // walking.time.XYSSP_start_time = walking.time.periodTime * walking.time.XYDSP_ratio / 2;
    // walking.time.XYSSP_time = walking.time.periodTime * walking.time.XYSSP_ratio;
    // walking.time.XYSSP_end_time = walking.time.XYSSP_start_time + walking.time.XYSSP_time;
}

void CKubot::FootStepsPlanner()
{ // turn관련 식만 주석처리하고 사용
    /* FootPlanner, Planning Footsteps
     *
     * main processing:
     * 1. Planning Footsteps
     */

    float Y_step_position = 0;

    if (walking.step.total_footprints % 2 == 0)
    {                                                                                           // hooooo //발자국 짝수 보행 홀수
        walking.step.LFoot = MatrixXd::Zero(PatternElement, walking.step.total_footprints / 2); // patternElement : 발자국 개수(xyzYRP)
        walking.step.RFoot = MatrixXd::Zero(PatternElement, walking.step.total_footprints / 2);
    }
    else
    {
        if (walking.step.start_swing_foot == walking.step.S_L_F)
        {
            walking.step.LFoot = MatrixXd::Zero(PatternElement, (int)((float)(walking.step.total_footprints) / 2.));
            walking.step.RFoot = MatrixXd::Zero(PatternElement, (int)((float)(walking.step.total_footprints) / 2.) + 1);
        }
        else if (walking.step.start_swing_foot == walking.step.S_R_F)
        {
            walking.step.LFoot = MatrixXd::Zero(PatternElement, (int)((float)(walking.step.total_footprints) / 2.) + 1);
            walking.step.RFoot = MatrixXd::Zero(PatternElement, (int)((float)(walking.step.total_footprints) / 2.)); // hooooo //반올림 반내림 자료형 이용
        }
    }

    for (int m_step = 0; m_step < walking.step.total_footprints; m_step++)
    { // hooooo //m_step : 발자국 개수 시작발왼발 -> 오른발 지지
        if (walking.step.start_swing_foot == walking.step.S_L_F)
        {
            if (m_step == walking.step.total_footprints - 1)
            {
                if (walking.step.total_footprints % 2 == 1)
                { // 왼쪽발과 위치를 나란히 놓겠다.
                    walking.step.RFoot(X, m_step / 2) = (1 - walking.step.walking_Turn) * (walking.step.LFoot(X, m_step / 2 - 1)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step - 1) * -walking.step.TurnSize));
                    walking.step.RFoot(Y, m_step / 2) = (1 - walking.step.walking_Turn) * (walking.step.LFoot(Y, m_step / 2 - 1) + 2 * body_kine.lower.R.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step - 1) * -walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.RFoot(Yaw, m_step / 2) = (m_step - 1) * walking.step.TurnSize; // hooooo //왼발 기준으로 오른발을 어디에 놨으면 좋겠다.
                    // 괄호안에 0 1 2 3 4 우리가 세는 발자국 1 2 3 4 5
                }
                else
                {
                    walking.step.LFoot(X, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * walking.step.RFoot(X, (int)((float)(m_step) / 2.)) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step - 1) * -walking.step.TurnSize));
                    walking.step.LFoot(Y, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * (((int)((float)(m_step) / 2.)) * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step - 1) * -walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.LFoot(Yaw, (int)((float)(m_step) / 2.)) = (m_step - 1) * walking.step.TurnSize;

                    walking.step.RFoot(X, m_step / 2) = (1 - walking.step.walking_Turn) * (walking.step.LFoot(X, m_step / 2)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step - 1) * -walking.step.TurnSize));
                    walking.step.RFoot(Y, m_step / 2) = (1 - walking.step.walking_Turn) * (walking.step.LFoot(Y, m_step / 2) + 2 * body_kine.lower.R.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step - 1) * -walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.RFoot(Yaw, m_step / 2) = (m_step - 1) * walking.step.TurnSize;
                }

                //= * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y))\
                                                                     + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step - 1)*-walking.step.TurnSize) + walking.step.turningPoint);
                //= 2 * body_kine.lower.R.Base_to_Foot(Y))\
                                                   + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step - 1)*-walking.step.TurnSize) + walking.step.turningPoint);
            }
            else if (m_step % 2 == 0)
            {
                if (m_step == 0)
                { // 짝수 1 2 3 발자국  0 1 2 m_step
                    walking.step.RFoot(X, m_step / 2) = 0;
                    walking.step.RFoot(Y, m_step / 2) = (1 - walking.step.walking_Turn) * ((m_step / 2) * walking.step.LRstepSize + body_kine.lower.R.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step) * -walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.RFoot(Yaw, m_step / 2) = 0;
                }
                else
                {
                    walking.step.RFoot(X, m_step / 2) = (1 - walking.step.walking_Turn) * ((m_step)*walking.step.FBstepSize) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step) * -walking.step.TurnSize));
                    walking.step.RFoot(Y, m_step / 2) = (1 - walking.step.walking_Turn) * ((m_step / 2) * walking.step.LRstepSize + body_kine.lower.R.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step) * -walking.step.TurnSize) + walking.step.turningPoint) + Y_step_position;
                    walking.step.RFoot(Yaw, m_step / 2) = (m_step)*walking.step.TurnSize;
                }
            }
            else
            { // 홀수 float0.5를 int로 때리면 0으로 만들고 행렬 속으로 집어넣어

                walking.step.LFoot(X, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * ((m_step)*walking.step.FBstepSize) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step) * -walking.step.TurnSize));
                walking.step.LFoot(Y, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * (((int)((float)(m_step) / 2.) + 1) * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step) * -walking.step.TurnSize) + walking.step.turningPoint) - Y_step_position;
                walking.step.LFoot(Yaw, (int)((float)(m_step) / 2.)) = (m_step)*walking.step.TurnSize;
            }
        }
        else if (walking.step.start_swing_foot == walking.step.S_R_F)
        {
            if (m_step == walking.step.total_footprints - 1)
            {
                if (walking.step.total_footprints % 2 == 1)
                {
                    walking.step.LFoot(X, m_step / 2) = (1 - walking.step.walking_Turn) * (walking.step.RFoot(X, m_step / 2 - 1)) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step - 1) * -walking.step.TurnSize));
                    walking.step.LFoot(Y, m_step / 2) = (1 - walking.step.walking_Turn) * (walking.step.RFoot(Y, m_step / 2 - 1) + 2 * body_kine.lower.L.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step - 1) * -walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.LFoot(Yaw, m_step / 2) = (m_step - 1) * walking.step.TurnSize;
                }
                else
                {
                    walking.step.RFoot(X, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * (walking.step.LFoot(X, (int)((float)(m_step) / 2.))) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step - 1) * -walking.step.TurnSize));
                    walking.step.RFoot(Y, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * (((int)((float)(m_step) / 2.)) * walking.step.LRstepSize + body_kine.lower.R.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step - 1) * -walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.RFoot(Yaw, (int)((float)(m_step) / 2.)) = (m_step - 1) * walking.step.TurnSize;
                }
            }
            else if (m_step % 2 == 0)
            {
                if (m_step == 0)
                {
                    walking.step.LFoot(X, m_step / 2) = 0;
                    walking.step.LFoot(Y, m_step / 2) = (1 - walking.step.walking_Turn) * ((m_step / 2) * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step) * -walking.step.TurnSize) + walking.step.turningPoint);
                    walking.step.LFoot(Yaw, m_step / 2) = 0;
                }
                else
                {
                    walking.step.LFoot(X, m_step / 2) = (1 - walking.step.walking_Turn) * ((m_step)*walking.step.FBstepSize) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step) * -walking.step.TurnSize));
                    walking.step.LFoot(Y, m_step / 2) = (1 - walking.step.walking_Turn) * ((m_step / 2) * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step) * -walking.step.TurnSize) + walking.step.turningPoint) - Y_step_position;
                    walking.step.LFoot(Yaw, m_step / 2) = (m_step)*walking.step.TurnSize;
                }
            }
            else
            {
                walking.step.RFoot(X, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * ((m_step)*walking.step.FBstepSize) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_step) * -walking.step.TurnSize));
                walking.step.RFoot(Y, (int)((float)(m_step) / 2.)) = (1 - walking.step.walking_Turn) * (((int)((float)(m_step) / 2.) + 1) * walking.step.LRstepSize + body_kine.lower.R.Base_to_Foot(Y)) + (walking.step.walking_Turn) * ((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_step) * -walking.step.TurnSize) + walking.step.turningPoint) + Y_step_position;
                walking.step.RFoot(Yaw, (int)((float)(m_step) / 2.)) = (m_step)*walking.step.TurnSize;
            }
        }
    }
    std::cout << "LFoot : " << std::endl << walking.step.LFoot << std::endl;
    std::cout << "RFoot : " << std::endl << walking.step.RFoot << std::endl;
    // printf(C_GREEN "Planning Foot steps \n" C_RESET);
}

void CKubot::generateZmpTraj(MatrixXd LfootPrints, MatrixXd RfootPrints)
{

    /* zmpPlanning
     * output : Generate ZMP reference trajectory for walking --> zmp.previewControl.Y.ref [Crobot class's Variable]
     *
     * Before run function
     * [First, Run once Crobot class's Function initializeSystemID when starting program]
     * [Second, Run Crobot class's Function setWalkingStep before running this function]
     * [Third, Run Crobot class's Function setWalkingTime before running this function]
     * [4th, Run Crobot class's Function setZmpPreview before running this function]
     */

    // printf( "Start generateZmpTraj\n" );

    static VectorXd Foot = VectorXd::Zero(6);
    static Vector3d global_offset = VectorXd::Zero(3);
    static double zmp_y_offset = 0.065 * 0;
    double ratio = 1;     // 0.1;  //0.15
    double ratio2 = 0.00; // 0.25;
    // double gazebo_ratio = 0.15;

    // printf( "Start 1\n" );

    for (int pre_cnt = 0;
         pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + 0 * walking.time.periodTime) / zmp.previewControl.RefTotalTime;
         pre_cnt++)
    { // hooooo : 전체 사이즈에 변수 넣어서 곱하고 어쩌고 한 값까지 cnt를 생성하는 것.
        zmp.previewControl.X.ref(pre_cnt) = walking.step.LastfootCenter(X);
        zmp.previewControl.Y.ref(pre_cnt) = walking.step.LastfootCenter(Y);
    }

    //    std::cout<<"Generate Zmp Traj"<<std::endl;
    for (int m_step = 0; m_step < walking.step.total_num; m_step++)
    { // 여기서 m_step -> 그냥 step

        if (walking.step.start_swing_foot == walking.step.S_L_F)
        {
            if ((m_step) % 2 == 0)
            {
                Eigen::Vector3d offset;
                offset(X) = 0;
                // offset(Y) = 0;
                // offset(X) = -walking.step.FBstepSize*ratio2;
                offset(Y) = body_kine.lower.LEG_SIDE_OFFSET * ratio;
                offset(Z) = 0;
                // printf( "Start 2\n" );

                Foot(X) = RfootPrints(X, m_step / 2);
                Foot(Y) = RfootPrints(Y, m_step / 2);
                Foot(Z) = RfootPrints(Z, m_step / 2);
                Foot(Roll) = RfootPrints(Roll, m_step / 2);
                Foot(Pitch) = RfootPrints(Pitch, m_step / 2);
                Foot(Yaw) = RfootPrints(Yaw, m_step / 2);

                global_offset = local2Global(offset, Foot); // 무시 ㄱ 회전관련 개삘   Z_Rot33 = MatrixXd CKubot::rotMatZ(double q)

                for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step)*walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt++)
                {
                    zmp.previewControl.X.ref(pre_cnt) = RfootPrints(X, m_step / 2);
                    zmp.previewControl.Y.ref(pre_cnt) = RfootPrints(Y, m_step / 2) + zmp_y_offset;
                }
            }

            else if ((m_step) % 2 == 1)
            {
                Eigen::Vector3d offset;
                offset(X) = 0;
                // offset(Y) = 0;
                // offset(X) = -walking.step.FBstepSize*ratio2;
                offset(Y) = -body_kine.lower.LEG_SIDE_OFFSET * ratio;
                offset(Z) = 0;

                VectorXd Foot = VectorXd::Zero(6);
                Foot(X) = LfootPrints(X, (int)((double)(m_step) / 2.));
                Foot(Y) = LfootPrints(Y, (int)((double)(m_step) / 2.));
                Foot(Z) = LfootPrints(Z, (int)((double)(m_step) / 2.));
                Foot(Roll) = LfootPrints(Roll, (int)((double)(m_step) / 2.));
                Foot(Pitch) = LfootPrints(Pitch, (int)((double)(m_step) / 2.));
                Foot(Yaw) = LfootPrints(Yaw, (int)((double)(m_step) / 2.));

                global_offset = local2Global(offset, Foot);

                for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step)*walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt++)
                {

                    zmp.previewControl.X.ref(pre_cnt) = LfootPrints(X, (int)((double)(m_step) / 2.));
                    zmp.previewControl.Y.ref(pre_cnt) = LfootPrints(Y, (int)((double)(m_step) / 2.)) - zmp_y_offset;
                }
            }
        }

        else if (walking.step.start_swing_foot == walking.step.S_R_F)
        {
            if ((m_step) % 2 == 0)
            {
                Eigen::Vector3d offset;

                offset(X) = 0;
                // offset(Y) = 0;
                // offset(X) = -walking.step.FBstepSize*ratio2;
                offset(Y) = -body_kine.lower.LEG_SIDE_OFFSET * ratio;
                offset(Z) = 0;

                VectorXd Foot = VectorXd::Zero(6);
                Foot(X) = LfootPrints(X, m_step / 2);
                Foot(Y) = LfootPrints(Y, m_step / 2);
                Foot(Z) = LfootPrints(Z, m_step / 2);
                Foot(Roll) = LfootPrints(Roll, m_step / 2);
                Foot(Pitch) = LfootPrints(Pitch, m_step / 2);
                Foot(Yaw) = LfootPrints(Yaw, m_step / 2);

                global_offset = local2Global(offset, Foot);

                for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step)*walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt++)
                {

                    zmp.previewControl.X.ref(pre_cnt) = LfootPrints(X, m_step / 2);
                    zmp.previewControl.Y.ref(pre_cnt) = LfootPrints(Y, m_step / 2) - zmp_y_offset;
                }
            }

            else if ((m_step) % 2 == 1)
            {

                Eigen::Vector3d offset;

                offset(X) = 0;
                // offset(Y) = 0;
                // offset(X) = -walking.step.FBstepSize*ratio2;
                offset(Y) = body_kine.lower.LEG_SIDE_OFFSET * ratio;
                offset(Z) = 0;

                VectorXd Foot = VectorXd::Zero(6);
                Foot(X) = RfootPrints(X, (int)((double)(m_step) / 2.));
                Foot(Y) = RfootPrints(Y, (int)((double)(m_step) / 2.));
                Foot(Z) = RfootPrints(Z, (int)((double)(m_step) / 2.));
                Foot(Roll) = RfootPrints(Roll, (int)((double)(m_step) / 2.));
                Foot(Pitch) = RfootPrints(Pitch, (int)((double)(m_step) / 2.));
                Foot(Yaw) = RfootPrints(Yaw, (int)((double)(m_step) / 2.));

                global_offset = local2Global(offset, Foot);

                for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step)*walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;
                     pre_cnt++)
                {

                    zmp.previewControl.X.ref(pre_cnt) = RfootPrints(X, (int)((double)(m_step) / 2.));
                    zmp.previewControl.Y.ref(pre_cnt) = RfootPrints(Y, (int)((double)(m_step) / 2.)) + zmp_y_offset;
                }
            }
        }
    }
    // printf( "Start 3\n" );

    //    std::cout<<"Generate Zmp Traj"<<std::endl; // zmp를 왼발 오른발 사이에 두겠다.
    for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (walking.step.total_num) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;
         pre_cnt < (int)(zmp.previewControl.RefTotalTrajSize + 0.001);
         pre_cnt++)
    {
        if (walking.step.start_swing_foot == walking.step.S_L_F)
        {
            if ((int)(walking.step.total_footprints) % 2 == 0)
            {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, (int)(walking.step.total_footprints / 2.) - 1) + RfootPrints(X, (int)(walking.step.total_footprints / 2.) - 1)) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, (int)(walking.step.total_footprints / 2.) - 1) + RfootPrints(Y, (int)(walking.step.total_footprints / 2.) - 1)) / 2.;
            }
            else
            {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, (int)(walking.step.total_footprints / 2.) - 1) + RfootPrints(X, (int)(walking.step.total_footprints / 2.))) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, (int)(walking.step.total_footprints / 2.) - 1) + RfootPrints(Y, (int)(walking.step.total_footprints / 2.))) / 2.;
            }
        }
        else if (walking.step.start_swing_foot == walking.step.S_R_F)
        {
            if ((int)(walking.step.total_footprints) % 2 == 0)
            {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, (int)(walking.step.total_footprints / 2.) - 1) + RfootPrints(X, (int)(walking.step.total_footprints / 2.) - 1)) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, (int)(walking.step.total_footprints / 2.) - 1) + RfootPrints(Y, (int)(walking.step.total_footprints / 2.) - 1)) / 2.;
            }
            else
            {
                zmp.previewControl.X.ref(pre_cnt) = (LfootPrints(X, (int)(walking.step.total_footprints / 2.)) + RfootPrints(X, (int)(walking.step.total_footprints / 2.) - 1)) / 2.;
                zmp.previewControl.Y.ref(pre_cnt) = (LfootPrints(Y, (int)(walking.step.total_footprints / 2.)) + RfootPrints(Y, (int)(walking.step.total_footprints / 2.) - 1)) / 2.;
            }
        }
        //        std::cout << "previewControl.X.ref :" << previewControl.X.ref(pre_cnt) << std::endl;
    }
    //            //    ////    std::cout<<"Generate Zmp Traj"<<std::endl;
    //    double first_step_time = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime) / zmp.previewControl.RefTotalTime;
    //    double first_step_x = zmp.previewControl.X.ref(int(first_step_time + 0.001));
    //    double first_step_y = zmp.previewControl.Y.ref(int(first_step_time + 0.001));
    //    for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime) / zmp.previewControl.RefTotalTime; \
            //             pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + walking.time.SSP_start_time) / zmp.previewControl.RefTotalTime; \
            //             pre_cnt++)
    //     {
    //        zmp.previewControl.X.ref(pre_cnt) = cosWave(first_step_x - walking.step.LastfootCenter(X), zmp.previewControl.RefTotalTrajSize * walking.time.SSP_start_time / zmp.previewControl.RefTotalTime, pre_cnt - first_step_time, walking.step.LastfootCenter(X));
    //        zmp.previewControl.Y.ref(pre_cnt) = cosWave(first_step_y - walking.step.LastfootCenter(Y), zmp.previewControl.RefTotalTrajSize * walking.time.SSP_start_time / zmp.previewControl.RefTotalTime, pre_cnt - first_step_time, walking.step.LastfootCenter(Y));
    //     }
    //    ////    std::cout<<"Generate Zmp Traj"<<std::endl;
    //    for (int m_step = 0; m_step < walking.step.total_footprints - 1; m_step++) {
    //        double step_time = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step) * walking.time.periodTime + walking.time.SSP_end_time) / zmp.previewControl.RefTotalTime;
    //        double next_step_time = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime + walking.time.SSP_end_time) / zmp.previewControl.RefTotalTime;
    //        double step_x = zmp.previewControl.X.ref(int(step_time + 0.001));
    //        double step_y = zmp.previewControl.Y.ref(int(step_time + 0.001));
    //        double next_step_x = zmp.previewControl.X.ref(int(next_step_time + 0.001));
    //        double next_step_y = zmp.previewControl.Y.ref(int(next_step_time + 0.001));
    //        for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step) * walking.time.periodTime + walking.time.SSP_end_time) / zmp.previewControl.RefTotalTime; \
            //                 pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (m_step + 1) * walking.time.periodTime + walking.time.SSP_start_time) / zmp.previewControl.RefTotalTime; \
            //                 pre_cnt++)
    //         {
    //            zmp.previewControl.X.ref(pre_cnt) = cosWave(next_step_x - step_x, zmp.previewControl.RefTotalTrajSize * walking.time.SSP_start_time * 2.0 / zmp.previewControl.RefTotalTime, pre_cnt - step_time, step_x);
    //            zmp.previewControl.Y.ref(pre_cnt) = cosWave(next_step_y - step_y, zmp.previewControl.RefTotalTrajSize * walking.time.SSP_start_time * 2.0 / zmp.previewControl.RefTotalTime, pre_cnt - step_time, step_y);
    //         }
    //    }
    //    //    ////        std::cout<<"Generate Zmp Traj"<<std::endl;
    //    double step_time = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (walking.step.total_footprints - 1) * walking.time.periodTime + walking.time.SSP_end_time) / zmp.previewControl.RefTotalTime;
    //    double final_step_time = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (walking.step.total_footprints) * walking.time.periodTime) / zmp.previewControl.RefTotalTime;
    //    //    std::cout<<"Generate Zmp Trajss"<<std::endl;
    //    //    std::cout<<step_time<<std::endl;
    //    //    std::cout<<final_step_time<<std::endl;
    //    double step_x = zmp.previewControl.X.ref(int(step_time + 0.001));
    //    double step_y = zmp.previewControl.Y.ref(int(step_time + 0.001));
    //    double final_step_x = zmp.previewControl.X.ref(int(final_step_time + 0.001));
    //    double final_step_y = zmp.previewControl.Y.ref(int(final_step_time + 0.001));
    //    //        std::cout<<"Generate Zmp Traj"<<std::endl;
    //    for (int pre_cnt = zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (walking.step.total_footprints - 1) * walking.time.periodTime + walking.time.SSP_end_time) / zmp.previewControl.RefTotalTime; \
            //             pre_cnt < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (walking.step.total_footprints) * walking.time.periodTime) / zmp.previewControl.RefTotalTime; \
            //             pre_cnt++)
    //     {
    //        zmp.previewControl.X.ref(pre_cnt) = cosWave(final_step_x - step_x, zmp.previewControl.RefTotalTrajSize * walking.time.SSP_start_time / zmp.previewControl.RefTotalTime, pre_cnt - step_time, step_x);
    //        zmp.previewControl.Y.ref(pre_cnt) = cosWave(final_step_y - step_y, zmp.previewControl.RefTotalTrajSize * walking.time.SSP_start_time / zmp.previewControl.RefTotalTime, pre_cnt - step_time, step_y);
    //     }
    //    //                std::cout<<"Generate Zmp Traj"<<std::endl;
    // printf( "\nGenerate Zmp Traj\n" );
}

VectorXd CKubot::local2Global(Vector3d Vector, VectorXd local)
{

    Vector3d outputVector = VectorXd::Zero(3);
    Vector3d inputVector = VectorXd::Zero(3);
    VectorXd Local = VectorXd::Zero(6);
    Local = local;
    inputVector(X) = Vector(X);
    inputVector(Y) = Vector(Y);
    inputVector(Z) = Vector(Z);

    outputVector = rotMatZ(Local(Yaw)) * inputVector;

    return outputVector;
}

void CKubot::initializeGlobalStepPosition()
{
    /* initializeGlobalStepPosition, Initialize Step Position for Walking in Global frame coordinate system
     *
     * main processing:
     * 1. initialize variables related to Global Step PositionS
     */
    body_kine.lower.L.Base_to_Foot(Y) = 0.05;
    body_kine.lower.R.Base_to_Foot(Y) = -0.05;

    walking.step.preLfoot(X) = 0;
    walking.step.preLfoot(Y) = 0.05; // walking.step.preLfoot(Y) = body_kine.lower.L.Base_to_Foot(Y);   //  0.065   //  0.05
    walking.step.preLfoot(Z) = 0;
    walking.step.preLfoot(Roll) = 0;
    walking.step.preLfoot(Pitch) = 0;
    walking.step.preLfoot(Yaw) = 0;
    walking.step.preRfoot(X) = 0;
    walking.step.preRfoot(Y) = -0.05; // body_kine.lower.R.Base_to_Foot(Y);
    walking.step.preRfoot(Z) = 0;
    walking.step.preRfoot(Roll) = 0;
    walking.step.preRfoot(Pitch) = 0;
    walking.step.preRfoot(Yaw) = 0;

    walking.step.prefootCenter(X) = 0;
    walking.step.prefootCenter(Y) = 0;
    walking.step.prefootCenter(Yaw) = 0;

    //////////////////////////////////////////
    LFoot.refpos(X) = 0;
    RFoot.refpos(X) = 0;
    LFoot.refpos(Y) = 0;
    RFoot.refpos(Y) = 0;
    LFoot.refpos(Yaw) = 0;
    RFoot.refpos(Yaw) = 0;
    Base.refpos(Yaw) = 0;
    //////////////////////////////////////////

    VectorXd init_LFoot_pos(6);
    VectorXd init_RFoot_pos(6);

    // init_LFoot_pos << 0, 0.05, 0, 0, 0, 0; //body_kine.lower.L.Base_to_Foot(Y)
    // init_RFoot_pos << 0, -0.05, 0, 0, 0, 0; //body_kine.lower.R.Base_to_Foot(Y)

    init_LFoot_pos << 0, walking.step.preLfoot(Y), 0, 0, 0, 0; // body_kine.lower.L.Base_to_Foot(Y)
    init_RFoot_pos << 0, walking.step.preRfoot(Y), 0, 0, 0, 0; // body_kine.lower.R.Base_to_Foot(Y)

    LFoot.ref_G_pattern_pos = init_LFoot_pos;
    RFoot.ref_G_pattern_pos = init_RFoot_pos;

    // for (unsigned int k = 0; k < MPC_GRF_N; ++k) {
    //     LFoot.ref_G_pattern_pos.col(k) = init_LFoot_pos;
    //     RFoot.ref_G_pattern_pos.col(k) = init_RFoot_pos;
    // }

    // printf(LFoot.ref_G_pattern_pos);
    // printf("\n");
    // printf(RFoot.ref_G_pattern_pos);
    // printf("\n");
    // static RigidBodyDynamics::Math::Vector3d EulerZYX;

    // EulerZYX = Vector3f::Zero().cast <double> ();
    // LFoot.ref_G_quat = Math::Quaternion::fromZYXAngles(EulerZYX);
    // RFoot.ref_G_quat = Math::Quaternion::fromZYXAngles(EulerZYX);

    // printf(C_GREEN "Initialize Global Step Position\n" C_RESET);

    lastGlobalStepPositionUpdate();
}

void CKubot::lastGlobalStepPositionUpdate()
{
    /* lastGlobalStepPositionUpdate, update Last Step Position for Walking in Global frame coordinate system
     *
     * main processing:
     * 1. update variables related to Last Step Position
     */

    walking.step.LastLfoot = walking.step.preLfoot;
    walking.step.LastRfoot = walking.step.preRfoot;

    walking.step.LastfootCenter = walking.step.prefootCenter;

    // printf(C_GREEN "Update Last Global Step Position\n" C_RESET);
}

void CKubot::generateFootTraj(MatrixXd LfootPrints, MatrixXd RfootPrints)
{

    static int currentLfootstep = 0;
    static int currentRfootstep = 0;
    double hip_compensate = 2.0 * PI / 180.0; // onlyzmpcontrol 1.75*PI/180.0

    static bool checkstep = false;
    static bool SSPMODE = false;
    static bool check_ground = false;

    static int roll = Roll - 3;
    static int pitch = Pitch - 3;
    static float predicted_walking_time = 0;
    static int predicted_walking_step_num = 0;
    static int LFoot_step_num = 0;
    static int RFoot_step_num = 0;

    static Vector3d LFoot_Start_pos;
    static Vector3d LFoot_3;
    static Vector3d LFoot_4;
    static Vector3d LFoot_5;
    static Vector3d LFoot_End_pos;
    static Vector3d LFoot_pos;

    static Vector3d RFoot_Start_pos;
    static Vector3d RFoot_3;
    static Vector3d RFoot_4;
    static Vector3d RFoot_5;
    static Vector3d RFoot_End_pos;
    static Vector3d RFoot_pos;

    if (walking.time.sec < zmp.previewControl.PreviewReadyTime)
    { // Pattern Phase : DSP
        walking.Pattern_SP = walking.DSP;
    }
    else if (walking.time.sec < zmp.previewControl.PreviewReadyTime + walking.step.total_num * walking.time.periodTime - tasktime / 2.)
    {

        predicted_walking_time = fmodf(walking.time.sec - zmp.previewControl.PreviewReadyTime, walking.time.periodTime); // fmodf : 실수형 나머지
        predicted_walking_step_num = (int)((walking.time.sec - zmp.previewControl.PreviewReadyTime) / walking.time.periodTime + 0.0001);
        // std::cout << "predicted_walking_time :\n" << predicted_walking_time  << std::endl;
        // std::cout << "predicted_walking_step_num :\n" << predicted_walking_step_num  << std::endl;


        if (predicted_walking_time < walking.time.SSP_start_time)
        {
            walking.Pattern_SP = walking.DSP;
        }
        else if (predicted_walking_time < walking.time.SSP_end_time)
        {
            if (predicted_walking_step_num % 2 == 0)
            {
                walking.Pattern_SP = walking.step.start_swing_foot * (-1);

            }
            else
            {
                walking.Pattern_SP = walking.step.start_swing_foot;

            }

            if (walking.step.footHeight == 0)
            {
                walking.Pattern_SP = walking.DSP;
                // printf( "Foot Traj 0\n" );
            }
        }
        else
        { // < walking.time.period
            walking.Pattern_SP = walking.DSP;
        }
    }
    else
    {
        walking.Pattern_SP = walking.DSP;
    }
    // printf( "Foot Traj 1\n" );

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (walking.Pattern_SP == walking.DSP)
    {
    }
    //* LfootTraj
    else if (walking.Pattern_SP == walking.RSSP)
    {
        // printf( "L1 IF\n" );

        // if(walking.time.init_sec < walking.time.SSP_start_time)
        // {
        //         // printf( "L2 IF\n" );

        // }
        // else if(walking.time.init_sec < walking.time.SSP_end_time)
        // {
        //         // printf( "L3 IF\n" );

        //     if(LFoot_step_num == 0) {
        //          //printf( "12432t534654645352\n" );
        //     //    LFoot.refpos(X)    = cosWave(LfootPrints(X,LFoot_step_num)-walking.step.LastLfoot(X), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, walking.step.LastLfoot(X));
        //     //    LFoot.refpos(Y)    = cosWave(LfootPrints(Y,LFoot_step_num)-walking.step.LastLfoot(Y), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, walking.step.LastLfoot(Y)-body_kine.lower.LEG_SIDE_OFFSET);
        //         LFoot.refpos(Yaw)  = cosWave(LfootPrints(Yaw,LFoot_step_num)-walking.step.LastLfoot(Yaw), walking.time.SSP_time, walking.time.init_sec-walking.time.SSP_start_time, walking.step.LastLfoot(Yaw));
        //             //std::cout << "\n왼발 :\n" << LfootPrints(Yaw,currentLfootstep) <<"라스트:\n" << walking.step.LastLfoot(Yaw)  << std::endl;
        //     }

        //     else {
        //          //printf( "436rtudfhngfxseryyfyjygh\n" );
        //     //    LFoot.refpos(X)   = cosWave(LfootPrints(X,LFoot_step_num)-LfootPrints(X,LFoot_step_num-1), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, LfootPrints(X,LFoot_step_num-1));
        //     //    LFoot.refpos(Y)   = cosWave(LfootPrints(Y,LFoot_step_num)-LfootPrints(Y,LFoot_step_num-1), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, LfootPrints(Y,LFoot_step_num-1)-body_kine.lower.LEG_SIDE_OFFSET);
        //         LFoot.refpos(Yaw) = cosWave(LfootPrints(Yaw,LFoot_step_num)-LfootPrints(Yaw,LFoot_step_num-1), walking.time.SSP_time, walking.time.init_sec-walking.time.SSP_start_time, LfootPrints(Yaw,LFoot_step_num-1));
        //             //std::cout << "\n왼발2222 :\n" << LfootPrints(Yaw,currentLfootstep) <<"라스트2222:\n" << walking.step.LastLfoot(Yaw)  << std::endl;
        //     }
        // }
        // else
        // {

        // }

        //////////////////////////////////////////////////////////

        if (walking.step.start_swing_foot == walking.step.S_L_F)
            LFoot_step_num = (int)(predicted_walking_step_num / 2);
        else
            LFoot_step_num = (int)(predicted_walking_step_num / 2) + 1;

        //* 0-1-2 point set
        if (LFoot_step_num == 0)
        {
            LFoot_Start_pos(X) = walking.step.LastLfoot(X);
            LFoot_Start_pos(Y) = walking.step.LastLfoot(Y);
            LFoot_Start_pos(Z) = walking.step.LastLfoot(Z);
            LFoot.refpos(Yaw) = cosWave(LfootPrints(Yaw, LFoot_step_num) - walking.step.LastLfoot(Yaw), walking.time.SSP_time, predicted_walking_time - walking.time.SSP_start_time, walking.step.LastLfoot(Yaw));
        }
        else
        {
            LFoot_Start_pos(X) = walking.step.LFoot(X, LFoot_step_num - 1);
            LFoot_Start_pos(Y) = walking.step.LFoot(Y, LFoot_step_num - 1);
            LFoot_Start_pos(Z) = walking.step.LFoot(Z, LFoot_step_num - 1);
            LFoot.refpos(Yaw) = cosWave(LfootPrints(Yaw, LFoot_step_num) - LfootPrints(Yaw, LFoot_step_num - 1), walking.time.SSP_time, predicted_walking_time - walking.time.SSP_start_time, LfootPrints(Yaw, LFoot_step_num - 1));
        }

        //* 6-7-8 point set
        LFoot_End_pos(X) = walking.step.LFoot(X, LFoot_step_num);
        LFoot_End_pos(Y) = walking.step.LFoot(Y, LFoot_step_num);
        LFoot_End_pos(Z) = walking.step.LFoot(Z, LFoot_step_num);

        LFoot_3(X) = LFoot_Start_pos(X);
        LFoot_4(X) = (LFoot_Start_pos(X) + LFoot_End_pos(X)) / 2.;
        LFoot_5(X) = LFoot_End_pos(X);

        LFoot_3(Y) = LFoot_Start_pos(Y);
        LFoot_4(Y) = (LFoot_Start_pos(Y) + LFoot_End_pos(Y)) / 2.;
        LFoot_5(Y) = LFoot_End_pos(Y);

        LFoot_3(Z) = LFoot_Start_pos(Z) + walking.step.footHeight;
        LFoot_4(Z) = (LFoot_Start_pos(Z) + LFoot_End_pos(Z)) / 2. + 2 * walking.step.footHeight;
        LFoot_5(Z) = LFoot_End_pos(Z) + walking.step.footHeight;
        // 3-4-5 point set

        LFoot_pos = genreate_8th_BezierCurve(LFoot_Start_pos, LFoot_3, LFoot_4, LFoot_5, LFoot_End_pos, walking.time.SSP_time, predicted_walking_time - walking.time.SSP_start_time);

        LFoot.ref_G_pattern_pos(X) = LFoot_pos(X);
        LFoot.ref_G_pattern_pos(Y) = LFoot_pos(Y);
        LFoot.ref_G_pattern_pos(Z) = LFoot_pos(Z);
    }
    //* RfootTraj
    else if (walking.Pattern_SP == walking.LSSP)
    {
        // printf( "RRRRRRRRRRRRRRR\n" );

        //  if(walking.time.init_sec < walking.time.XYSSP_start_time)
        //     {

        //     }
        //     else if(walking.time.init_sec < walking.time.XYSSP_end_time)
        //     {

        //         if(RFoot_step_num == 0) {
        //         //    RFoot.refpos(X)    = cosWave(RfootPrints(X,RFoot_step_num)-walking.step.LastRfoot(X), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, walking.step.LastRfoot(X));
        //         //    RFoot.refpos(Y)    = cosWave(RfootPrints(Y,RFoot_step_num)-walking.step.LastRfoot(Y), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, walking.step.LastRfoot(Y)+body_kine.lower.LEG_SIDE_OFFSET);
        //             RFoot.refpos(Yaw)  = cosWave(RfootPrints(Yaw,RFoot_step_num)-walking.step.LastRfoot(Yaw), walking.time.XYSSP_time, walking.time.init_sec-walking.time.XYSSP_start_time, walking.step.LastRfoot(Yaw));
        //         }
        //         else {
        //         //    RFoot.refpos(X)   = cosWave(RfootPrints(X,RFoot_step_num)-RfootPrints(X,RFoot_step_num-1), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, RfootPrints(X,RFoot_step_num-1));
        //         //    RFoot.refpos(Y)   = cosWave(RfootPrints(Y,RFoot_step_num)-RfootPrints(Y,RFoot_step_num-1), walking.time.XYSSP_time, walking.time.sec-walking.time.XYSSP_start_time, RfootPrints(Y,RFoot_step_num-1)+body_kine.lower.LEG_SIDE_OFFSET);
        //             RFoot.refpos(Yaw) = cosWave(RfootPrints(Yaw,RFoot_step_num)-RfootPrints(Yaw,RFoot_step_num-1), walking.time.XYSSP_time, walking.time.init_sec-walking.time.XYSSP_start_time, RfootPrints(Yaw,RFoot_step_num-1));
        //         }
        //     }
        //     else
        //     {

        //     }

        //////////////////////////////////////////////////////////////////
        if (walking.step.start_swing_foot == walking.step.S_L_F)
            RFoot_step_num = (int)(predicted_walking_step_num / 2) + 1;
        else
            RFoot_step_num = (int)(predicted_walking_step_num / 2);

        //* 0-1-2 point set
        if (RFoot_step_num == 0)
        {
            RFoot_Start_pos(X) = walking.step.LastRfoot(X);
            RFoot_Start_pos(Y) = walking.step.LastRfoot(Y);
            RFoot_Start_pos(Z) = walking.step.LastRfoot(Z);
            RFoot.refpos(Yaw) = cosWave(RfootPrints(Yaw, RFoot_step_num) - walking.step.LastRfoot(Yaw), walking.time.SSP_time, predicted_walking_time - walking.time.SSP_start_time, walking.step.LastRfoot(Yaw));
        }
        else
        {
            RFoot_Start_pos(X) = walking.step.RFoot(X, RFoot_step_num - 1);
            RFoot_Start_pos(Y) = walking.step.RFoot(Y, RFoot_step_num - 1);
            RFoot_Start_pos(Z) = walking.step.RFoot(Z, RFoot_step_num - 1);
            RFoot.refpos(Yaw) = cosWave(RfootPrints(Yaw, RFoot_step_num) - RfootPrints(Yaw, RFoot_step_num - 1), walking.time.SSP_time, predicted_walking_time - walking.time.SSP_start_time, RfootPrints(Yaw, RFoot_step_num - 1));
        }

        //* 6-7-8 point set
        RFoot_End_pos(X) = walking.step.RFoot(X, RFoot_step_num);
        RFoot_End_pos(Y) = walking.step.RFoot(Y, RFoot_step_num);
        RFoot_End_pos(Z) = walking.step.RFoot(Z, RFoot_step_num);

        RFoot_3(X) = RFoot_Start_pos(X);
        RFoot_4(X) = (RFoot_Start_pos(X) + RFoot_End_pos(X)) / 2.;
        RFoot_5(X) = RFoot_End_pos(X);

        RFoot_3(Y) = RFoot_Start_pos(Y);
        RFoot_4(Y) = (RFoot_Start_pos(Y) + RFoot_End_pos(Y)) / 2.;
        RFoot_5(Y) = RFoot_End_pos(Y);

        RFoot_3(Z) = RFoot_Start_pos(Z) + walking.step.footHeight;
        RFoot_4(Z) = (RFoot_Start_pos(Z) + RFoot_End_pos(Z)) / 2. + 2 * walking.step.footHeight;
        RFoot_5(Z) = RFoot_End_pos(Z) + walking.step.footHeight;
        // 3-4-5 point set

        RFoot_pos = genreate_8th_BezierCurve(RFoot_Start_pos, RFoot_3, RFoot_4, RFoot_5, RFoot_End_pos, walking.time.SSP_time, predicted_walking_time - walking.time.SSP_start_time);

        RFoot.ref_G_pattern_pos(X) = RFoot_pos(X);
        RFoot.ref_G_pattern_pos(Y) = RFoot_pos(Y);
        RFoot.ref_G_pattern_pos(Z) = RFoot_pos(Z);
    }

    Base.refpos(Yaw) = (LFoot.refpos(Yaw) + RFoot.refpos(Yaw)) / 2.;

    if (zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime) / zmp.previewControl.RefTotalTime)
    {
        //  printf( "\n0 IF\n" );

        walking.time.wsec = 0;
        walking.walk_SP = walking.DSP;
        walking.walk_SSP = walking.step.start_swing_foot * -1;
        walking.step.cur_num = 0;
        walking.step.cur_num_footprints = 1;
    }
    else if (zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (walking.step.total_num) * walking.time.periodTime) / zmp.previewControl.RefTotalTime)
    {
        //  printf( "\n0.5 IF\n" );

        if (walking.time.wsec < walking.time.periodTime)
        {
            //  printf( "\n1 IF\n" );

            if (walking.time.wsec < walking.time.SSP_start_time)
            {
                //  printf( "\n2 IF\n" );

                walking.walk_SP = walking.DSP;
            }
            else if (walking.time.wsec < walking.time.SSP_end_time)
            {
                //  printf( "\n3 IF\n" );

                //                if(walking.time.sec < walking.time.SSP_start_time+tasktime)
                //                {
                //                    zmp.controller.gainchangeflag = true;
                //                    zmp.controller.dsp2ssp = true;
                //                    zmp.controller.ssp2dsp = false;
                //                }
                //                else if(walking.time.sec >= (walking.time.SSP_end_time-zmp.controller.gainChangeTime)-2*tasktime && walking.time.sec < (walking.time.SSP_end_time-zmp.controller.gainChangeTime)-tasktime)
                //                {
                //                    zmp.controller.gainchangeflag = true;
                //                    zmp.controller.dsp2ssp = false;
                //                    zmp.controller.ssp2dsp = true;

                //                }
                //                else

                //                zmpControlGainChange();

                walking.walk_SP = walking.walk_SSP;
                if (walking.walk_SP == walking.LSSP)
                {
                    walking.step.prefootCenter(X) = LfootPrints(X, LFoot_step_num);
                    walking.step.prefootCenter(Y) = LfootPrints(Y, LFoot_step_num);
                    walking.step.prefootCenter(Yaw) = LfootPrints(Yaw, LFoot_step_num);

                    // std::cout << "\nwalking.step.prefootCenter(X)\n" << walking.step.prefootCenter(X) << std::endl;
                    // std::cout << "\nwalking.step.(Y)\n" << walking.step.prefootCenter(Y) << std::endl;

                    // if(d_L_offset_Z > 0)
                    //     Base.refpos(Z) = cosWave((L_offset_Z)-Base.prerefpos(Z), walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time, Base.prerefpos(Z));

                    // if(d_R_offset_Z < 0)
                    //     Base.refpos(Z) = cosWave((d_R_offset_Z+walking.step.LastRfoot(Z))-Base.prerefpos(Z), walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time, Base.prerefpos(Z));
                }
                else if (walking.walk_SP == walking.RSSP)
                {
                    walking.step.prefootCenter(X) = RfootPrints(X, RFoot_step_num);
                    walking.step.prefootCenter(Y) = RfootPrints(Y, RFoot_step_num);
                    walking.step.prefootCenter(Yaw) = RfootPrints(Yaw, RFoot_step_num);

                    // std::cout << "\nwalking.step.prefootCenter(X)\n" << walking.step.prefootCenter(X) << std::endl;
                    // std::cout << "\nwalking.step.prefootCenter(Y)\n" << walking.step.prefootCenter(Y) << std::endl;

                    // if(d_R_offset_Z > 0)
                    //     Base.refpos(Z) = cosWave((R_offset_Z)-Base.prerefpos(Z), walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time, Base.prerefpos(Z));

                    // if(d_L_offset_Z < 0)
                    //     Base.refpos(Z) = cosWave((d_L_offset_Z+walking.step.LastLfoot(Z))-Base.prerefpos(Z), walking.time.SSP_time, walking.time.sec-walking.time.SSP_start_time, Base.prerefpos(Z));
                }
                // if(walking.time.sec >= (walking.time.periodTime)/2-tasktime)
                // {
                //     if(walking.step.current < walking.step.total-1)
                //         adjust_CoMHeight = cosWave(-25, walking.time.SSP_time/2, walking.time.sec-(walking.time.periodTime)/2-tasktime, 0);
                // }
                // else
                // {
                //     if(walking.step.current > 0)
                //         adjust_CoMHeight  = cosWave(25, walking.time.SSP_time/2, walking.time.sec-walking.time.SSP_start_time, -25);
                // }

                if (walking.time.wsec >= (walking.time.periodTime) / 2 - tasktime && walking.time.wsec < (walking.time.periodTime) / 2)
                {
                    //  printf( "\n3.5 IF\n" );

                    walking.old_walk_SSP = walking.walk_SSP;
                    // walking.Landing.EarlycheckPoint = true;
                    if (walking.ssp_mode == true)
                        SSPMODE = true;
                    else
                        SSPMODE = false;
                }
            }
            else
            {
                //  printf( "\n4 IF\n" );

                walking.walk_SP = walking.DSP;
                Base.prerefpos(Z) = Base.refpos(Z);

                if (checkstep == false)
                {
                    //  printf( "\nstep_num IF\n" );

                    walking.step.cur_num++;
                    globalPositionUpdate(LfootPrints.col(LFoot_step_num), RfootPrints.col(RFoot_step_num));
                    if (walking.walk_SSP == walking.RSSP)
                        RFoot_step_num++;
                    else if (walking.walk_SSP == walking.LSSP)
                        LFoot_step_num++;
                    walking.step.cur_num_footprints++;

                    // walking.Landing.EarlycheckPoint = false;

                    if (walking.step.cur_num != walking.step.total_num)
                    {
                        walking.walk_SSP = walking.walk_SSP * -1;
                    }
                    //                    std::cout<<"L == "<< currentLfootstep <<" R == "<< currentRfootstep <<std::endl;
                    //                    std::cout<<"walking.step.current == "<< walking.step.current <<std::endl;
                    //                    std::cout<<"L == "<< currentLfootstep <<" R == "<< currentRfootstep <<std::endl;
                    //                    std::cout<<"LX == "<< walking.step.preLfoot(X) <<" RX == "<< walking.step.preRfoot(X) <<std::endl;
                    //                    std::cout<<"LY == "<< walking.step.preLfoot(Y) <<" RY == "<< walking.step.preRfoot(Y) <<std::endl;
                    checkstep = true;
                }
            }

            //            joint[WST].compensate_refAngle = walking.step.start_swing_foot*sinWave(walking.step.FBstepSize*0.125, 2*walking.time.periodTime, walking.time.sec, 0);

            if (SSPMODE == false)
            {
                walking.time.wsec += (double)tasktime;

                //  printf( "\n5 IF\n" );
            }
            // WSTtime += (double)tasktime;
            // if(WSTtime >= 2*walking.time.periodTime)
            // {
            //     WSTtime = 0;
            // }

            //            printf("joint[WST].compensate_refAngle = %lf\n",joint[WST].compensate_refAngle);
            if (walking.time.wsec >= walking.time.periodTime - 0.001)
            {
                //  printf( "\n6 IF\n" );
                //                std::cout<<"walking.time.sec == "<< walking.time.sec <<std::endl;
                checkstep = false;
                walking.time.wsec = 0;

                //                std::cout<<"zmp.previewControl.count == "<< zmp.previewControl.count <<std::endl;

                if ((CommandFlag == ACT_INF_WALK || CommandFlag == ACT_STOPPING_WALK) && zmp.previewControl.count > zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (2) * walking.time.periodTime) / zmp.previewControl.RefTotalTime)
                {
                    //  printf( "\n7 IF\n" );

                    if (walking.old_walk_SSP == walking.RSSP)
                    {
                        RFoot_step_num--;
                    }
                    else if (walking.old_walk_SSP == walking.LSSP)
                    {
                        LFoot_step_num--;
                    }

                    walking.step.cur_num--; // 위에서는 cur_num_footprints++    했는데 여기서는 cur_num--    ..?
                    zmp.previewControl.count = (int)(zmp.previewControl.RefTotalTrajSize * (zmp.previewControl.PreviewReadyTime + (2) * walking.time.periodTime) / zmp.previewControl.RefTotalTime + 0.001) - 1;
                    walking.NewfootPlan = true;

                    //                    std::cout<<"L == "<< currentLfootstep <<" R == "<< currentRfootstep <<std::endl;
                    //                    std::cout<<"zmp.previewControl.count == "<< zmp.previewControl.count <<std::endl;
                    if (stop_msgs == 1)
                    {
                        printf("\nstop IF\n");
                        CommandFlag = ACT_STOPPING_WALK;

                        if (CommandFlag == ACT_STOPPING_WALK)
                        {
                            CommandFlag = ACT_STOP_WALK;
                            stop_msgs = 0;
                        }
                    }
                }
            }
        }
        //    std::cout<<"LFoot_step_num= "<< LFoot_step_num <<std::endl;
        //    std::cout<<"RFoot_step_num= "<< RFoot_step_num <<std::endl;
    }
    else
    {
        //        std::cout<<"L == "<< currentLfootstep <<" R == "<< currentRfootstep <<std::endl;
        //        std::cout<<"LX == "<< walking.step.preLfoot(X) <<" RX == "<< walking.step.preRfoot(X) <<std::endl;
        //        std::cout<<"LY == "<< walking.step.preLfoot(Y) <<" RY == "<< walking.step.preRfoot(Y) <<std::endl;
        //        std::cout<<"ZMPX == "<< zmp.previewControl.X.m_ref <<" ZMPY == "<< zmp.previewControl.Y.m_ref <<std::endl;

        // WSTtime = 0;
        checkstep = false;
        LFoot_step_num = 0;
        RFoot_step_num = 0;
    }

    if (SSPMODE == false)
    {
        zmp.previewControl.count++;
    }
}

void CKubot::globalPositionUpdate(VectorXd preLfoot, VectorXd preRfoot)
{

    walking.step.preLfoot = preLfoot;
    walking.step.preRfoot = preRfoot;

    walking.step.prefootCenter = (walking.step.preLfoot + walking.step.preRfoot) / 2.;

    //    std::cout<<"L = "<<std::endl<< walking.step.preLfoot <<std::endl;
    //    std::cout<<"R = "<<std::endl<< walking.step.preRfoot <<std::endl;
}

Vector3d CKubot::genreate_8th_BezierCurve(Vector3d point_0_1_2, Vector3d point_3, Vector3d point_4, Vector3d point_5, Vector3d point_6_7_8, double period, double time)
{
    VectorXd output_pos = VectorXd::Zero(3);
    double t = time / period;
    //    std::cout << "t="<<t<< std::endl;

    Eigen::Vector3d B;
    VectorXd b(9);

    Eigen::Vector3d P0, P1, P2, P3, P4, P5, P6, P7, P8;

    P0 = point_0_1_2;
    P1 = P0;
    P2 = P1;
    // Set start vel & acc to zero.

    P3 = point_3;
    P4 = point_4;
    P5 = point_5;

    P6 = point_6_7_8;
    P7 = P6;
    P8 = P7;
    // Set end vel & acc to zero.

    for (int i = 0; i < 9; i++)
    {
        b(i) = (factorial(8) / (factorial(i) * factorial(8 - i))) * pow(1 - t, 8 - i) * pow(t, i);
        //        std::cout << "b("<<i<<")"<<b(i)<<" (factorial(8)/(factorial(i)*factorial(8-i))) " <<(factorial(8)/(factorial(i)*factorial(8-i))) <<std::endl;
    }

    B = b(0) * P0 + b(1) * P1 + b(2) * P2 + b(3) * P3 + b(4) * P4 + b(5) * P5 + b(6) * P6 + b(7) * P7 + b(8) * P8;
    //    std::cout << "B="<<B<< std::endl;

    output_pos = B;

    return output_pos;
}

int CKubot::factorial(int n)
{
    if (n < 0)
    {
        std::cout << "n is a negative value" << std::endl;
    }
    else if (n == 0)
    {
        n = 1;
    }
    else
    {
        n = n * factorial(n - 1);
    }

    return n;
}

// cpp에 추가
void CKubot::save_file(int col_cnt)
{
    string line;
    ofstream file;

    for (int i = 0; i < 3; i++)
    {
        if (i == 0)
        {
            file.open("/home/park/catkin_ws/src/beginner_kubot_pkgs/src/joint.txt", ios_base::app); // 파일 열기
            if (file.is_open())
            {
                cout << "joint save start" << endl;

                // file <<"count"<<","<<"angle_ref"<<","<<"angle_act"<<"\n";
                //          cnt/        joint_ref 1~12/     joint actual 1~12
                for (int k = 1; k <= col_cnt; k++)
                {
                    for (int u = 1; u <= 25; u++)
                    {
                        file << save_jointData[u - 1][k - 1] << " ";
                    }
                    file << "\n";
                }

                cout << "joint save done" << endl;
                file.close();
            }
            else
            {
                cout << "error" << endl;
            }
        }
        else if (i == 1)
        {
            file.open("/home/park/catkin_ws/src/beginner_kubot_pkgs/src/preview.txt", ios_base::app);
            if (file.is_open())
            {
                cout << "preview save start" << endl;

                // file <<"count"<<","<<"zmp_ref"<<","<<"zmp_calc"<<"\n"<<"CoM_ref"<<","<<"FK"<<"\n";;
                //          cnt/            X,Y/            X,Y             X,Y,Z          X,Y,Z
                for (int k = 1; k <= col_cnt; k++)
                {
                    for (int u = 1; u <= 11; u++)
                    {
                        file << save_previewData[u - 1][k - 1] << " ";
                    }
                    file << "\n";
                }

                cout << "preview save done" << endl;
                file.close();
            }
            else
            {
                cout << "error" << endl;
            }
        }
        else if (i == 2)
        {
            file.open("/home/park/catkin_ws/src/beginner_kubot_pkgs/src/foot.txt", ios_base::app);
            if (file.is_open())
            {
                cout << "foot save start" << endl;

                //          cnt/        L_ref XYZ/      R_ref XYZ/     L_FK XYZ/     R_FK XYZ
                for (int k = 1; k <= col_cnt; k++)
                {
                    for (int u = 1; u <= 13; u++)
                    {
                        file << save_footData[u - 1][k - 1] << " ";
                    }
                    file << "\n";
                }

                cout << "foot save done" << endl;
                file.close();
            }
            else
            {
                cout << "error" << endl;
            }
        }
    }
}

void CKubot::changeWalkingStep(double FBsize, double LRsize, double Turnsize)
{

    if (walking.old_walk_SSP == walking.RSSP)
        walking.step.start_swing_foot = walking.step.S_R_F;
    else if (walking.old_walk_SSP == walking.LSSP)
        walking.step.start_swing_foot = walking.step.S_L_F;

    if (walking.step.oldFBstepSize != FBsize || walking.step.oldLRstepSize != LRsize || walking.step.oldTurnSize != Turnsize)
    {

        if (walking.step.oldTurnSize != Turnsize && walking.change_Turn_flag == false)
        {

            if (walking.old_walk_SSP == walking.RSSP && Turnsize >= 0)
            {
                walking.step.TurnSize = Turnsize;
                walking.change_Turn_flag = true;
            }
            else if (walking.old_walk_SSP == walking.LSSP && Turnsize <= 0)
            {
                walking.step.TurnSize = Turnsize;
                walking.change_Turn_flag = true;
            }

            if (Turnsize - walking.step.oldTurnSize > 0.2)
            {
                walking.step.TurnSize = walking.step.oldTurnSize + 0.2;
            }
            else if (Turnsize - walking.step.oldTurnSize < -0.2)
            {
                walking.step.TurnSize = walking.step.oldTurnSize - 0.2;
            }
            else
                walking.step.TurnSize = Turnsize;
        }
        else
        {
            walking.change_Turn_flag = false;
        }

        if (walking.step.oldFBstepSize != FBsize && walking.change_FB_walking_flag == false)
        {
            walking.change_FB_walking_flag = true;

            if (FBsize - walking.step.oldFBstepSize > 0.05)
            {
                walking.step.FBstepSize = walking.step.oldFBstepSize + 0.05;
            }
            else if (FBsize - walking.step.oldFBstepSize < -0.05)
            {
                walking.step.FBstepSize = walking.step.oldFBstepSize - 0.05;
            }
            else
                walking.step.FBstepSize = FBsize;
        }
        else
            walking.change_FB_walking_flag = false;

        if (walking.step.oldLRstepSize != LRsize && walking.change_LR_walking_flag == false)
        {
            if (walking.old_walk_SSP == walking.RSSP && LRsize >= 0)
            {
                walking.step.LRstepSize = LRsize;
                walking.change_LR_walking_flag = true;
            }
            else if (walking.old_walk_SSP == walking.LSSP && LRsize <= 0)
            {
                walking.step.LRstepSize = LRsize;
                walking.change_LR_walking_flag = true;
            }

            if (LRsize - walking.step.oldLRstepSize > 0.02)
            {
                walking.step.LRstepSize = walking.step.oldLRstepSize + 0.02;
            }
            else if (LRsize - walking.step.oldLRstepSize < -0.02)
            {
                walking.step.LRstepSize = walking.step.oldLRstepSize - 0.02;
            }
            else
                walking.step.LRstepSize = LRsize;
        }
        else
            walking.change_LR_walking_flag = false;

        //        printf("oldFBstepSize =%lf mm\t oldLRstepSize =%lf mm\t oldTurnSize =%lf degree\n",walking.step.oldFBstepSize,walking.step.oldLRstepSize,walking.step.oldTurnSize*R2D);
        // printf("FBstepSize =%lf mm\t LRstepSize =%lf mm\t TurnSize =%lf degree\n",walking.step.FBstepSize,walking.step.LRstepSize,walking.step.TurnSize*R2D);
    }
    else
    {
        walking.change_FB_walking_flag = false;
        walking.change_LR_walking_flag = false;
        walking.change_Turn_flag = false;
        ;
    }
}

void CKubot::FootPlannerWindowMove()
{

    if (walking.change_Turn_flag == true)
    {
        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(Yaw, 0) = walking.step.RFoot(Yaw, 1);
            walking.step.RFoot(Yaw, 1) = walking.step.RFoot(Yaw, 2);
            walking.step.LFoot(Yaw, 2) = walking.step.RFoot(Yaw, 1) + walking.step.TurnSize;
            walking.step.RFoot(Yaw, 2) = walking.step.LFoot(Yaw, 2);
            walking.step.LFoot(Yaw, 3) = walking.step.RFoot(Yaw, 2) + walking.step.TurnSize;
            walking.step.RFoot(Yaw, 3) = walking.step.LFoot(Yaw, 3);
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(Yaw, 0) = walking.step.LFoot(Yaw, 1);
            walking.step.LFoot(Yaw, 1) = walking.step.LFoot(Yaw, 2);
            walking.step.RFoot(Yaw, 2) = walking.step.LFoot(Yaw, 1) + walking.step.TurnSize;
            walking.step.LFoot(Yaw, 2) = walking.step.RFoot(Yaw, 2);
            walking.step.RFoot(Yaw, 3) = walking.step.LFoot(Yaw, 2) + walking.step.TurnSize;
            walking.step.LFoot(Yaw, 3) = walking.step.RFoot(Yaw, 3);
        }

        walking.step.oldTurnSize = walking.step.TurnSize;
    }
    else
    {
        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(Yaw, 0) = walking.step.RFoot(Yaw, 1);
            walking.step.RFoot(Yaw, 1) = walking.step.RFoot(Yaw, 2);
            walking.step.RFoot(Yaw, 2) = walking.step.RFoot(Yaw, 3);
            walking.step.RFoot(Yaw, 3) = walking.step.RFoot(Yaw, 3) + walking.step.TurnSize;
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(Yaw, 0) = walking.step.LFoot(Yaw, 1);
            walking.step.LFoot(Yaw, 1) = walking.step.LFoot(Yaw, 2);
            walking.step.LFoot(Yaw, 2) = walking.step.LFoot(Yaw, 3);
            walking.step.LFoot(Yaw, 3) = walking.step.LFoot(Yaw, 3) + walking.step.TurnSize;
        }
    }

    static Vector3d oldStep = VectorXd::Zero(3);
    static Vector3d nowStep = VectorXd::Zero(3);

    static Vector3d LRstep = VectorXd::Zero(3);
    static Vector3d RLstep = VectorXd::Zero(3);

    static VectorXd footLocal = VectorXd::Zero(6);

    oldStep(X) = 2 * walking.step.oldFBstepSize;
    oldStep(Y) = walking.step.oldLRstepSize;

    if (walking.step.LRstepSize > 0)
    {
        RLstep(X) = walking.step.FBstepSize;
        LRstep(X) = walking.step.FBstepSize;
        RLstep(Y) = walking.step.LRstepSize + 2 * body_kine.lower.LEG_SIDE_OFFSET;
        LRstep(Y) = -2 * body_kine.lower.LEG_SIDE_OFFSET;
    }
    else if (walking.step.LRstepSize < 0)
    {
        RLstep(X) = walking.step.FBstepSize;
        LRstep(X) = walking.step.FBstepSize;
        RLstep(Y) = 2 * body_kine.lower.LEG_SIDE_OFFSET;
        LRstep(Y) = walking.step.LRstepSize + -2 * body_kine.lower.LEG_SIDE_OFFSET;
    }
    else
    {
        RLstep(X) = walking.step.FBstepSize;
        LRstep(X) = walking.step.FBstepSize;
        RLstep(Y) = 2 * body_kine.lower.LEG_SIDE_OFFSET;
        LRstep(Y) = -2 * body_kine.lower.LEG_SIDE_OFFSET;
    }

    if (walking.change_FB_walking_flag == true || walking.change_LR_walking_flag == true || walking.change_Turn_flag == true)
    {
        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(X, 0) = walking.step.RFoot(X, 1);
            walking.step.RFoot(X, 1) = walking.step.RFoot(X, 2);
            walking.step.LFoot(X, 2) = walking.step.RFoot(X, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(X);
            walking.step.RFoot(X, 2) = walking.step.LFoot(X, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(X);
            walking.step.LFoot(X, 3) = walking.step.RFoot(X, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(X);
            walking.step.RFoot(X, 3) = walking.step.LFoot(X, 3) + local2Global(LRstep, walking.step.LFoot.col(3))(X);

            walking.step.RFoot(Y, 0) = walking.step.RFoot(Y, 1);
            walking.step.RFoot(Y, 1) = walking.step.RFoot(Y, 2);
            walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(Y);
            walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(Y);
            walking.step.LFoot(Y, 3) = walking.step.RFoot(Y, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(Y);
            walking.step.RFoot(Y, 3) = walking.step.LFoot(Y, 3) + local2Global(LRstep, walking.step.LFoot.col(3))(Y);
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(X, 0) = walking.step.LFoot(X, 1);
            walking.step.LFoot(X, 1) = walking.step.LFoot(X, 2);
            walking.step.RFoot(X, 2) = walking.step.LFoot(X, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(X);
            walking.step.LFoot(X, 2) = walking.step.RFoot(X, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(X);
            walking.step.RFoot(X, 3) = walking.step.LFoot(X, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(X);
            walking.step.LFoot(X, 3) = walking.step.RFoot(X, 3) + local2Global(RLstep, walking.step.RFoot.col(3))(X);

            walking.step.LFoot(Y, 0) = walking.step.LFoot(Y, 1);
            walking.step.LFoot(Y, 1) = walking.step.LFoot(Y, 2);
            walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(Y);
            walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(Y);
            walking.step.RFoot(Y, 3) = walking.step.LFoot(Y, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(Y);
            walking.step.LFoot(Y, 3) = walking.step.RFoot(Y, 3) + local2Global(RLstep, walking.step.RFoot.col(3))(Y);
        }
        if (walking.change_FB_walking_flag == true)
        {
            walking.step.oldFBstepSize = walking.step.FBstepSize;
            walking.change_FB_walking_flag = false;
        }
        if (walking.change_LR_walking_flag == true)
        {
            walking.step.oldLRstepSize = walking.step.LRstepSize;
            walking.change_LR_walking_flag = false;
        }
        if (walking.change_Turn_flag == true)
        {
            walking.change_Turn_flag = false;
        }
    }
    else
    {
        // printf("=========집가자========\n");

        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(X, 0) = walking.step.RFoot(X, 1);
            walking.step.RFoot(X, 1) = walking.step.RFoot(X, 2);
            walking.step.RFoot(X, 2) = walking.step.RFoot(X, 3);
            walking.step.RFoot(X, 3) = walking.step.LFoot(X, 3) + local2Global(LRstep, walking.step.LFoot.col(3))(X);

            walking.step.RFoot(Y, 0) = walking.step.RFoot(Y, 1);
            walking.step.RFoot(Y, 1) = walking.step.RFoot(Y, 2);
            walking.step.RFoot(Y, 2) = walking.step.RFoot(Y, 3);
            walking.step.RFoot(Y, 3) = walking.step.LFoot(Y, 3) + local2Global(LRstep, walking.step.LFoot.col(3))(Y);

            std::cout << "\nwalking.step.RFoot\n" << walking.step.RFoot << std::endl;
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(X, 0) = walking.step.LFoot(X, 1);
            walking.step.LFoot(X, 1) = walking.step.LFoot(X, 2);
            walking.step.LFoot(X, 2) = walking.step.LFoot(X, 3);
            walking.step.LFoot(X, 3) = walking.step.RFoot(X, 3) + local2Global(RLstep, walking.step.RFoot.col(3))(X);

            walking.step.LFoot(Y, 0) = walking.step.LFoot(Y, 1);
            walking.step.LFoot(Y, 1) = walking.step.LFoot(Y, 2);
            walking.step.LFoot(Y, 2) = walking.step.LFoot(Y, 3);
            walking.step.LFoot(Y, 3) = walking.step.RFoot(Y, 3) + local2Global(RLstep, walking.step.RFoot.col(3))(Y);

            std::cout << "\nwalking.step.LFoot\n" << walking.step.LFoot << std::endl;
        }
    }
}

///////////////////////////////////////////////////////////////////////////
//////////////////////////////upper body///////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void CKubot::standupFront(float standupTime)
{

    static float STUtime = 0;
    static float currentAngle[20]; // Now nDoF = 20

    if (STUtime >= 0 && motion_1 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = 0 * D2R;    // L_Hip_pitch_joint
        standupAngle[3] = 0 * D2R;    // L_Knee_joint
        standupAngle[4] = -90 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = 0 * D2R;    // R_Hip_pitch_joint
        standupAngle[9] = 0 * D2R;    // R_Knee_joint
        standupAngle[10] = -90 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = (55 * D2R);  // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (55 * D2R);  // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_1 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_1 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_1 = true;
            motion_2 = false;
        }
    }

    else if (STUtime >= 0 && motion_2 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = 0 * D2R;    // L_Hip_pitch_joint
        standupAngle[3] = 0 * D2R;    // L_Knee_joint
        standupAngle[4] = -90 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = 0 * D2R;    // R_Hip_pitch_joint
        standupAngle[9] = 0 * D2R;    // R_Knee_joint
        standupAngle[10] = -90 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;  // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;  // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;  // L_hand_pitch_joint
        standupAngle[15] = -0 * D2R;  // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R; // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;  // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = -10 * D2R; // head_pitch_joint

        if (STUtime == 0 && motion_2 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_2 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_2 = true;
            motion_3 = false;
        }
    }

    else if (STUtime >= 0 && motion_3 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = 0 * D2R;    // L_Hip_pitch_joint
        standupAngle[3] = 0 * D2R;    // L_Knee_joint
        standupAngle[4] = -90 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = 0 * D2R;    // R_Hip_pitch_joint
        standupAngle[9] = 0 * D2R;    // R_Knee_joint
        standupAngle[10] = -90 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = -180 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;   // L_hand_pitch_joint
        standupAngle[15] = -180 * D2R; // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R;  // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;   // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;    // Neck_yaw_joint
        standupAngle[19] = -10 * D2R;  // head_pitch_joint

        if (STUtime == 0 && motion_3 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_3 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_3 = true;
            motion_4 = false;
        }
    }

    else if (STUtime >= 0 && motion_4 == false)
    {

        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = 0 * D2R;    // L_Hip_pitch_joint
        standupAngle[3] = 0 * D2R;    // L_Knee_joint
        standupAngle[4] = -90 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = 0 * D2R;    // R_Hip_pitch_joint
        standupAngle[9] = 0 * D2R;    // R_Knee_joint
        standupAngle[10] = -90 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = -180 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;   // L_hand_pitch_joint
        standupAngle[15] = -180 * D2R; // R_shoulder_pitch_joint
        standupAngle[16] = 0 * D2R;    // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;   // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;    // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;    // head_pitch_joint
        // std::cout << "STUtime : \n" << STUtime << std::endl;

        if (STUtime == 0 && motion_4 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_4 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_4 = true;
            motion_5 = false;
        }
    }

    else if (STUtime >= 0 && motion_5 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = -80 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 120 * D2R;  // L_Knee_joint
        standupAngle[4] = -80 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = -80 * D2R;  // R_Hip_pitch_joint
        standupAngle[9] = 120 * D2R;  // R_Knee_joint
        standupAngle[10] = -80 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = -75 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -15 * D2R; // L_hand_pitch_joint
        standupAngle[15] = -75 * D2R; // R_shoulder_pitch_joint
        standupAngle[16] = 0 * D2R;   // R_elbow_roll_joint
        standupAngle[17] = -15 * D2R; // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint
        // std::cout << "STUtime : \n" << STUtime << std::endl;

        if (STUtime == 0 && motion_5 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_5 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_5 = true;
            motion_6 = false;
        }
    }

    else if (STUtime >= 0 && motion_6 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = -55 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 135 * D2R;  // L_Knee_joint
        standupAngle[4] = -92 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = -55 * D2R;  // R_Hip_pitch_joint
        standupAngle[9] = 135 * D2R;  // R_Knee_joint
        standupAngle[10] = -92 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = 0 * D2R;  // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;  // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R; // L_hand_pitch_joint
        standupAngle[15] = 0 * D2R;  // R_shoulder_pitch_joint
        standupAngle[16] = -0 * D2R; // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R; // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;  // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;  // head_pitch_joint
        // std::cout << "STUtime : \n" << STUtime << std::endl;

        if (STUtime == 0 && motion_6 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_6 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_6 = true;
            motion_7 = false;
        }
    }

    else if (STUtime >= 0 && motion_7 == false)
    {
        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {

            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_7 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_7 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_7 = true;
            STANDUP_ING = true;
        }
    }

    else
    {
        motion_1 = false;
        motion_2 = true;
        motion_3 = true;
        motion_4 = true;
        motion_5 = true;
        motion_6 = true;
        motion_7 = true;
    }
}

void CKubot::standupBack(float standupTime)
{

    static float STUtime = 0;
    static float currentAngle[20]; // Now nDoF = 20

    if (STUtime >= 0 && motion_1 == false)
    {
        standupAngle[0] = 0 * D2R;   // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -30 * D2R; // L_Hip_pitch_joint
        standupAngle[3] = 90 * D2R;  // L_Knee_joint
        standupAngle[4] = 30 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;   // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;   // R_Hip_roll_joint
        standupAngle[8] = -30 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 90 * D2R;  // R_Knee_joint
        standupAngle[10] = 30 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;  // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;  // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;   // L_hand_pitch_joint
        standupAngle[15] = -0 * D2R;  // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R; // R_elbow_roll_joint
        standupAngle[17] = 0 * D2R;   // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint

        if (STUtime == 0 && motion_1 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_1 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_1 = true;
            motion_2 = false;
        }
    }

    else if (STUtime >= 0 && motion_2 == false)
    {
        standupAngle[0] = 0 * D2R;   // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -70 * D2R; // L_Hip_pitch_joint
        standupAngle[3] = 90 * D2R;  // L_Knee_joint
        standupAngle[4] = 35 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;   // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;   // R_Hip_roll_joint
        standupAngle[8] = -70 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 90 * D2R;  // R_Knee_joint
        standupAngle[10] = 35 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;   // L_shoulder_pitch_joint
        standupAngle[13] = 180 * D2R;  // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;    // L_hand_pitch_joint
        standupAngle[15] = -0 * D2R;   // R_shoulder_pitch_joint
        standupAngle[16] = -180 * D2R; // R_elbow_roll_joint
        standupAngle[17] = 0 * D2R;    // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;    // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;    // head_pitch_joint

        if (STUtime == 0 && motion_2 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_2 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_2 = true;
            motion_3 = false;
        }
    }

    else if (STUtime >= 0 && motion_3 == false)
    {
        standupAngle[0] = 0 * D2R;   // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -10 * D2R; // L_Hip_pitch_joint
        standupAngle[3] = 100 * D2R; // L_Knee_joint
        standupAngle[4] = 0 * D2R;   // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;   // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;   // R_Hip_roll_joint
        standupAngle[8] = -10 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 100 * D2R; // R_Knee_joint
        standupAngle[10] = 0 * D2R;  // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;   // L_shoulder_pitch_joint
        standupAngle[13] = 180 * D2R;  // L_elbow_roll_joint
        standupAngle[14] = 90 * D2R;   // L_hand_pitch_joint
        standupAngle[15] = -0 * D2R;   // R_shoulder_pitch_joint
        standupAngle[16] = -180 * D2R; // R_elbow_roll_joint
        standupAngle[17] = 90 * D2R;   // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;    // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;    // head_pitch_joint

        if (STUtime == 0 && motion_3 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_3 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_3 = true;
            motion_4 = false;
        }
    }

    else if (STUtime >= 0 && motion_4 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = 30 * D2R;   // L_Hip_pitch_joint
        standupAngle[3] = 75 * D2R;   // L_Knee_joint
        standupAngle[4] = -30 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = 30 * D2R;   // R_Hip_pitch_joint
        standupAngle[9] = 75 * D2R;   // R_Knee_joint
        standupAngle[10] = -30 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = -90 * D2R;  // L_shoulder_pitch_joint
        standupAngle[13] = 180 * D2R;  // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;    // L_hand_pitch_joint
        standupAngle[15] = -90 * D2R;  // R_shoulder_pitch_joint
        standupAngle[16] = -180 * D2R; // R_elbow_roll_joint
        standupAngle[17] = 0 * D2R;    // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;    // Nfeck_yaw_joint
        standupAngle[19] = 0 * D2R;    // head_pitch_joint

        if (STUtime == 0 && motion_4 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_4 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_4 = true;
            motion_5 = false;
        }
    }

    else if (STUtime >= 0 && motion_5 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = 45 * D2R;   // L_Hip_pitch_joint
        standupAngle[3] = 90 * D2R;   // L_Knee_joint
        standupAngle[4] = -75 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = 45 * D2R;   // R_Hip_pitch_joint
        standupAngle[9] = 90 * D2R;   // R_Knee_joint
        standupAngle[10] = -75 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = -90 * D2R;  // L_shoulder_pitch_joint
        standupAngle[13] = 180 * D2R;  // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;    // L_hand_pitch_joint
        standupAngle[15] = -90 * D2R;  // R_shoulder_pitch_joint
        standupAngle[16] = -180 * D2R; // R_elbow_roll_joint
        standupAngle[17] = 0 * D2R;    // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;    // Nfeck_yaw_joint
        standupAngle[19] = 0 * D2R;    // head_pitch_joint

        if (STUtime == 0 && motion_5 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_5 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_5 = true;
            motion_6 = false;
        }
    }

    else if (STUtime >= 0 && motion_6 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;    // L_Hip_roll_joint
        standupAngle[2] = 15 * D2R;   // L_Hip_pitch_joint
        standupAngle[3] = 75 * D2R;   // L_Knee_joint
        standupAngle[4] = -55 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 0 * D2R;    // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 0 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = 15 * D2R;   // R_Hip_pitch_joint
        standupAngle[9] = 75 * D2R;   // R_Knee_joint
        standupAngle[10] = -55 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;   // R_Ankle_roll_joint

        standupAngle[12] = -30 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;  // L_hand_pitch_joint
        standupAngle[15] = -30 * D2R; // R_shoulder_pitch_joint
        standupAngle[16] = 0 * D2R;   // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;  // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Nfeck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint

        if (STUtime == 0 && motion_6 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_6 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_6 = true;
            motion_7 = false;
        }
    }

    else if (STUtime >= 0 && motion_7 == false)
    {

        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {

            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (0 * D2R);   // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (55 * D2R);  // head_pitch_joint

        if (STUtime == 0 && motion_7 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_7 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_7 = true;
            STANDUP_ING = true;
        }
    }
    else
    {
        motion_1 = false;
        motion_2 = true;
        motion_3 = true;
        motion_4 = true;
        motion_5 = true;
        motion_6 = true;
        motion_7 = true;
    }
}

void CKubot::hurdlemotion(float standupTime)
{

    static float STUtime = 0;
    static float currentAngle[20]; // Now nDoF = 20

    if (STUtime >= 0 && motion_1 == false)
    {

        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_1 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_1 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_1 = true;
            motion_2 = false;
        }
    }

    else if (STUtime >= 0 && motion_2 == false)
    {

        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = 0 * D2R;     // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;     // L_hand_pitch_joint
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;     // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;     // head_pitch_joint

        if (STUtime == 0 && motion_2 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_2 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_2 = true;
            motion_3 = false;
        }
    }

    else if (STUtime >= 0 && motion_3 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = -10 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -40 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 80 * D2R;   // L_Knee_joint
        standupAngle[4] = -40 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 18 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = -10 * D2R;  // R_Hip_roll_joint
        standupAngle[8] = -38 * D2R;  // R_Hip_pitch_joint
        standupAngle[9] = 76 * D2R;   // R_Knee_joint
        standupAngle[10] = -38 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = 0 * D2R;     // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;     // L_hand_pitch_joint
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;     // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;     // head_pitch_joint

        if (STUtime == 0 && motion_3 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_3 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_3 = true;
            motion_4 = false;
        }
    }

    else if (STUtime >= 0 && motion_4 == false)
    {
        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = -12 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -40 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 80 * D2R;   // L_Knee_joint
        standupAngle[4] = -40 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 18 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = -20 * D2R;  // R_Hip_roll_joint
        standupAngle[8] = -60 * D2R;  // R_Hip_pitch_joint
        standupAngle[9] = 120 * D2R;  // R_Knee_joint
        standupAngle[10] = -60 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;    // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;     // L_hand_pitch_joint
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;     // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;     // head_pitch_joint

        if (STUtime == 0 && motion_4 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_4 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_4 = true;
            motion_5 = false;
        }
    }

        else if (STUtime >= 0 && motion_5 == false)
    {
        standupAngle[0] = -20 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = -12 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -40 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 80 * D2R;   // L_Knee_joint
        standupAngle[4] = -40 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 18 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = -20 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = -20 * D2R;  // R_Hip_roll_joint
        standupAngle[8] = -60 * D2R;  // R_Hip_pitch_joint
        standupAngle[9] = 120 * D2R;  // R_Knee_joint
        standupAngle[10] = -60 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;    // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;     // L_hand_pitch_joint
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;     // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;     // head_pitch_joint

        if (STUtime == 0 && motion_5 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_5 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_5 = true;
            motion_6 = false;
        }
    }

    else if (STUtime >= 0 && motion_6 == false)
    {
        standupAngle[0] = -20 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = -12 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -30 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 60 * D2R;   // L_Knee_joint
        standupAngle[4] = -30 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 18 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = -20 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = -20 * D2R;  // R_Hip_roll_joint
        standupAngle[8] = -90 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 120 * D2R;  // R_Knee_joint
        standupAngle[10] = -50 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;    // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;     // L_hand_pitch_joint
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;     // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;     // head_pitch_joint

        if (STUtime == 0 && motion_6 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_6 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_6 = true;
            motion_7 = false;
        }
    }

    else if (STUtime >= 0 && motion_7 == false)
    {

        standupAngle[0] = -20 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = -12 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -40 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 80 * D2R;   // L_Knee_joint
        standupAngle[4] = -40 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 18 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = -20 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = -0 * D2R;  // R_Hip_roll_joint
        standupAngle[8] = -105 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 50 * D2R;   // R_Knee_joint
        standupAngle[10] = -40 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;    // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;     // L_hand_pitch_joint
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;     // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;     // head_pitch_joint

        if (STUtime == 0 && motion_7 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_7 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_7 = true;
            motion_8 = false;
        }
    }

    else if (STUtime >= 0 && motion_8 == false) // 여기서 앵클 피치 조금 기울이기
    {

        standupAngle[0] = -20 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = -5 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -55 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 110 * D2R;  // L_Knee_joint
        standupAngle[4] = -55 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = 18 * D2R;   // L_Ankle_roll_joint
        standupAngle[6] = -20 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = -18 * D2R;  // R_Hip_roll_joint
        standupAngle[8] = -75 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 50 * D2R;   // R_Knee_joint
        standupAngle[10] = 10 * D2R;  // R_Ankle_pitch_joint
        standupAngle[11] = 30 * D2R;  // R_Ankle_roll_joint

        standupAngle[12] = -0 * D2R;    // L_shoulder_pitch_joint
        standupAngle[13] = 90 * D2R;    // L_elbow_roll_joint
        standupAngle[14] = 0 * D2R;     // L_hand_pitch_joint
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;     // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;     // head_pitch_joint

        if (STUtime == 0 && motion_8 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_8 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_8 = true;
            motion_9 = false;
        }
    }

    else if (STUtime >= 0 && motion_9 == false)
    {

        // standupAngle[0] = -20 * D2R;    // L_Hip_yaw_joint
        // standupAngle[1] = -5 * D2R;   // L_Hip_roll_joint
        // standupAngle[2] = -55 * D2R;  // L_Hip_pitch_joint
        // standupAngle[3] = 110 * D2R;  // L_Knee_joint
        // standupAngle[4] = -55 * D2R;  // L_Ankle_pitch_joint
        // standupAngle[5] = 18 * D2R;   // L_Ankle_roll_joint
        // standupAngle[6] = -20 * D2R;    // R_Hip_yaw_joint
        // standupAngle[7] = -18 * D2R;  // R_Hip_roll_joint
        // standupAngle[8] = -75 * D2R; // R_Hip_pitch_joint
        // standupAngle[9] = 50 * D2R;   // R_Knee_joint
        // standupAngle[10] = 10 * D2R;  // R_Ankle_pitch_joint
        // standupAngle[11] = 30 * D2R;  // R_Ankle_roll_joint

        standupAngle[0] = -0 * D2R;   // L_Hip_yaw_joint
        standupAngle[1] = -5 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -30 * D2R; // L_Hip_pitch_joint
        standupAngle[3] = 50 * D2R;  // L_Knee_joint
        standupAngle[4] = -45 * D2R; // L_Ankle_pitch_joint
        standupAngle[5] = -4 * D2R;  // L_Ankle_roll_joint
        standupAngle[6] = -0 * D2R;   // R_Hip_yaw_joint
        standupAngle[7] = -8 * D2R;   // R_Hip_roll_joint
        standupAngle[8] = -75 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 50 * D2R;  // R_Knee_joint
        standupAngle[10] = 5 * D2R;  // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R; // R_Ankle_roll_joint

        standupAngle[12] = -90 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;  // L_hand_pitch_joint
        standupAngle[15] = 0 * D2R;   // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R; // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;  // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint

        if (STUtime == 0 && motion_9 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_9 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_9 = true;
            motion_10 = false;
        }
    }

    else if (STUtime >= 0 && motion_10 == false)
    {

        standupAngle[0] = -0 * D2R;   // L_Hip_yaw_joint
        standupAngle[1] = -5 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -30 * D2R; // L_Hip_pitch_joint
        standupAngle[3] = 50 * D2R;  // L_Knee_joint
        standupAngle[4] = -45 * D2R; // L_Ankle_pitch_joint
        standupAngle[5] = -4 * D2R;  // L_Ankle_roll_joint
        standupAngle[6] = -0 * D2R;   // R_Hip_yaw_joint
        standupAngle[7] = -8 * D2R;   // R_Hip_roll_joint
        standupAngle[8] = -85 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 75 * D2R;  // R_Knee_joint
        standupAngle[10] = -20 * D2R;  // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R; // R_Ankle_roll_joint

        standupAngle[12] = -90 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;  // L_hand_pitch_joint
        standupAngle[15] = 0 * D2R;   // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R; // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;  // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint

        if (STUtime == 0 && motion_10 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_10 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_10 = true;
            motion_11 = false;
        }
    }

    else if (STUtime >= 0 && motion_11 == false)
    {

        standupAngle[0] = -0 * D2R;   // L_Hip_yaw_joint
        standupAngle[1] = -5 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -30 * D2R; // L_Hip_pitch_joint
        standupAngle[3] = 90 * D2R;  // L_Knee_joint
        standupAngle[4] = -45 * D2R; // L_Ankle_pitch_joint
        standupAngle[5] = -4 * D2R;  // L_Ankle_roll_joint
        standupAngle[6] = -0 * D2R;   // R_Hip_yaw_joint
        standupAngle[7] = -15 * D2R;   // R_Hip_roll_joint
        standupAngle[8] = -85 * D2R; // R_Hip_pitch_joint
        standupAngle[9] = 75 * D2R;  // R_Knee_joint
        standupAngle[10] = -30 * D2R;  // R_Ankle_pitch_joint
        standupAngle[11] = 0 * D2R; // R_Ankle_roll_joint

        standupAngle[12] = -90 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;  // L_hand_pitch_joint
        standupAngle[15] = 0 * D2R;   // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R; // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;  // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint

        if (STUtime == 0 && motion_11 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_11 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_11 = true;
            motion_12 = false;
        }
    }

    else if (STUtime >= 0 && motion_12 == false)
    {

        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 0 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -60 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 120 * D2R;  // L_Knee_joint
        standupAngle[4] = -60 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = -0 * D2R;  // L_Ankle_roll_joint
        standupAngle[6] = -15 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = -20 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = -40 * D2R;  // R_Hip_pitch_joint
        standupAngle[9] = 80 * D2R;   // R_Knee_joint
        standupAngle[10] = -40 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = -0 * D2R; // R_Ankle_roll_joint

        standupAngle[12] = -90 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;  // L_hand_pitch_joint
        standupAngle[15] = 0 * D2R;   // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R; // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;  // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint

        if (STUtime == 0 && motion_12 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_12 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_12 = true;
            motion_13 = false;
        }
    }

    else if (STUtime >= 0 && motion_13 == false)
    {

        standupAngle[0] = 0 * D2R;    // L_Hip_yaw_joint
        standupAngle[1] = 10 * D2R;   // L_Hip_roll_joint
        standupAngle[2] = -38 * D2R;  // L_Hip_pitch_joint
        standupAngle[3] = 76 * D2R;   // L_Knee_joint
        standupAngle[4] = -38 * D2R;  // L_Ankle_pitch_joint
        standupAngle[5] = -0 * D2R;  // L_Ankle_roll_joint
        standupAngle[6] = 0 * D2R;    // R_Hip_yaw_joint
        standupAngle[7] = 10 * D2R;    // R_Hip_roll_joint
        standupAngle[8] = -40 * D2R;  // R_Hip_pitch_joint
        standupAngle[9] = 80 * D2R;   // R_Knee_joint
        standupAngle[10] = -40 * D2R; // R_Ankle_pitch_joint
        standupAngle[11] = -18 * D2R; // R_Ankle_roll_joint

        standupAngle[12] = -90 * D2R; // L_shoulder_pitch_joint
        standupAngle[13] = 0 * D2R;   // L_elbow_roll_joint
        standupAngle[14] = -0 * D2R;  // L_hand_pitch_joint
        standupAngle[15] = 0 * D2R;   // R_shoulder_pitch_joint
        standupAngle[16] = -90 * D2R; // R_elbow_roll_joint
        standupAngle[17] = -0 * D2R;  // R_hand_pitch_joint
        standupAngle[18] = 0 * D2R;   // Neck_yaw_joint
        standupAngle[19] = 0 * D2R;   // head_pitch_joint

        if (STUtime == 0 && motion_13 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_13 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_13 = true;
            motion_14 = false;
        }
    }

    else if (STUtime >= 0 && motion_14 == false)
    {

        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.4;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {

            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_14 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_14 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_14 = true;
            MOTION_ING = true;
        }
    }

    else
    {
        motion_1 = false;
        motion_2 = true;
        motion_3 = true;
        motion_4 = true;
        motion_5 = true;
        motion_6 = true;
        motion_7 = true;
        motion_8 = true;
        motion_9 = true;
        motion_10 = true;
        motion_11 = true;
        motion_12 = true;
        motion_13 = true;
        motion_14 = true;
    }
}


void CKubot::FootStepsPlanner(MatrixXd LfootPrints, MatrixXd RfootPrints,
                              const double step_y_offset)
{
    /* FootPlanner, Planning Footsteps
     * 
     * main processing:  
     * 1. Planning Footsteps
     */

    double step_offset = 0.0; // lowerbody.LEG_SIDE_OFFSET : 105, lowerbody.foot_y_length :125 (Foot Size)
    static int L_step_total_footpritns, R_step_total_footpritns;
    int m_L_footprints = 0, m_R_footprints = 0;

    if (walking.step.total_footprints % 2 == 0) { // Step numbers are even.
        LfootPrints = MatrixXd::Zero(PatternElement, walking.step.total_footprints / 2);
        RfootPrints = MatrixXd::Zero(PatternElement, walking.step.total_footprints / 2);
    }
    else {
        if (walking.step.start_swing_foot == walking.step.S_L_F) {
            L_step_total_footpritns = (int) ((float) (walking.step.total_footprints) / 2.);
            R_step_total_footpritns = (int) ((float) (walking.step.total_footprints) / 2.) + 1;
            LfootPrints = MatrixXd::Zero(PatternElement, L_step_total_footpritns);
            RfootPrints = MatrixXd::Zero(PatternElement, R_step_total_footpritns);
        }
        else if (walking.step.start_swing_foot == walking.step.S_R_F) {
            L_step_total_footpritns = (int) ((float) (walking.step.total_footprints) / 2.) + 1;
            R_step_total_footpritns = (int) ((float) (walking.step.total_footprints) / 2.);
            LfootPrints = MatrixXd::Zero(PatternElement, L_step_total_footpritns);
            RfootPrints = MatrixXd::Zero(PatternElement, R_step_total_footpritns);
        }
    }

    for (int m_footprints = 0; m_footprints < walking.step.total_footprints; m_footprints++) {
        if (m_footprints != 0 && m_footprints < walking.step.total_footprints - 2)
            step_offset = step_y_offset;
        else
            step_offset = 0;
        if (walking.step.start_swing_foot == walking.step.S_L_F) {
            if (m_footprints == walking.step.total_footprints - 1) {
                if (walking.step.total_footprints % 2 == 1) {
                    RfootPrints(X, m_R_footprints) = (1 - walking.step.walking_Turn)*(LfootPrints(X, m_R_footprints - 1)) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints - 1)*-walking.step.TurnSize));
                    RfootPrints(Y, m_R_footprints) = (1 - walking.step.walking_Turn)*(LfootPrints(Y, m_R_footprints - 1) + 2 * body_kine.lower.R.Base_to_Foot(Y)) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints - 1)*-walking.step.TurnSize) + walking.step.turningPoint);
                    RfootPrints(Yaw, m_R_footprints) = (m_footprints - 1) * walking.step.TurnSize;
                }
                else {
                    LfootPrints(X, m_L_footprints) = (1 - walking.step.walking_Turn) * RfootPrints(X, m_L_footprints) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints - 1)*-walking.step.TurnSize));
                    LfootPrints(Y, m_L_footprints) = (1 - walking.step.walking_Turn)*((m_L_footprints) * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y)) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints - 1)*-walking.step.TurnSize) + walking.step.turningPoint);
                    LfootPrints(Yaw, m_L_footprints) = (m_footprints - 1) * walking.step.TurnSize;
                }
            }
            else if (m_footprints % 2 == 0) { // Right Foot Step
                if (m_footprints == 0) {
                    RfootPrints(X, 0) = 0;
                    RfootPrints(Y, 0) = body_kine.lower.R.Base_to_Foot(Y);
                    RfootPrints(Yaw, 0) = 0;
                    m_R_footprints++;
                }
                else {
                    RfootPrints(X, m_R_footprints) = (1 - walking.step.walking_Turn)*((m_footprints) * walking.step.FBstepSize) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints)*-walking.step.TurnSize));
                    RfootPrints(Y, m_R_footprints) = (1 - walking.step.walking_Turn)*((m_R_footprints) * walking.step.LRstepSize + body_kine.lower.R.Base_to_Foot(Y) + step_offset) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints)*-walking.step.TurnSize) + walking.step.turningPoint);
                    RfootPrints(Yaw, m_R_footprints) = (m_footprints) * walking.step.TurnSize;
                    m_R_footprints++;
                }
            }
            else { // Left Foot Step
                LfootPrints(X, m_L_footprints) = (1 - walking.step.walking_Turn)*((m_footprints) * walking.step.FBstepSize) \
                                                + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints)*-walking.step.TurnSize));
                LfootPrints(Y, m_L_footprints) = (1 - walking.step.walking_Turn)*((m_L_footprints + 1) * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y) - step_offset) \
                                                + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints)*-walking.step.TurnSize) + walking.step.turningPoint);
                LfootPrints(Yaw, m_L_footprints) = (m_footprints) * walking.step.TurnSize;
                m_L_footprints++;
            }
        }
        else if (walking.step.start_swing_foot == walking.step.S_R_F) {
            if (m_footprints == walking.step.total_footprints - 1) {
                if (walking.step.total_footprints % 2 == 1) {

                    LfootPrints(X, m_L_footprints) = (1 - walking.step.walking_Turn)*(RfootPrints(X, m_L_footprints - 1)) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints - 1)*-walking.step.TurnSize));
                    LfootPrints(Y, m_L_footprints) = (1 - walking.step.walking_Turn)*(RfootPrints(Y, m_L_footprints - 1) + 2 * body_kine.lower.L.Base_to_Foot(Y)) \
                                                    + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints - 1)*-walking.step.TurnSize) + walking.step.turningPoint);
                    LfootPrints(Yaw, m_L_footprints) = (m_footprints - 1) * walking.step.TurnSize;
                }
                else {
                    RfootPrints(X, m_R_footprints) = (1 - walking.step.walking_Turn)*(LfootPrints(X, m_R_footprints)) \
                                             + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints - 1)*-walking.step.TurnSize));
                    RfootPrints(Y, m_R_footprints) = (1 - walking.step.walking_Turn)*(m_R_footprints * walking.step.LRstepSize + body_kine.lower.R.Base_to_Foot(Y)) \
                                             + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints - 1)*-walking.step.TurnSize) + walking.step.turningPoint);
                    RfootPrints(Yaw, m_R_footprints) = (m_footprints - 1) * walking.step.TurnSize;
                }
            }
            else if (m_footprints % 2 == 0) { // Left Foot Step
                if (m_footprints == 0) {
                    LfootPrints(X, 0) = 0;
                    LfootPrints(Y, 0) = body_kine.lower.L.Base_to_Foot(Y);
                    LfootPrints(Yaw, 0) = 0;
                    m_L_footprints++;
                }
                else {
                    LfootPrints(X, m_footprints / 2) = (1 - walking.step.walking_Turn)*((m_footprints) * walking.step.FBstepSize) \
                                         + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints)*-walking.step.TurnSize));
                    LfootPrints(Y, m_footprints / 2) = (1 - walking.step.walking_Turn)*((m_footprints / 2) * walking.step.LRstepSize + body_kine.lower.L.Base_to_Foot(Y) - step_offset) \
                                         + (walking.step.walking_Turn)*((body_kine.lower.L.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints)*-walking.step.TurnSize) + walking.step.turningPoint);
                    LfootPrints(Yaw, m_footprints / 2) = (m_footprints) * walking.step.TurnSize;
                    m_L_footprints++;
                }
            }
            else { // Right Foot Ste
                RfootPrints(X, m_R_footprints) = (1 - walking.step.walking_Turn)*((m_footprints) * walking.step.FBstepSize) \
                                         + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * sin((m_footprints)*-walking.step.TurnSize));
                RfootPrints(Y, m_R_footprints) = (1 - walking.step.walking_Turn)*((m_R_footprints + 1) * walking.step.LRstepSize + body_kine.lower.R.Base_to_Foot(Y) + step_offset) \
                                         + (walking.step.walking_Turn)*((body_kine.lower.R.Base_to_Foot(Y) - walking.step.turningPoint) * cos((m_footprints)*-walking.step.TurnSize) + walking.step.turningPoint);
                RfootPrints(Yaw, m_R_footprints) = (m_footprints) * walking.step.TurnSize;
                m_R_footprints++;
            }
        }
    }
    std::cout << "LFoot : " << std::endl << LfootPrints << std::endl;
    std::cout << "RFoot : " << std::endl << RfootPrints << std::endl;
    // printf(C_GREEN "Planning Foot steps \n" C_RESET);
}


void CKubot::FootPlannerWindowMove(const double step_y_offset)
{

    double step_offset = step_y_offset; // lowerbody.LEG_SIDE_OFFSET : 105, lowerbody.foot_y_length :125 (Foot Size)

    if (walking.change_Turn_flag == true)
    {
        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(Yaw, 0) = walking.step.RFoot(Yaw, 1);
            walking.step.RFoot(Yaw, 1) = walking.step.RFoot(Yaw, 2);
            walking.step.LFoot(Yaw, 2) = walking.step.RFoot(Yaw, 1) + walking.step.TurnSize;
            walking.step.RFoot(Yaw, 2) = walking.step.LFoot(Yaw, 2);
            //            walking.step.LFoot(Yaw, 3) = walking.step.RFoot(Yaw, 2) + walking.step.TurnSize;
            //            walking.step.RFoot(Yaw, 3) = walking.step.LFoot(Yaw, 3);
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(Yaw, 0) = walking.step.LFoot(Yaw, 1);
            walking.step.LFoot(Yaw, 1) = walking.step.LFoot(Yaw, 2);
            walking.step.RFoot(Yaw, 2) = walking.step.LFoot(Yaw, 1) + walking.step.TurnSize;
            walking.step.LFoot(Yaw, 2) = walking.step.RFoot(Yaw, 2);
            //            walking.step.RFoot(Yaw, 3) = walking.step.LFoot(Yaw, 2) + walking.step.TurnSize;
            //            walking.step.LFoot(Yaw, 3) = walking.step.RFoot(Yaw, 3);
        }

        walking.step.oldTurnSize = walking.step.TurnSize;
    }
    else
    {
        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(Yaw, 0) = walking.step.RFoot(Yaw, 1);
            walking.step.RFoot(Yaw, 1) = walking.step.RFoot(Yaw, 2);
            walking.step.RFoot(Yaw, 2) = walking.step.RFoot(Yaw, 2) + walking.step.TurnSize;
            //            walking.step.RFoot(Yaw, 3) = walking.step.RFoot(Yaw, 3) + walking.step.TurnSize;
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(Yaw, 0) = walking.step.LFoot(Yaw, 1);
            walking.step.LFoot(Yaw, 1) = walking.step.LFoot(Yaw, 2);
            walking.step.LFoot(Yaw, 2) = walking.step.LFoot(Yaw, 2) + walking.step.TurnSize;
            //            walking.step.LFoot(Yaw, 3) = walking.step.LFoot(Yaw, 3) + walking.step.TurnSize;
        }
    }

    //    std::cout << "LFoot : " << std::endl << walking.step.LFoot.row(5) * R2D << std::endl;
    //    std::cout << "RFoot : " << std::endl << walking.step.RFoot.row(5) * R2D << std::endl;

    static Vector3d oldStep = Vector3d::Zero();
    static Vector3d nowStep = Vector3d::Zero();

    static Vector3d LRstep = Vector3d::Zero();
    static Vector3d RLstep = Vector3d::Zero();
    static Vector3d Last_before_LRstep = Vector3d::Zero();
    static Vector3d Last_before_RLstep = Vector3d::Zero();
    static Vector3d Last_LRstep = Vector3d::Zero();
    static Vector3d Last_RLstep = Vector3d::Zero();

    oldStep(X) = 2 * walking.step.oldFBstepSize;
    oldStep(Y) = walking.step.oldLRstepSize;

    if (walking.step.LRstepSize > 0)
    {
        RLstep(X) = walking.step.FBstepSize;
        LRstep(X) = walking.step.FBstepSize;
        RLstep(Y) = walking.step.LRstepSize + 2 * body_kine.lower.L.Base_to_Foot(Y) - 2 * step_offset;
        LRstep(Y) = 2 * body_kine.lower.R.Base_to_Foot(Y) + 2 * step_offset;
    }
    else if (walking.step.LRstepSize < 0)
    {
        RLstep(X) = walking.step.FBstepSize;
        LRstep(X) = walking.step.FBstepSize;
        RLstep(Y) = 2 * body_kine.lower.L.Base_to_Foot(Y) - 2 * step_offset;
        LRstep(Y) = walking.step.LRstepSize + 2 * body_kine.lower.R.Base_to_Foot(Y) + 2 * step_offset;
    }
    else
    {
        RLstep(X) = walking.step.FBstepSize;
        LRstep(X) = walking.step.FBstepSize;
        RLstep(Y) = 2 * body_kine.lower.L.Base_to_Foot(Y) - 2 * step_offset;
        LRstep(Y) = 2 * body_kine.lower.R.Base_to_Foot(Y) + 2 * step_offset;
    }

    Last_before_RLstep(X) = walking.step.FBstepSize;
    Last_before_LRstep(X) = walking.step.FBstepSize;
    Last_before_RLstep(Y) = 2 * body_kine.lower.L.Base_to_Foot(Y) - step_offset;
    Last_before_LRstep(Y) = 2 * body_kine.lower.R.Base_to_Foot(Y) + step_offset;

    Last_RLstep(X) = walking.step.FBstepSize;
    Last_LRstep(X) = walking.step.FBstepSize;
    Last_RLstep(Y) = 2 * body_kine.lower.L.Base_to_Foot(Y);
    Last_LRstep(Y) = 2 * body_kine.lower.R.Base_to_Foot(Y);

    if (walking.change_FB_walking_flag == true || walking.change_LR_walking_flag == true || walking.change_Turn_flag == true || CommandFlag == ACT_STOP_WALK)
    {
        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(X, 0) = walking.step.RFoot(X, 1);
            walking.step.RFoot(X, 1) = walking.step.LFoot(X, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(X);
            //            walking.step.LFoot(X, 2) = walking.step.RFoot(X, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(X);
            //            walking.step.RFoot(X, 2) = walking.step.LFoot(X, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(X);
            if (CommandFlag == ACT_STOP_WALK)
            {
                walking.step.LFoot(X, 2) = walking.step.RFoot(X, 1) + local2Global(Last_before_RLstep, walking.step.RFoot.col(1))(X);
                walking.step.RFoot(X, 2) = walking.step.LFoot(X, 2) + local2Global(Last_LRstep, walking.step.LFoot.col(2))(X);
            }
            else
            {
                walking.step.LFoot(X, 2) = walking.step.RFoot(X, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(X);
                walking.step.RFoot(X, 2) = walking.step.LFoot(X, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(X);
            }

            walking.step.RFoot(Y, 0) = walking.step.RFoot(Y, 1);
            walking.step.RFoot(Y, 1) = walking.step.LFoot(Y, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(Y);
            //            walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(Y);
            //            walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(Y);
            if (CommandFlag == ACT_STOP_WALK)
            {
                walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 1) + local2Global(Last_before_RLstep, walking.step.RFoot.col(1))(Y);
                walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 2) + local2Global(Last_LRstep, walking.step.LFoot.col(2))(Y);
            }
            else
            {
                walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(Y);
                walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(Y);
            }
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(X, 0) = walking.step.LFoot(X, 1);
            walking.step.LFoot(X, 1) = walking.step.RFoot(X, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(X);
            //            walking.step.RFoot(X, 2) = walking.step.LFoot(X, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(X);
            //            walking.step.LFoot(X, 2) = walking.step.RFoot(X, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(X);
            if (CommandFlag == ACT_STOP_WALK)
            {
                walking.step.RFoot(X, 2) = walking.step.LFoot(X, 1) + local2Global(Last_before_LRstep, walking.step.LFoot.col(1))(X);
                walking.step.LFoot(X, 2) = walking.step.RFoot(X, 2) + local2Global(Last_RLstep, walking.step.RFoot.col(2))(X);
            }
            else
            {
                walking.step.RFoot(X, 2) = walking.step.LFoot(X, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(X);
                walking.step.LFoot(X, 2) = walking.step.RFoot(X, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(X);
            }

            walking.step.LFoot(Y, 0) = walking.step.LFoot(Y, 1);
            walking.step.LFoot(Y, 1) = walking.step.RFoot(Y, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(Y);
            //            walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(Y);
            //            walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(Y);
            if (CommandFlag == ACT_STOP_WALK)
            {
                walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 1) + local2Global(Last_before_LRstep, walking.step.LFoot.col(1))(Y);
                walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 2) + local2Global(Last_RLstep, walking.step.RFoot.col(2))(Y);
            }
            else
            {
                walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(Y);
                walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(Y);
            }
        }
        if (walking.change_FB_walking_flag == true)
        {
            walking.step.oldFBstepSize = walking.step.FBstepSize;
            walking.change_FB_walking_flag = false;
        }
        if (walking.change_LR_walking_flag == true)
        {
            walking.step.oldLRstepSize = walking.step.LRstepSize;
            walking.change_LR_walking_flag = false;
        }
        if (walking.change_Turn_flag == true)
        {
            walking.change_Turn_flag = false;
        }
    }
    else
    {
        if (walking.old_walk_SSP == walking.RSSP)
        {
            walking.step.RFoot(X, 0) = walking.step.RFoot(X, 1);
            walking.step.RFoot(X, 1) = walking.step.LFoot(X, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(X);
            walking.step.LFoot(X, 2) = walking.step.RFoot(X, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(X);
            walking.step.RFoot(X, 2) = walking.step.LFoot(X, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(X);

            walking.step.RFoot(Y, 0) = walking.step.RFoot(Y, 1);
            walking.step.RFoot(Y, 1) = walking.step.LFoot(Y, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(Y);
            walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(Y);
            walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 2) + local2Global(LRstep, walking.step.LFoot.col(2))(Y);
        }
        else if (walking.old_walk_SSP == walking.LSSP)
        {
            walking.step.LFoot(X, 0) = walking.step.LFoot(X, 1);
            walking.step.LFoot(X, 1) = walking.step.RFoot(X, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(X);
            walking.step.RFoot(X, 2) = walking.step.LFoot(X, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(X);
            walking.step.LFoot(X, 2) = walking.step.RFoot(X, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(X);

            walking.step.LFoot(Y, 0) = walking.step.LFoot(Y, 1);
            walking.step.LFoot(Y, 1) = walking.step.RFoot(Y, 1) + local2Global(RLstep, walking.step.RFoot.col(1))(Y);
            walking.step.RFoot(Y, 2) = walking.step.LFoot(Y, 1) + local2Global(LRstep, walking.step.LFoot.col(1))(Y);
            walking.step.LFoot(Y, 2) = walking.step.RFoot(Y, 2) + local2Global(RLstep, walking.step.RFoot.col(2))(Y);
        }
    }

    std::cout << "LFoot : " << std::endl << walking.step.LFoot << std::endl;
    std::cout << "RFoot : " << std::endl << walking.step.RFoot << std::endl;
}

void CKubot::BallCatchMotion(float standupTime)
{
    static float STUtime = 0;
    static float currentAngle[20]; // Now nDoF = 20

    if (STUtime >= 0 && motion_1 == false)
    {

        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-90 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-85 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_1 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_1 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_1 = true;
            motion_2 = false;
        }
    }

    else if (STUtime >= 0 && motion_2 == false)
    {
        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-90 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-0 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_2 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_2 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_2 = true;
            motion_3 = false;
        }
    }

    else if (STUtime >= 0 && motion_3 == false)
    {
        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-10 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint


        if (STUtime == 0 && motion_3 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_3 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_3 = true;
            MOTION_ING = true;
        }
    }


    else
    {
        motion_1 = false;
        motion_2 = true;
        motion_3 = true;
    }
}


void CKubot::BallSwingMotion(float standupTime)
{
    static float STUtime = 0;
    static float currentAngle[20]; // Now nDoF = 20

    if (STUtime >= 0 && motion_1 == false)
    {

        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-15 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-10 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_1 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_1 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_1 = true;
            motion_2 = false;
        }
    }

    else if (STUtime >= 0 && motion_2 == false)
    {
        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-210 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-10 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_2 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_2 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_2 = true;
            motion_3 = false;
        }
    }

    else if (STUtime >= 0 && motion_3 == false)
    {
        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-210 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-50 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_3 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_3 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_3 = true;
            motion_4 = false;
        }
    }

    else if (STUtime >= 0 && motion_4 == false)
    {
        VectorXd Base_pos(6);

        Base_pos(0) = 0;
        Base_pos(1) = 0;
        Base_pos(2) = 0.38;
        Base_pos(3) = 0;
        Base_pos(4) = 0;
        Base_pos(5) = 0;

        VectorXd init_LFoot(6), init_RFoot(6);
        init_LFoot << 0, 0.05, 0, 0, 0, 0;
        init_RFoot << 0, -0.05, 0, 0, 0, 0;

        VectorXd IK_walkreadyAngle(12);

        IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot), Geometric_IK_R(Base_pos, init_RFoot);

        // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++)
        {
            standupAngle[j] = IK_walkreadyAngle(j);
            // printf("The Robot's Joint for Walking Ready = %f\n" , walkReadyAngle[j] * R2D);
        }

        standupAngle[12] = (-15 * D2R); // L_shoulder_pitch_joint 플러스가 어깨 뒤로
        standupAngle[13] = (0 * D2R);   // L_elbow_roll_joint 플러스가 좌우로 나란히
        standupAngle[14] = (-85 * D2R); // L_hand_pitch_joint  플러스가 안으로 굽
        standupAngle[15] = (-0 * D2R); // R_shoulder_pitch_joint
        standupAngle[16] = (-0 * D2R);  // R_elbow_roll_joint
        standupAngle[17] = (-50 * D2R); // R_hand_pitch_joint
        standupAngle[18] = (0 * D2R);   // Neck_yaw_joint
        standupAngle[19] = (0 * D2R);   // head_pitch_joint

        if (STUtime == 0 && motion_4 == false)
        {

            for (int j = 0; j < 20; j++)
            {
                currentAngle[j] = refAngle[j];
            }
        }

        if (STUtime < standupTime && motion_4 == false)
        {
            for (int j = 0; j < 20; j++)
            {
                refAngle[j] = cosWave(standupAngle[j] - currentAngle[j], standupTime, STUtime, currentAngle[j]);
            }
            STUtime += tasktime;
        }

        else
        {
            STUtime = 0;
            motion_4 = true;
            MOTION_ING = true;
        }
    }


    else
    {
        motion_1 = false;
        motion_2 = true;
        motion_3 = true;
        motion_4 = true;
    }
}
