#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpQuadProg.h>
#include <ecn_common/visp_utils.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    PioneerCam robot;

    // pose error gain
    const double lv = .5;
    // constraints gain
    const double lc = 2;
    geometry_msgs::Pose2D target;

    // robot command u = (v, omega, dot q_p, dot q_t)
    vpColVector u(4);

    // QP solver
    vpQuadProg qp;

    // Cost Function part of the QP
    int n=2;
    vpMatrix Q(n,4);
    Q.eye();
    vpColVector r(n);

    // Inequality part: wheel speed constraint
    double radius=robot.radius();
    double base=robot.base();
    vpMatrix C(8,4);
    vpColVector d(8);
    vpMatrix T(2,2);
    T[0][0] = 1/radius;
    T[0][1] = base/radius;
    T[1][0] = 1/radius;
    T[1][1] = -base/radius;
    ecn::putAt(C,T,0,0);
    ecn::putAt(C,-T,2,0);


    d[0]=d[1]=d[2]=d[3] = robot.wmax();

    while(ros::ok())
    {
        cout << "-------------" << endl;

        if(robot.stateReceived())
        {
            // get robot and target positions to get position error
            target = robot.targetRelativePose();

            // linear velocity
            u[0] = lv*(target.x - .1);
            // angular velocity
            u[1] = lv*std::atan2(target.y, target.x);

            // Inequality constraint: mantaining visual of sphere

            r[0] = u[0];
            r[1] = u[1];

            vpFeaturePoint sphere_point = robot.imagePoint();
            vpColVector cam_corner = robot.camLimits();
            vpMatrix Jc = robot.camJacobian();

            vpMatrix L = sphere_point.interaction();

            // Size 2x4
            auto Js = L*Jc;

            ecn::putAt(C, Js, 4, 0);
            ecn::putAt(C, -Js, 6, 0);

            //visual constraint coefficient
            double a = 1;
            //current coordinates of sphere in the image plane
            vpColVector sphere_point_vec(2,1);
            sphere_point_vec[0] = sphere_point.get_x();
            sphere_point_vec[1] = sphere_point.get_y();

            //required to set the target position of sphere in the center of the image plane (0,0 would be the corner of the image plane)
            vpColVector offset(2);
            offset[0]=0.5;
            offset[1]=0.5;
            //d=+-a(s_lim-s-x0) where x0 is the position of the center of the image plane
            vpColVector splus_side = a*(cam_corner - sphere_point_vec - offset);
            vpColVector sminus_side = -a*(cam_corner - sphere_point_vec - offset);
            ecn::putAt(d, splus_side, 4);
            ecn::putAt(d, sminus_side, 6);

            //equality constraint vw*-v*w=0
            //vpColVector b(1);
            //b[1]=0;
            //vpMatrix A(1,4);
            //A[0][0]=r[1];
            //A[0][1]=-r[0];


            //qp.solveQP( Q, r, A, b, C, d, u);
            qp.solveQPi( Q, r, C, d, u);


            cout << "u: " << u.t() << endl;

            robot.sendVelocity(u);
        }
    }
}
