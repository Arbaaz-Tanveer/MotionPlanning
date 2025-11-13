#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

USING_NAMESPACE_ACADO

// --------------------- HELPER FUNCTIONS ---------------------
// Normalize angle to [-π, π]
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Compute shortest angular difference from angle1 to angle2
inline double angle_difference(double target, double current) {
    double diff = target - current;
    return normalize_angle(diff);
}

// --------------------- GLOBAL VARIABLES ---------------------
double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
double current_vx = 0.0, current_vy = 0.0, current_omega = 0.0;
double next_x = 0.0, next_y = 0.0, next_theta = 0.0;
double next_vx = 0.0, next_vy = 0.0, next_omega = 0.0;

// DRIBBLING FLAG
bool is_dribbling = false;

// ========== ROBOT PHYSICAL PARAMETERS ==========
// Adjust these to match your robot!
const double ROBOT_MASS = 25.0;              // kg (measure your robot)
const double ROBOT_INERTIA = 1.56;            // kg⋅m² (I = m⋅r² for approximation)
const double WHEEL_RADIUS = 0.05;            // m (5cm wheels)
const double ROBOT_RADIUS = 0.25;             // m (distance from center to wheel)

// Motor specifications
const double MAX_WHEEL_TORQUE = 1;         // N⋅m (check motor datasheet)
const double MAX_WHEEL_VEL = (750/60)*6.28;           // rad/s (motor speed limit)

// Computed force limits
const double MAX_WHEEL_FORCE = MAX_WHEEL_TORQUE / WHEEL_RADIUS;  // N (force at wheel contact)

// ========== MPC PARAMETERS ==========
const double NORMAL_HORIZON = 1.0;      // normal mode horizon (seconds)
const int NORMAL_N = 8;           // normal mode intervals
const double DRIBBLING_HORIZON = 1.5;   // dribbling mode horizon (seconds)
const int DRIBBLING_N = 8;        // dribbling mode intervals (longer horizon)

// Path reference (each element = {x_ref, y_ref, theta_ref})
std::vector<std::array<double, 3>> path_points;

// Differential states and controls
DifferentialState x, y, theta, vx, vy, omega;
Control ax, ay, alpha;
TIME t;

// Dynamics model
DifferentialEquation f;

// --------------------- NODE CLASS ---------------------
class MyRobotNode : public rclcpp::Node {
public:
    MyRobotNode() : Node("my_robot_node") {
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("o1/cmd_vel", 10);
        self_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/o1_data", 10, std::bind(&MyRobotNode::mpc_callback, this, std::placeholders::_1));
        target_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_pos", 10, std::bind(&MyRobotNode::update_target_callback, this, std::placeholders::_1));
        dribbling_subscription = this->create_subscription<std_msgs::msg::Bool>(
            "dribbling_flag", 10, std::bind(&MyRobotNode::dribbling_callback, this, std::placeholders::_1));

        // ------------------- Define Dynamics -------------------
        f << dot(x) == vx;
        f << dot(y) == vy;
        f << dot(theta) == omega;
        f << dot(vx) == ax;
        f << dot(vy) == ay;
        f << dot(omega) == alpha;

        std::cout << "========== Robot Parameters ==========" << std::endl;
        std::cout << "Mass: " << ROBOT_MASS << " kg" << std::endl;
        std::cout << "Inertia: " << ROBOT_INERTIA << " kg⋅m²" << std::endl;
        std::cout << "Wheel radius: " << WHEEL_RADIUS << " m" << std::endl;
        std::cout << "Robot radius: " << ROBOT_RADIUS << " m" << std::endl;
        std::cout << "Max wheel torque: " << MAX_WHEEL_TORQUE << " N⋅m" << std::endl;
        std::cout << "Max wheel force: " << MAX_WHEEL_FORCE << " N" << std::endl;
        std::cout << "=====================================" << std::endl;
    }

    // ----------------------------------------------------------
    //                  DRIBBLING FLAG CALLBACK
    // ----------------------------------------------------------
    void dribbling_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(dribbling_mutex);
        is_dribbling = msg->data;
        RCLCPP_INFO(this->get_logger(), "🏀 Dribbling mode: %s", is_dribbling ? "ENABLED" : "DISABLED");
    }

    // ----------------------------------------------------------
    //                      MPC CALLBACK
    // ----------------------------------------------------------
    void mpc_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        current_x = msg->data[0];
        current_y = msg->data[1];
        current_theta = normalize_angle(msg->data[2]);  // Normalize incoming angle

        current_vx = next_vx;
        current_vy = next_vy;
        current_omega = next_omega;

        // Adjust MPC parameters based on dribbling mode
        int N = is_dribbling ? DRIBBLING_N : NORMAL_N;
        double horizon = is_dribbling ? DRIBBLING_HORIZON : NORMAL_HORIZON;

        OCP ocp(0.0, horizon, N);

        // ------------------- LAGRANGE COST ------------------- 
        Function h;
        h << x << y;  // track position only
        DMatrix W(2, 2);
        W.setAll(0.0);
        W(0,0) = is_dribbling ? 500.0 : 50;   // x weight
        W(1,1) = is_dribbling ? 500.0 : 50;   // y weight

        // ------------------- TERMINAL COST -------------------
        // Use sin/cos representation for angle to avoid wrapping issues in cost
        Function hN;
        hN << x << y << sin(theta) << cos(theta);
        DMatrix WN(4, 4);
        WN.setAll(0.0);
        WN(0,0) = 1500.0;  // x
        WN(1,1) = 1500.0;  // y
        WN(2,2) = 200.0;   // sin(theta)
        WN(3,3) = 200.0;   // cos(theta)

        // ------------------- REFERENCE TRAJECTORY -------------------
        VariablesGrid reference;
        reference.init(h.getDim(), Grid(0.0, horizon, N+1));

        if (path_points.empty()) {
            for (int i = 0; i <= N; ++i) {
                DVector r(h.getDim());
                r.setAll(0.0);
                r(0) = current_x;
                r(1) = current_y;
                reference.setVector(i, r);
            }
        } else {
            for (int i = 0; i <= N; ++i) {
                DVector r(h.getDim());
                r.setAll(0.0);
                size_t idx = std::min(i, (int)path_points.size() - 1);
                r(0) = path_points[idx][0];
                r(1) = path_points[idx][1];
                reference.setVector(i, r);
            }
        }

        // ------------------- COST SETUP -------------------
        ocp.minimizeLSQ(W, h, reference);

        // Terminal reference with sin/cos representation
        DVector refN(hN.getDim());
        double target_theta;
        if (path_points.empty()) {
            refN(0) = current_x;
            refN(1) = current_y;
            target_theta = current_theta;
        } else {
            refN(0) = path_points.back()[0];
            refN(1) = path_points.back()[1];
            target_theta = normalize_angle(path_points.back()[2]);
        }
        refN(2) = sin(target_theta);  // sin component
        refN(3) = cos(target_theta);  // cos component
        
        ocp.minimizeLSQEndTerm(WN, hN, refN);

        // ========== DYNAMICS-BASED CONSTRAINTS ==========
        
        const double m = ROBOT_MASS;
        const double I = ROBOT_INERTIA;
        const double r = WHEEL_RADIUS;
        const double L = ROBOT_RADIUS;
        const double sqrt2 = sqrt(2.0);
        const double F_max = MAX_WHEEL_FORCE;
        const double omega_max = MAX_WHEEL_VEL;

        // ----- KINEMATICS (velocity mapping) -----
        // Wheel velocities for 45° X-configuration
        // corrected wheel angular velocities (X-configuration, ±45deg wheels)

        Expression vx_local =  cos(theta)*vx + sin(theta)*vy;
        Expression vy_local = -sin(theta)*vx + cos(theta)*vy;

        Expression ax_local =  cos(theta)*ax + sin(theta)*ay;
        Expression ay_local = -sin(theta)*ax + cos(theta)*ay;

        Expression omega1 = (sqrt2/(2.0*r)) * (-vx_local + vy_local) + (L/r) * omega;
        Expression omega2 = (sqrt2/(2.0*r)) * ( vx_local + vy_local) + (L/r) * omega;
        Expression omega3 = (sqrt2/(2.0*r)) * ( vx_local - vy_local) + (L/r) * omega;
        Expression omega4 = (sqrt2/(2.0*r)) * (-vx_local - vy_local) + (L/r) * omega;

        // corrected wheel angular accelerations
        Expression alpha_wheel1 = (sqrt2/(2.0*r)) * (-ax_local + ay_local) + (L/r) * alpha;
        Expression alpha_wheel2 = (sqrt2/(2.0*r)) * ( ax_local + ay_local) + (L/r) * alpha;
        Expression alpha_wheel3 = (sqrt2/(2.0*r)) * ( ax_local - ay_local) + (L/r) * alpha;
        Expression alpha_wheel4 = (sqrt2/(2.0*r)) * (-ax_local - ay_local) + (L/r) * alpha;

        // Force mapping (these were correct in your snippet)
        Expression F1 = (m*sqrt2/4.0) * (-ax_local + ay_local) + (I/(4.0*L)) * alpha;
        Expression F2 = (m*sqrt2/4.0) * ( ax_local + ay_local) + (I/(4.0*L)) * alpha;
        Expression F3 = (m*sqrt2/4.0) * ( ax_local - ay_local) + (I/(4.0*L)) * alpha;
        Expression F4 = (m*sqrt2/4.0) * (-ax_local - ay_local) + (I/(4.0*L)) * alpha;

        // ----- APPLY CONSTRAINTS -----
        
        ocp.subjectTo(f);
        // 1. Wheel velocity constraints (motor speed limits)
        ocp.subjectTo(-omega_max <= omega1 <= omega_max);
        ocp.subjectTo(-omega_max <= omega2 <= omega_max);
        ocp.subjectTo(-omega_max <= omega3 <= omega_max);
        ocp.subjectTo(-omega_max <= omega4 <= omega_max);

        // 2. Wheel force constraints (motor torque limits)
        ocp.subjectTo(-F_max <= F1 <= F_max);
        ocp.subjectTo(-F_max <= F2 <= F_max);
        ocp.subjectTo(-F_max <= F3 <= F_max);
        ocp.subjectTo(-F_max <= F4 <= F_max);

        // ========== DRIBBLING MODE CONSTRAINTS ==========
        if (is_dribbling) {
            // Prevent backward motion in local frame (vx_local >= 0) 
            ocp.subjectTo(-0.2 <= vx_local <= 2.0);  // some finite negative value is needed to correct slight errors without major rotation needed
            
            // Allow lateral motion but limit it
            ocp.subjectTo(-1.0 <= vy_local <= 1.0);
            
            // Limit angular velocity for stability while dribbling
            ocp.subjectTo(-0.8 <= omega <= 0.8);
            
            RCLCPP_INFO(this->get_logger(), "🏀 Dribbling constraints active: no backward motion allowed");
        }else{
                // In normal mode, allow full velocity ranges
                ocp.subjectTo(-2.5 <= vx_local <= 2.5);
                ocp.subjectTo(-2.5 <= vy_local <= 2.5);
                ocp.subjectTo(-1.0 <= omega <= 1.0);
        }

        // ------------------- INITIAL CONDITIONS -------------------
        ocp.subjectTo(AT_START, x == current_x);
        ocp.subjectTo(AT_START, y == current_y);
        ocp.subjectTo(AT_START, theta == current_theta);
        ocp.subjectTo(AT_START, vx == current_vx);
        ocp.subjectTo(AT_START, vy == current_vy);
        ocp.subjectTo(AT_START, omega == current_omega);

        RCLCPP_INFO(this->get_logger(), "Running MPC [%s mode] N=%d, T=%.2fs with %zu path refs. θ_target=%.3f", 
                    is_dribbling ? "DRIBBLING" : "NORMAL", N, horizon, path_points.size(), target_theta);

        // ------------------- SOLVER CONFIG -------------------
        OptimizationAlgorithm algorithm(ocp);
        algorithm.set(PRINTLEVEL, NONE);
        // algorithm.set(HOTSTART, YES);
        algorithm.set(MAX_NUM_ITERATIONS, 3);  // fast real-time iteration
        algorithm.set(KKT_TOLERANCE, 1e-3);

        // ------------------- SOLVE AND TIME IT -------------------
        auto t0 = std::chrono::high_resolution_clock::now();
        int ret = algorithm.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        RCLCPP_INFO(this->get_logger(), "MPC solve time: %.2f ms", solve_ms);

        if (ret != SUCCESSFUL_RETURN) {
            RCLCPP_WARN(this->get_logger(), "⚠️ MPC failed to solve (code %d)", ret);
            return;
        }

        // ------------------- EXTRACT RESULTS -------------------
        VariablesGrid states;
        algorithm.getDifferentialStates(states);
        if (states.getNumPoints() > 1) {
            DVector next_state = states.getVector(1);
            next_x = next_state(0);
            next_y = next_state(1);
            next_theta = normalize_angle(next_state(2));  // Normalize output angle
            next_vx = next_state(3);
            next_vy = next_state(4);
            next_omega = next_state(5);
        }

        // Calculate angle error for logging
        double angle_error = angle_difference(target_theta, current_theta);

        RCLCPP_INFO(this->get_logger(), 
            "Next -> X: %.3f, Y: %.3f, θ: %.3f (err: %.3f), Vx: %.3f, Vy: %.3f, ω: %.3f",
            next_x, next_y, next_theta, angle_error, next_vx, next_vy, next_omega);

        // ------------------- PUBLISH -------------------
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = next_vx;
        twist_msg.linear.y = next_vy;
        twist_msg.angular.z = next_omega;
        cmd_vel_publisher->publish(twist_msg);
    }

    // ----------------------------------------------------------
    //           Parse and store target path from topic
    // ----------------------------------------------------------
    void update_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(target_mutex);
        path_points.clear();

        for (size_t i = 0; i + 2 < msg->data.size(); i += 3) {
            // Normalize angles when storing path points
            path_points.push_back({ 
                msg->data[i], 
                msg->data[i + 1], 
                normalize_angle(msg->data[i + 2]) 
            });
        }

        if (!path_points.empty()) {
            RCLCPP_INFO(this->get_logger(), "Received path with %zu points.", path_points.size());
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr self_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribbling_subscription;
    std::mutex target_mutex;
    std::mutex dribbling_mutex;
};

// --------------------- MAIN ---------------------
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotNode>());
    rclcpp::shutdown();
    return 0;
}