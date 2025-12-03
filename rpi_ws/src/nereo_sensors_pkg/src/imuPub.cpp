#include "nereo_sensors_pkg/imuPub.hpp"
#include <type_traits>
using namespace std::chrono_literals;
using std::endl;

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherIMU>());
    rclcpp::shutdown();
    return 0;
}

void GetCovarianceMatrix(std::queue<Vec3> window, CovarianceMatrix *matrix) {

    if (window.empty())
        return;

    std::queue<Vec3> copy = window;

    Vec3 mean = {0};
    float sum[9] = {0};

    // CALCULATE SUM
    while(!copy.empty()){
        mean.x += copy.front().x;
        mean.y += copy.front().y;
        mean.z += copy.front().z;

        copy.pop();
    }


    // CALCULATE MEAN
    mean.x = mean.x / window.size();
    mean.y = mean.y / window.size();
    mean.z = mean.z / window.size();


    // CALCULATE VARIANCES
    copy = window;

    while(!copy.empty()){
        sum[0] += (copy.front().x - mean.x)*(copy.front().x - mean.x);  // V[x]
        sum[1] += (copy.front().x - mean.x)*(copy.front().y - mean.y);  // Cov[x,y]
        sum[2] += (copy.front().x - mean.x)*(copy.front().z - mean.z);  // Cov[x,z]
        sum[4] += (copy.front().y - mean.y)*(copy.front().y - mean.y);  // V[y]
        sum[5] += (copy.front().y - mean.y)*(copy.front().z - mean.z);  // Cov[y,z]
        sum[8] += (copy.front().z - mean.z)*(copy.front().z - mean.z);  // V[z]

        copy.pop();
    }

    matrix->matrix[0] = sum[0] / window.size();
    matrix->matrix[1] = sum[1] / window.size();
    matrix->matrix[2] = sum[2] / window.size();
    matrix->matrix[3] = matrix->matrix[1];
    matrix->matrix[4] = sum[4] / window.size();
    matrix->matrix[5] = sum[5] / window.size();
    matrix->matrix[6] = matrix->matrix[2];
    matrix->matrix[7] = matrix->matrix[5];
    matrix->matrix[8] = sum[8] / window.size();
}


void PublisherIMU::timer_callback()
{
    tf2::Quaternion tf2_quat, tf2_quat_from_msg;

    auto imu_data_message = sensor_msgs::msg::Imu();

    imu_data_message.header.stamp = this->get_clock()->now();
    imu_data_message.header.frame_id = "IMU Data";

    // READ DATA FROM IMU
    imu_ang_vel_error = WT61P_read_angular_vel();

    imu_data_message.angular_velocity.x = WT61P_get_angular_vel_x();
    imu_data_message.angular_velocity.y = WT61P_get_angular_vel_y();
    imu_data_message.angular_velocity.z = WT61P_get_angular_vel_z();

    imu_acc_error = WT61P_read_acc();
    imu_data_message.linear_acceleration.x = WT61P_get_acc_x();
    imu_data_message.linear_acceleration.y = WT61P_get_acc_y();
    imu_data_message.linear_acceleration.z = WT61P_get_acc_z();

    imu_angle_error = WT61P_read_angle();
    Vec3 angles = { WT61P_get_pitch(), WT61P_get_roll(), WT61P_get_yaw() };

    // TF2 QUATERNION CONVERSION
    tf2_quat.setRPY(angles.x * 0.0174533, angles.y * 0.0174533, angles.z * 0.0174533); // Converted to radians
    tf2_quat.normalize();

    imu_data_message.orientation = tf2::toMsg(tf2_quat);

    // LINEAR ACCELERATION COVARIANCE
    GetCovarianceMatrix(acceleration_window, &matrix);

    for (int i = 0; i < 9; i++)
        imu_data_message.linear_acceleration_covariance[i] = matrix.matrix[i];

    // ANGULAR VELOCITY COVARIANCE
    GetCovarianceMatrix(angular_velocity_window, &matrix);

    for (int i = 0; i < 9; i++)
        imu_data_message.angular_velocity_covariance[i] = matrix.matrix[i];

    // ORIENTATION COVARIANCE
    GetCovarianceMatrix(angles_window, &matrix);

    for (int i = 0; i < 9; i++)
        imu_data_message.orientation_covariance[i] = matrix.matrix[i];

    
    // Diagnostic
    auto imu_diagnostic_message = diagnostic_msgs::msg::DiagnosticArray();

    imu_diagnostic_message.header.stamp = this->get_clock()->now();
    imu_diagnostic_message.header.frame_id = "Rov Body Frame";

    //std::cout << "acc err: " << imu_acc_error << endl << "ang vel err: " << imu_ang_vel_error << endl << "ori err: " << imu_angle_error << endl;

    // GENERAL ERROR ENCOUTERED
    if (imu_acc_error || imu_ang_vel_error || imu_angle_error) {
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU data acquisition.";
        diagnostic_status.message = "Errors encountered while acquiring data from IMU";
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }
    else {
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = OK;
        diagnostic_status.name = "IMU data acquisition.";
        diagnostic_status.message = "All data acquired correctly.";
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }

    // ACCELERATION ERROR
    if (imu_acc_error) {
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU acceleration acquisition.";
        diagnostic_status.message = "Error while acquiring Acceleration " + std::to_string(imu_acc_error);
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }

    // ANGULAR VELOCITY ERROR
    if (imu_ang_vel_error) {
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU Angular velocity acquisition.";
        diagnostic_status.message = "Error while acquiring Angular velocity" + std::to_string(imu_ang_vel_error);
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }

    // ANGLE ERROR
    if (imu_angle_error) {
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU Angle acquisition.";
        diagnostic_status.message = "Error while acquiring Angular velocity" + std::to_string(imu_angle_error);
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }
    imu_diagnostic_publisher_->publish(imu_diagnostic_message);

   imu_data_publisher_->publish(imu_data_message);
}

PublisherIMU::PublisherIMU(): Node("imu_publisher")
{
    // imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", getSensorQoS());
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    // imu_diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("imu_diagnostic", getReliableQoS());
    imu_diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("imu_diagnostic", 10);

    timer_ = this->create_wall_timer(200ms, std::bind(&PublisherIMU::timer_callback, this));

    int ret = WT61P_begin(i2c_device, WT61P_IIC_ADDR);
};

/*
Diagnostic status
- [0]: General INFO
- [1 - 3]: Errors encountered (Acc, AngVel, Angle)
*/
