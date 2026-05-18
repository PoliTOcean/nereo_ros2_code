#include "nereo_sensors_pkg/imuPub.hpp"
using namespace std::chrono_literals;

// Defined here, declared extern in imuPub.hpp
char i2c_device[] = "/dev/i2c-1";

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherIMU>());
    rclcpp::shutdown();
    return 0;
}

// ── covariance helper ─────────────────────────────────────────────────────

void GetCovarianceMatrix(std::queue<Vec3> window, CovarianceMatrix *matrix)
{
    if (window.empty())
        return;

    std::queue<Vec3> copy = window;
    Vec3 mean = {0, 0, 0};
    float sum[9] = {0};

    while (!copy.empty()) {
        mean.x += copy.front().x;
        mean.y += copy.front().y;
        mean.z += copy.front().z;
        copy.pop();
    }

    const float n = static_cast<float>(window.size());
    mean.x /= n;
    mean.y /= n;
    mean.z /= n;

    copy = window;
    while (!copy.empty()) {
        float dx = copy.front().x - mean.x;
        float dy = copy.front().y - mean.y;
        float dz = copy.front().z - mean.z;
        sum[0] += dx * dx;   // V[x]
        sum[1] += dx * dy;   // Cov[x,y]
        sum[2] += dx * dz;   // Cov[x,z]
        sum[4] += dy * dy;   // V[y]
        sum[5] += dy * dz;   // Cov[y,z]
        sum[8] += dz * dz;   // V[z]
        copy.pop();
    }

    matrix->matrix[0] = sum[0] / n;
    matrix->matrix[1] = sum[1] / n;
    matrix->matrix[2] = sum[2] / n;
    matrix->matrix[3] = matrix->matrix[1];
    matrix->matrix[4] = sum[4] / n;
    matrix->matrix[5] = sum[5] / n;
    matrix->matrix[6] = matrix->matrix[2];
    matrix->matrix[7] = matrix->matrix[5];
    matrix->matrix[8] = sum[8] / n;
}

// ── sliding window helper ─────────────────────────────────────────────────

void PublisherIMU::push_window(std::queue<Vec3> &window, Vec3 sample)
{
    window.push(sample);
    if (window.size() > MAXN)
        window.pop();
}

// ── timer callback ────────────────────────────────────────────────────────

void PublisherIMU::timer_callback()
{
    auto imu_data_message = sensor_msgs::msg::Imu();
    imu_data_message.header.stamp    = this->get_clock()->now();
    imu_data_message.header.frame_id = "imu";

    // READ — angular velocity
    imu_ang_vel_error = WT61P_read_angular_vel();
    imu_data_message.angular_velocity.x = WT61P_get_angular_vel_x();
    imu_data_message.angular_velocity.y = WT61P_get_angular_vel_y();
    imu_data_message.angular_velocity.z = WT61P_get_angular_vel_z();
    push_window(angular_velocity_window, {
        static_cast<float>(imu_data_message.angular_velocity.x),
        static_cast<float>(imu_data_message.angular_velocity.y),
        static_cast<float>(imu_data_message.angular_velocity.z)
    });

    // READ — linear acceleration
    imu_acc_error = WT61P_read_acc();
    imu_data_message.linear_acceleration.x = WT61P_get_acc_x();
    imu_data_message.linear_acceleration.y = WT61P_get_acc_y();
    imu_data_message.linear_acceleration.z = WT61P_get_acc_z();
    push_window(acceleration_window, {
        static_cast<float>(imu_data_message.linear_acceleration.x),
        static_cast<float>(imu_data_message.linear_acceleration.y),
        static_cast<float>(imu_data_message.linear_acceleration.z)
    });

    // READ — angles → quaternion
    imu_angle_error = WT61P_read_angle();
    Vec3 angles = { WT61P_get_pitch(), WT61P_get_roll(), WT61P_get_yaw() };
    push_window(angles_window, angles);

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(
        angles.x * (M_PI / 180.0),
        angles.y * (M_PI / 180.0),
        angles.z * (M_PI / 180.0)
    );
    tf2_quat.normalize();
    imu_data_message.orientation = tf2::toMsg(tf2_quat);

    // COVARIANCE
    GetCovarianceMatrix(acceleration_window,    &matrix);
    for (int i = 0; i < 9; i++)
        imu_data_message.linear_acceleration_covariance[i] = matrix.matrix[i];

    GetCovarianceMatrix(angular_velocity_window, &matrix);
    for (int i = 0; i < 9; i++)
        imu_data_message.angular_velocity_covariance[i] = matrix.matrix[i];

    GetCovarianceMatrix(angles_window, &matrix);
    for (int i = 0; i < 9; i++)
        imu_data_message.orientation_covariance[i] = matrix.matrix[i];

    // DIAGNOSTIC
    auto imu_diagnostic_message = diagnostic_msgs::msg::DiagnosticArray();
    imu_diagnostic_message.header.stamp    = this->get_clock()->now();
    imu_diagnostic_message.header.frame_id = "imu";

    auto general_status = diagnostic_msgs::msg::DiagnosticStatus();
    if (imu_acc_error || imu_ang_vel_error || imu_angle_error) {
        general_status.level   = ERROR;
        general_status.name    = "IMU data acquisition";
        general_status.message = "Errors encountered while acquiring IMU data";
        RCLCPP_WARN(this->get_logger(), "IMU error — acc:%d angvel:%d angle:%d",
            imu_acc_error, imu_ang_vel_error, imu_angle_error);
    } else {
        general_status.level   = OK;
        general_status.name    = "IMU data acquisition";
        general_status.message = "All data acquired correctly";
    }
    imu_diagnostic_message.status.push_back(general_status);

    if (imu_acc_error) {
        auto s = diagnostic_msgs::msg::DiagnosticStatus();
        s.level   = ERROR;
        s.name    = "IMU acceleration acquisition";
        s.message = "Error " + std::to_string(imu_acc_error);
        imu_diagnostic_message.status.push_back(s);
    }
    if (imu_ang_vel_error) {
        auto s = diagnostic_msgs::msg::DiagnosticStatus();
        s.level   = ERROR;
        s.name    = "IMU angular velocity acquisition";
        s.message = "Error " + std::to_string(imu_ang_vel_error);
        imu_diagnostic_message.status.push_back(s);
    }
    if (imu_angle_error) {
        auto s = diagnostic_msgs::msg::DiagnosticStatus();
        s.level   = ERROR;
        s.name    = "IMU angle acquisition";
        s.message = "Error " + std::to_string(imu_angle_error);
        imu_diagnostic_message.status.push_back(s);
    }

    imu_diagnostic_publisher_->publish(imu_diagnostic_message);
    imu_data_publisher_->publish(imu_data_message);
}

// ── constructor ───────────────────────────────────────────────────────────

PublisherIMU::PublisherIMU(): Node("imu_publisher")
{
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu_data", 10);
    imu_diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "imu_diagnostic", 10);

    timer_ = this->create_wall_timer(200ms, std::bind(&PublisherIMU::timer_callback, this));

    if (WT61P_begin(i2c_device, WT61P_IIC_ADDR) != 0)
        RCLCPP_ERROR(this->get_logger(), "WT61P_begin failed on %s", i2c_device);
    else
        RCLCPP_INFO(this->get_logger(), "IMU initialized on %s", i2c_device);
}
