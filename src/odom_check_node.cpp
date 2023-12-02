#include <cstdint>
#include <cmath>
#include <chrono>
#include <tuple>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <can_plugins2/msg/frame.hpp>
#include <can_utils.hpp>

using namespace std::chrono_literals;

struct RawData final
{
	std::int16_t roll{0};
	std::int16_t pitch{0};
	std::int16_t yaw{0};
	std::int16_t vx{0};

	std::int16_t ax{0};
	std::int16_t ay{0};
	std::int16_t az{0};
	std::int16_t vy{0};
};

class OdomCheckNode final : public rclcpp::Node
{
private:
	rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr sub_{};
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_{};
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_{};
	rclcpp::TimerBase::SharedPtr timer_{};
	std::mt19937 engine{};
	RawData odom{};

public:
	OdomCheckNode():
		Node("odom_check_node")
	{
		sub_ = this->create_subscription<can_plugins2::msg::Frame>("can_rx", 10, std::bind(&OdomCheckNode::can_callback, this, std::placeholders::_1));
		pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
		pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
		timer_ = this->create_wall_timer(1ms, std::bind(&OdomCheckNode::timer_callback, this));
	}

private:
	void can_callback(const can_plugins2::msg::Frame::SharedPtr msg)
	{
		if (msg->id == 0x555)
		{
			std::memcpy((char *)&odom, msg->data.data(), 8);
		}
		else if (msg->id == 0x556)
		{
			std::memcpy((char *)&odom + 8, msg->data.data(), 8);
		}

		// odom_msgD_.roll = odom_msg16_.roll * 6.28 / 360.0;
		// odom_msgD_.pitch = odom_msg16_.pitch * 6.28 / 360.0;
		// odom_msgD_.yaw = odom_msg16_.yaw * 6.28 / 360.0;
		// odom_msgD_.vx = odom_msg16_.vx * 45.0 * 3.14 / 2048.0;
		// odom_msgD_.vy = odom_msg16_.vy * 45.0 * 3.14 / 2048.0;
		// odom_msgD_.ax = odom_msg16_.ax * 1.0;
		// odom_msgD_.ay = odom_msg16_.ay * 1.0;
		// odom_msgD_.az = odom_msg16_.az * 1.0;
	}

	void timer_callback()
	{
		auto [odometry, imu] = raw_to_msgs(odom, this->now());
		pub_odom_->publish(odometry);
		pub_imu_->publish(imu);
	}

	// constexprなsqrt
	static constexpr auto constexpr_sqrt(double s) -> double {
		double x = s / 2.0;
		double prev = 0.0;

		while (x != prev)
		{
			prev = x;
			x = (x + s / x) / 2.0;
		}
		return x;
	}

	static constexpr auto encoder_to_odom_linear_velocity(std::int16_t v) -> double {
		constexpr double encoder_wheel_radius = 0.045;
		constexpr int encoder_ppr = 2048;
		constexpr int sampling_frequency = 1000;
		constexpr double encoder_coefficient = sampling_frequency * encoder_wheel_radius * std::numbers::pi / encoder_ppr;
		
		return v * encoder_coefficient;
	}

	static constexpr auto mpu_to_gyro_angular_velocity(std::int16_t theta) -> double {
		constexpr int k_nomi = 4000;
		constexpr int k_deno = 1 << 16;
		constexpr double mpu_gyro_coeeficient = 2 * std::numbers::pi * k_nomi / (k_deno * 360);
		return theta * mpu_gyro_coeeficient;
	}

	static constexpr auto mpu_to_accelaration(std::int16_t a) -> double {
		constexpr int k_nomi = 32;
		constexpr int k_deno = 1 << 16;
		constexpr double mpu_acc_coeeficient = 9.8 * k_nomi / k_deno;
		return a * mpu_acc_coeeficient;
	}

	static auto raw_to_msgs(const RawData& data, const rclcpp::Time& now) -> std::tuple<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu> {
		constexpr auto set_covariance = []<std::size_t n>(std::array<double, n> &cov, double x)
		{
			for (size_t i = 0; i < n; ++i)
			{
				cov[i] = 0;
			}

			constexpr auto n_sqrt = constexpr_sqrt(n);
			for (size_t i = 0; i < n_sqrt; ++i)
			{
				cov[(n_sqrt + 1) * i] = x;
			}
		};

		auto odometry_msg = nav_msgs::msg::Odometry();
		odometry_msg.header.frame_id = "odom";
		odometry_msg.child_frame_id = "base_link";
		odometry_msg.twist.twist.linear.x = encoder_to_odom_linear_velocity(data.vx);
		odometry_msg.twist.twist.linear.y = encoder_to_odom_linear_velocity(data.vy);
		odometry_msg.header.stamp = now;

		auto imu_msg = sensor_msgs::msg::Imu();
		imu_msg.header.frame_id = "odom";
		imu_msg.angular_velocity.x = mpu_to_gyro_angular_velocity(data.roll);
		imu_msg.angular_velocity.y = mpu_to_gyro_angular_velocity(data.pitch);
		imu_msg.angular_velocity.z = mpu_to_gyro_angular_velocity(data.yaw);
		imu_msg.linear_acceleration.x = mpu_to_accelaration(data.ax);
		imu_msg.linear_acceleration.y = mpu_to_accelaration(data.ay);
		imu_msg.linear_acceleration.z = mpu_to_accelaration(data.az);

		imu_msg.header.stamp = now;

		set_covariance(odometry_msg.pose.covariance, 1);
		set_covariance(odometry_msg.twist.covariance, 1);
		set_covariance(imu_msg.angular_velocity_covariance, 1);
		set_covariance(imu_msg.linear_acceleration_covariance, 1);
		set_covariance(imu_msg.orientation_covariance, 1);

		return {odometry_msg, imu_msg};
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdomCheckNode>());
	rclcpp::shutdown();
	return 0;
}
