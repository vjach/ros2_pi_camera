#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "core/libcamera_encoder.hpp"
#include "output/output.hpp"
#include "h264_msgs/msg/packet.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      h264_pub_ = this->create_publisher<h264_msgs::msg::Packet>("h264_image", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
    void AddData(uint8_t *buffer, size_t size, int64_t timestamp, bool iframe) {
        static uint64_t seq {0};
          h264_msgs::msg::Packet h264_msg;
          h264_msg.header.frame_id = "camera_frame";
          h264_msg.seq = seq++;

          auto stamp = now();

          // Copy to the ROS message and free the packet
//if (h264_pub_->get_subscription_count() > 0) {
            h264_msg.data.insert(h264_msg.data.end(), &buffer[0], &buffer[size]);
            h264_msg.header.stamp = stamp;
            h264_pub_->publish(h264_msg);
          }
//    }

  private:
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub_;
    size_t count_;
};

std::shared_ptr<MinimalPublisher> node;

void DataReady(void *buffer, size_t size, int64_t timestamp, bool iframe) {
    node->AddData(reinterpret_cast<uint8_t*>(buffer), size, timestamp, iframe);

}
void MetaDataReady(libcamera::ControlList&) {
}

static void event_loop(LibcameraEncoder &app)
{
	VideoOptions const *options = app.GetOptions();
	//std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(DataReady);
	app.SetMetadataReadyCallback(MetaDataReady);

	app.OpenCamera();
	app.ConfigureVideo(LibcameraEncoder::FLAG_VIDEO_NONE);
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
	{
		LibcameraEncoder::Msg msg = app.Wait();
		if (msg.type == LibcameraApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == LibcameraEncoder::MsgType::Quit)
			return;
		else if (msg.type != LibcameraEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->frames && options->timeout &&
					   ((now - start_time) > options->timeout.value);
		bool frameout = options->frames && count >= options->frames;

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		app.EncodeBuffer(completed_request, app.VideoStream());
		app.ShowPreview(completed_request, app.VideoStream());
	}
}

int camera_thread() {
    char * argv[] = {"",
        "--width", "1280",
        "--height", "1080",
        "--codec", "h264",
    };
    int argc = 7;
	try
	{
		LibcameraEncoder app;
		VideoOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<MinimalPublisher>();
  auto cam_thread = std::thread(camera_thread);
  rclcpp::spin(node);
  rclcpp::shutdown();
  cam_thread.join();
  return 0;
}
