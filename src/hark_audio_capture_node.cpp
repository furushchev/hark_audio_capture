/*
 * hark_audio_capture_node.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <hark_audio_capture/HarkAudioCaptureConfig.h>
#include <audio_common_msgs/AudioData.h>
#include <hark_msgs/HarkWave.h>


class HarkAudioCapture
{
public:
  typedef hark_audio_capture::HarkAudioCaptureConfig Config;
  HarkAudioCapture(): nh_(""), pnh_("~"), max_channel_(100) {

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType dyn_cb =
      boost::bind(&HarkAudioCapture::config_cb, this, _1, _2);
    srv_->setCallback(dyn_cb);

    ros::SubscriberStatusCallback conn_cb =
      boost::bind(&HarkAudioCapture::connect_cb, this, _1);
    pub_ = nh_.advertise<audio_common_msgs::AudioData>(
      "audio", 1, conn_cb, conn_cb, ros::VoidConstPtr(), false);
  }

  ~HarkAudioCapture(){}

  void connect_cb(const ros::SingleSubscriberPublisher&) {
    if (pub_.getNumSubscribers() == 0) {
      sub_.shutdown();
    } else if (!sub_) {
      sub_ = nh_.subscribe("HarkWave", 1, &HarkAudioCapture::hark_wave_cb, this);
    }
  }

  void config_cb(Config& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config.channel = std::min(max_channel_, config.channel);
    channel_ = config.channel;
  }

  void hark_wave_cb(const hark_msgs::HarkWave::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(mutex_);

    max_channel_ = msg->nch-1;
    channel_ = std::min(max_channel_, channel_);

    uint8_t const *p = reinterpret_cast<uint8_t const*>(msg->src[channel_].wavedata.data());
    audio_common_msgs::AudioData pub_msg;
    pub_msg.data = std::vector<uint8_t>(p, p + msg->length * 32 / 8);
    pub_.publish(pub_msg);
  }

  boost::mutex mutex_;
  boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  int channel_;
  int max_channel_;
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "hark_audio_capture");
  HarkAudioCapture n;
  ros::spin();

  return 0;
}
