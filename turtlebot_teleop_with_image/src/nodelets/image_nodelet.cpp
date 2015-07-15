#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "window_thread.h"

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include "std_msgs/String.h"
#include <sstream>

#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: interaction_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that interaction_view exits.
static void destroyNode(GtkWidget *widget, gpointer data)
{
  /// @todo On ros::shutdown(), the node hangs. Why?
  //ros::shutdown();
  exit(0); // brute force solution
}

static void destroyNodelet(GtkWidget *widget, gpointer data)
{
  // We can't actually unload the nodelet from here, but we can at least
  // unsubscribe from the image topic.
  reinterpret_cast<image_transport::Subscriber*>(data)->shutdown();
}
#endif


namespace interaction_view {

class ImageNodelet : public nodelet::Nodelet
{
  image_transport::Subscriber sub_1;
  image_transport::Subscriber sub_2;

  boost::mutex image_mutex_;
  cv::Mat last_image_;
  
  std::string window_name_;
  int count_;

  ros::Publisher move_pub;

  int flag;

  virtual void onInit();
  
  void imageCb1(const sensor_msgs::ImageConstPtr& msg);
  void imageCb2(const sensor_msgs::ImageConstPtr& msg);

  static void mouseCb(int event, int x, int y, int flags, void* param);

public:
  ImageNodelet();

  ~ImageNodelet();
};

ImageNodelet::ImageNodelet()
  : count_(0)
{
}

ImageNodelet::~ImageNodelet()
{
  cv::destroyWindow(window_name_);
}

void ImageNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();

  flag = 0;

  // Command line argument parsing
  const std::vector<std::string>& argv = getMyArgv();
  // First positional argument is the transport type
  std::string transport;
  local_nh.param("image_transport", transport, std::string("raw"));
  for (int i = 0; i < (int)argv.size(); ++i)
  {
    if (argv[i][0] != '-')
    {
      transport = argv[i];
      break;
    }
  }
  NODELET_INFO_STREAM("Using transport \"" << transport << "\"");
  // Internal option, should be used only by the interaction_view node
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  // Default window name is the resolved topic name
  std::string windows_name_ = "Interaction View";
  local_nh.param("window_name", window_name_, windows_name_);

  bool autosize;
  local_nh.param("autosize", autosize, false);
  
  cv::namedWindow(window_name_, autosize ? cv::WND_PROP_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &ImageNodelet::mouseCb, this);
  
#ifdef HAVE_GTK
  // Register appropriate handler for when user closes the display window
  GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
  if (shutdown_on_close)
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
  else{
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNodelet), &sub_1);
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNodelet), &sub_2);
  }
#endif

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(transport, ros::TransportHints(), getPrivateNodeHandle());
  sub_1 = it.subscribe("/camera/rgb/image_raw", 1, &ImageNodelet::imageCb1, this, hints);
  sub_2 = it.subscribe("/camera/depth/image_raw", 1, &ImageNodelet::imageCb2, this, hints);

  move_pub = nh.advertise<std_msgs::String>("move_topic", 1000);
}

void ImageNodelet::imageCb1(const sensor_msgs::ImageConstPtr& msg)
{
  if(flag==0){
  image_mutex_.lock();

  // We want to scale floating point images so that they display nicely
  if(msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if(max_val > 0)
      last_image_ = float_image / max_val;
    else
      last_image_ = float_image.clone();
  }
  else
  {
    // Convert to OpenCV native BGR color
    try {
      last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
      NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image to bgr8: '%s'",
                             msg->encoding.c_str(), e.what());
    }
  }

  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  image_mutex_.unlock();
  if (!last_image_.empty())
    cv::imshow(window_name_, last_image_);
 }
}

void ImageNodelet::imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
if(flag==1){
  image_mutex_.lock();

  // We want to scale floating point images so that they display nicely
  if(msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if(max_val > 0)
      last_image_ = float_image / max_val;
    else
      last_image_ = float_image.clone();
  }
  else
  {
    // Convert to OpenCV native BGR color
    try {
      last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
      NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image to bgr8: '%s'",
                             msg->encoding.c_str(), e.what());
    }
  }

  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  image_mutex_.unlock();
  if (!last_image_.empty())
    cv::imshow(window_name_, last_image_);
}
}

void ImageNodelet::mouseCb(int event, int x, int y, int flags, void* param)
{
  ImageNodelet *this_ = reinterpret_cast<ImageNodelet*>(param);
  // Trick to use NODELET_* logging macros in static function
  boost::function<const std::string&()> getName =
    boost::bind(&ImageNodelet::getName, this_);

  const cv::Mat &image = this_->last_image_;
  if (image.empty())
  {
    return;
  }

  if (event == cv::EVENT_LBUTTONDOWN)
  {
    if(y<240){
      std_msgs::String msg;
  
      std::stringstream ss;
      ss << "forward";
      msg.data = ss.str();
      this_->move_pub.publish(msg);
      //this_->move_pub.publish(msg);
      ros::spinOnce();
    }
    else if(y>240){
      std_msgs::String msg;
  
      std::stringstream ss;
      ss << "back";
      msg.data = ss.str();
      this_->move_pub.publish(msg);
      //this_->move_pub.publish(msg);
      ros::spinOnce();
    }
  }
  else if (event == cv::EVENT_RBUTTONDOWN){
    if(x<320){
      std_msgs::String msg;
  
      std::stringstream ss;
      ss << "left";
      msg.data = ss.str();
      this_->move_pub.publish(msg);
      //this_->move_pub.publish(msg);
      ros::spinOnce();
    }
    else if(x>320){
      std_msgs::String msg;
  
      std::stringstream ss;
      ss << "right";
      msg.data = ss.str();
      this_->move_pub.publish(msg);
      //this_->move_pub.publish(msg);
      ros::spinOnce();
    }  
  }
  else if(event == cv::EVENT_MBUTTONDOWN)
  {
    this_->flag = 1 - this_->flag;
  }
}

} // namespace interaction_view

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( interaction_view::ImageNodelet, nodelet::Nodelet)
