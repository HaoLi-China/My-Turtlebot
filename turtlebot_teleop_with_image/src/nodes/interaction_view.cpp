#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interaction_view", ros::init_options::AnonymousName);

  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string my_argv(argv + 1, argv + argc);
  my_argv.push_back("--shutdown-on-close"); // Internal

  manager.load(ros::this_node::getName(), "interaction_view/image", remappings, my_argv);

  ros::spin();
  return 0;
}
