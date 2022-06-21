/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#include <boost/core/enable_if.hpp>
#include <depthimage_to_laserscan/DepthImageToLaserScanROS.h>

using namespace depthimage_to_laserscan;

DepthImageToLaserScanROS::DepthImageToLaserScanROS(ros::NodeHandle& n, ros::NodeHandle& pnh):pnh_(pnh), it_(n), srv_(pnh) {
  boost::mutex::scoped_lock lock(connect_mutex_);

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<depthimage_to_laserscan::DepthConfig>::CallbackType f;
  f = boost::bind(&DepthImageToLaserScanROS::reconfigureCb, this, _1, _2);
  srv_.setCallback(f);

  // Lazy subscription to depth image topic
  pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&DepthImageToLaserScanROS::connectCb, this, _1), boost::bind(&DepthImageToLaserScanROS::disconnectCb, this, _1));

  n.getParam("depthimage_to_laserscan/filter_sensor", filter_sensor_);
  std::cout << "Filter measurements from sensor: " << filter_sensor_ << std::endl;
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS(){
  sub_.shutdown();
}
void plot_image(char * name , cv::Mat map){
  double min;
  double max;
  cv::minMaxIdx(map, &min, &max);
  cv::Mat adjMap;
  min=1<<4;
  max=(1<<16)/10;
  std::cout<<"min: "<<min<<"max: "<<max<<std::endl;
  // expand your range to 0..255. Similar to histEq();
  map.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 

  // this is great. It converts your grayscale image into a tone-mapped one, 
  // much more pleasing for the eye
  // function is found in contrib module, so include contrib.hpp 
  // and link accordingly
  cv::Mat falseColorsMap;
  applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_HOT);

  // cv::imshow(name, falseColorsMap);

}
void filterDepth(cv::Mat input, cv::Mat &_output){
  cv::Mat zeros = input*0;
  // output= input;
  cv::Mat output(cv::Size(input.cols,input.rows), CV_16UC1, (void*)input.data, cv::Mat::AUTO_STEP);
  
  std::cout<<output.rows<<' '<<output.cols<<std::endl;
  
  for(int i=0  ; i<2*output.rows; i+=1){
    for(int j=0 ; j<output.cols; j++){
      if(output.data[(output.cols*i + j)] <(1)){
        output.data[(output.cols*i + j)] = 255; //65535; //2**16
        zeros.data[(output.cols*i + j)] = 1;
      }
    }
  }

  // for (int i = 0; i<output.rows; i++)
  // for( int j =0 ; j<output.cols; j++){
  //   if(output.data[(output.rows*j + i)] ==0){
  //     output.data[(output.rows*j + i)] = 65535; //2**16
  //     zeros.data[(output.rows*j + i)] = 1;
  //   }
  // }
  
  int morph_size =10;
  cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,  cv::Size( 2*morph_size + 1, 10*morph_size+1 ), cv::Point(-1,-1));
    cv::erode(output,output, element);
    for (int i = 0; i<2*output.rows; i++)
  for( int j =0 ; j<output.cols; j++){
    if(zeros.data[(output.cols*i + j)]){
      output.data[(output.cols*i + j)] = 0; //2**16
    }
  }  
  _output = output;
}

void DepthImageToLaserScanROS::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
        const sensor_msgs::CameraInfoConstPtr& info_msg){
  try
  {
    sensor_msgs::LaserScan scan_msg;
    if (filter_sensor_)
    {
    sensor_msgs::Image image_filtered = *depth_msg.get();
    cv::Mat image = cv_bridge::toCvShare(depth_msg,image_filtered.encoding)->image;
    // plot_image("original", image);
    filterDepth(image,image);
    // plot_image("filtered", image);
    
    // flip image upside-down
    // cv::flip(image,image,-1);
    // plot_image("flipped", image);
   
    // create a sensor msgs::Image from the cv::Mat
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(image_filtered.header, image_filtered.encoding, image).toImageMsg();
    cv::waitKey(1);
    scan_msg = *dtl_.convert_msg(image_msg, info_msg).get();
    }
    else {
      scan_msg = *dtl_.convert_msg(depth_msg, info_msg).get();
    }
    pub_.publish(scan_msg);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
  }
}

void DepthImageToLaserScanROS::connectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (!sub_ && pub_.getNumSubscribers() > 0) {
    ROS_DEBUG("Connecting to depth topic.");
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    sub_ = it_.subscribeCamera("image", 10, &DepthImageToLaserScanROS::depthCb, this, hints);
  }
}

void DepthImageToLaserScanROS::disconnectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0) {
    ROS_DEBUG("Unsubscribing from depth topic.");
    sub_.shutdown();
  }
}

void DepthImageToLaserScanROS::reconfigureCb(depthimage_to_laserscan::DepthConfig& config, uint32_t level){
    dtl_.set_scan_time(config.scan_time);
    dtl_.set_range_limits(config.range_min, config.range_max);
    dtl_.set_scan_height(config.scan_height);
    dtl_.set_output_frame(config.output_frame_id);
}
