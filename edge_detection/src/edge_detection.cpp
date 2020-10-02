#include <edge_detection/edge_detection.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <ros/ros.h>

namespace towr {

    EdgeDetection::EdgeDetection (ros::NodeHandle& node_handle, double & min_length, double & min_height):
    node_handle_(node_handle),
    min_length_(min_length),
    min_height_(min_height)
    {
      edges_publisher_.reset(
                    new locomotion_viewer::LocomotionViewer("point_cloud_odom", "/edge_detection/detected_edges", node_handle_));
      max_height_ = 1.0;
      std::cout<<"[EdgeDetection::detectEdges] min length: "<<min_length_<<std::endl;
      std::cout<<"[EdgeDetection::detectEdges] min height: "<<min_height_<<std::endl;
      std::cout<<"[EdgeDetection::detectEdges] max height: "<<max_height_<<std::endl;

    }

    EdgeDetection::~EdgeDetection()
    {
    }

    bool EdgeDetection::advance(const Eigen::Vector3d & base_pose, const grid_map_msgs::GridMap & message){
      grid_map::GridMapRosConverter::fromMessage(message, gridMap_);

      cv::Mat image, im_edges, im_copy;
      grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
              gridMap_, "elevation", CV_8UC1, image);

      // Check if image is loaded fine
      if(image.empty()){
        printf(" Error opening image\n");
      }
      // Edge detection
      cv::Canny(image, im_edges, 50, 200, 3);
      // Standard Hough Line Transform
      std::vector<cv::Vec4i> lines; // will hold the results of the detection
      //cv::HoughLines(im_edges, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection

      // Probabilistic Line Transform
      HoughLinesP(im_edges, lines, 1, CV_PI/180, 5, 5, 5 ); // runs the actual detection

      //imwrite("image.png",image);
      //imwrite("edge_det.png",im_edges);

      checkPreviousEdges();

      findNewEdges(lines);
      //setFakeEdges();

      closest_orthogonal_edge_index_ = findNextEdge(base_pose);

      std::cout<<"[EdgeDetection::detectEdges] number of detected edges: "<<edges_.size()<<std::endl;

      if(edges_.size()!=0){
        EdgeContainer next_edge = edges_.at(closest_orthogonal_edge_index_);
        edge_direction_ = next_edge.line_coeffs;
      }

      return true;
    }

    void EdgeDetection::setFakeEdges(){

      Eigen::MatrixXd fake_points_wf(10,4);
      fake_points_wf << 0.74, 0.5, 0.74, -0.5,
      1.26, 0.5, 1.26, -0.5,
      1.88, 0.5, 1.88, -0.5,
      2.12, 0.5, 2.12, -0.5,
      3.0, 0.5, 3.0, -0.5,
      4.0, 0.5, 4.0, -0.5,
      4.74, 0.5, 4.74, -0.5,
      5.26, 0.5, 5.26, -0.5,
      6.38, 0.5, 6.38, -0.5,
      6.62, 0.5, 6.62, -0.5;

      Eigen::VectorXd fake_height(10);
      fake_height << 0.06, 0.06, 0.12, 0.12, 0.25, 0.25, 0.18, 0.18, 0.12, 0.12;
            
      for( size_t i = 0; i < 10; i++ ) {
        EdgeContainer new_edge;
        new_edge.point1_wf[0] = fake_points_wf(i,0);
        new_edge.point1_wf[1] = fake_points_wf(i,1);
        new_edge.point2_wf[0] = fake_points_wf(i,2);
        new_edge.point2_wf[1] = fake_points_wf(i,3);
        new_edge.length = 1.0;
        new_edge.height = fake_height[i];
        new_edge.yaw = 1.57;
        new_edge.line_coeffs = Eigen::Vector2d(sin(new_edge.yaw), cos(new_edge.yaw));

        if (!isEdgeRedundant(new_edge.point1_wf, new_edge.point2_wf)) {
          edges_.push_back(new_edge);
          orthogonal_edge_indices_.push_back(i);
        }
        //std::cout<<"[EdgeDetection::detectEdges] edge lenght: "<<new_edge.length<<std::endl;
        //std::cout<<"[EdgeDetection::detectEdges] edge height: "<<new_edge.height<<std::endl;
        //std::cout<<"[EdgeDetection::detectEdges] edge angle: "<<new_edge.yaw<<std::endl;

      }
    }

    void EdgeDetection::checkPreviousEdges(){
      if(edges_.size()>0){
        std::vector<towr::EdgeContainer> edges_tmp = edges_;
        edges_.clear();
        for( size_t i = 0; i < edges_tmp.size(); i++ ){
          double height_check = computeStepHeight(edges_tmp.at(i).point1_wf, edges_tmp.at(i).point2_wf, edges_tmp.at(i).z);
          if(fabs(height_check) > min_height_){
            edges_.push_back(edges_tmp.at(i));
          }
        }
      }


    }

    void EdgeDetection::findNewEdges(const std::vector<cv::Vec4i> & lines){
      orthogonal_edge_indices_.clear();

      for( size_t i = 0; i < lines.size(); i++ ) {
        cv::Vec4i l = lines[i];
        //std::cout<<"line n. "<<i<<" p1x "<<l[0]<<" p1y "<<l[1]<<" p2x "<<l[2]<<" p2y "<<l[3]<<std::endl;
        EdgeContainer new_edge;
        new_edge.point1_wf = convertImageToOdomFrame(gridMap_.getResolution(), gridMap_.getSize(), l[0], l[1]);
        new_edge.point2_wf = convertImageToOdomFrame(gridMap_.getResolution(), gridMap_.getSize(), l[2], l[3]);
        //new_edge.middle_point_bf = (new_edge.point1_bf + new_edge.point2_bf) / 2.0;
        new_edge.length = computeLength(new_edge.point1_wf, new_edge.point2_wf);
        if (new_edge.length > min_length_) {
          //std::cout << "[EdgeDetection::detectEdges] new edge > 0.5 " << std::endl;
          double edge_yaw_bf = computeEdgeOrientation(new_edge.point1_wf, new_edge.point2_wf);
          new_edge.line_coeffs = Eigen::Vector2d(sin(edge_yaw_bf), cos(edge_yaw_bf));
          double delta_range = M_PI / 6.0;
          //if ((edge_yaw_bf > M_PI / 2.0 - delta_range) && (edge_yaw_bf < M_PI / 2.0 + delta_range) ||
          //    (edge_yaw_bf < -M_PI / 2.0 + delta_range) && (edge_yaw_bf > -M_PI / 2.0 - delta_range)) {
            new_edge.height = computeStepHeight(new_edge.point1_wf, new_edge.point2_wf, new_edge.z);
            if ((fabs(new_edge.height) > min_height_)&&(fabs(new_edge.height) < max_height_)){
              if (!isEdgeRedundant(new_edge.point1_wf, new_edge.point2_wf)) {
                new_edge.yaw = computeEdgeOrientation(new_edge.point1_wf, new_edge.point2_wf);
                new_edge.line_coeffs = Eigen::Vector2d(sin(new_edge.yaw), cos(new_edge.yaw));
                //new_edge.distance_from_base = computeDistanceFromBase(new_edge.point1_bf, new_edge.point2_bf);
                edges_.push_back(new_edge);
                orthogonal_edge_indices_.push_back(i);

                //std::cout<<"[EdgeDetection::detectEdges] edge lenght: "<<new_edge.length<<std::endl;
                //std::cout<<"[EdgeDetection::detectEdges] edge height: "<<new_edge.height<<std::endl;
                //std::cout<<"[EdgeDetection::detectEdges] edge angle: "<<new_edge.yaw<<std::endl;
                //std::cout<<"[EdgeDetection::detectEdges] min_dist: "<<new_edge.distance_from_base<<std::endl;

              }
            }
          //}
        }
      }

    }

    edge_idx EdgeDetection::findNextEdge(const Eigen::Vector3d & base_pose){
      double min_dist = 10000.0;
      edge_idx closest_idx;
      for( size_t i = 0; i < edges_.size(); i++ ){
        Eigen::Vector2d base_pos = Eigen::Vector2d(base_pose[0], base_pose[1]);
        double distance_from_base = computeDistanceBtwEdgeAndBaseInWorldFrame(edges_.at(i).point1_wf, edges_.at(i).point2_wf, base_pos);
        if(distance_from_base<min_dist){
          min_dist = distance_from_base;
          closest_idx = i;
          //std::cout<<"[EdgeDetection::detectEdges] edge index: "<<i<<std::endl;
          //std::cout<<"[EdgeDetection::detectEdges] edge distance_from_base: "<<edges_.at(i).distance_from_base<<std::endl;
          //std::cout<<"[EdgeDetection::detectEdges] edge lenght: "<<edges_.at(i).length<<std::endl;
          //std::cout<<"[EdgeDetection::detectEdges] edge height: "<<edges_.at(i).height<<std::endl;
          //std::cout<<"[EdgeDetection::detectEdges] edge angle: "<<edges_.at(i).yaw<<std::endl;
        }
      }

      return closest_idx;
    }

    void EdgeDetection::plotEdges(){
      edges_publisher_->deleteAllMarkers();

      for( size_t i = 0; i < edges_.size(); i++ )
      {
        Eigen::VectorXd x(5), y(5);
        x[0] = edges_.at(i).point1_wf[0]; y[0] = edges_.at(i).point1_wf[1];
        x[1] = edges_.at(i).point2_wf[0]; y[1] = edges_.at(i).point2_wf[1];
        x[2] = edges_.at(i).point2_wf[0]; y[2] = edges_.at(i).point2_wf[1];
        x[3] = edges_.at(i).point1_wf[0]; y[3] = edges_.at(i).point1_wf[1];
        x[4] = edges_.at(i).point1_wf[0]; y[4] = edges_.at(i).point1_wf[1];

        Eigen::VectorXd z(5);
        z[0] = edges_.at(i).z;
        z[1] = edges_.at(i).z;
        z[2] = edges_.at(i).z - fabs(edges_.at(i).height);
        z[3] = edges_.at(i).z - fabs(edges_.at(i).height);
        z[4] = edges_.at(i).z;

        rviz_visual_tools::colors rviz_color;
        if(closest_orthogonal_edge_index_==i){
          rviz_color = rviz_visual_tools::GREEN;

          //std::cout<<"[EdgeDetection::detectEdges] edge index: "<<i<<std::endl;
          //std::cout<<"[EdgeDetection::detectEdges] edge lenght: "<<edges_.at(i).length<<std::endl;
          //std::cout<<"[EdgeDetection::detectEdges] edge height: "<<edges_.at(i).height<<std::endl;
          //std::cout<<"[EdgeDetection::detectEdges] edge angle: "<<edges_.at(i).yaw<<std::endl;
        }else{
          rviz_color = rviz_visual_tools::RED;
        }
        edges_publisher_->publishEigenPath(x, y, z, rviz_color, rviz_visual_tools::MEDIUM);
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().x() = x[0];
        text_pose.translation().y() = y[0];
        text_pose.translation().z() = edges_.at(i).z;
        edges_publisher_->publishText(text_pose, std::to_string(i), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE, false);
        //edges_publisher_->publishText(text_pose, std::to_string(orthogonal_edge_indices_.at(next_i)));
      }

      edges_publisher_->trigger();
    }

    Eigen::Vector2d EdgeDetection::convertImageToOdomFrame(const double & resolution, const Eigen::Array2i & grid_size, const int & p_x, const int & p_y){

      double gsx = grid_size[0];
      double gsy = grid_size[1];
      double p_x_base = (- p_y + gsy/2.0)*resolution;
      double p_y_base = (- p_x + gsx/2.0)*resolution;
      Eigen::Vector2d pos_bf = Eigen::Vector2d(p_x_base, p_y_base);
      Eigen::Vector2d pos_odom_frame = pos_bf+gridMap_.getPosition();
      return pos_odom_frame;
    }

    double EdgeDetection::GetHeight(double & x, double & y) {
      grid_map::Position pos = {x,y};
      if (!gridMap_.isInside(pos)){
        //std::cout<<"pos: "<<pos<<" is NOT inside gridmap"<<std::endl;
        return 0.;
      }
      else {
        double height = static_cast<double>(gridMap_.atPosition("elevation", pos, grid_map::InterpolationMethods::INTER_NEAREST));
        //std::cout<<"pos: "<<pos<<" is inside gridmap"<<std::endl;
        //std::cout<<"terrain height is "<<height<<std::endl;
        if (std::isnan(height)) {
          return 0.;
        }
        return height;
      }
    }

    double EdgeDetection::computeLength(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2){
      return sqrt(pow(p1[0] - p2[0],2) + pow(p1[1] - p2[1],2));
    }

    bool EdgeDetection::isInsideEllipse(const double & edge_yaw, const Eigen::Vector2d & ellipse_center, const Eigen::Vector2d & p){
      Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(-edge_yaw + M_PI/2.0, Eigen::Vector3d::UnitZ());
      Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
      Eigen::Matrix3d rotationMatrix = q.matrix();
      Eigen::Vector2d point2d = p - ellipse_center;
      Eigen::Vector2d point = (rotationMatrix*Eigen::Vector3d(point2d[0], point2d[1], 0.0)).segment(0,2);
      double d1 = min_length_/10.0;
      double d2 = min_length_/2.0;
      double a2 = pow(d1,2);
      double b2 = pow(d2,2);
      double y = pow(point[0],2)/a2 + pow(point[1],2)/b2;
      if (y<=1.0){
        return true;
      }else{
        return false;
      }
    }

    double EdgeDetection::computeEdgeOrientation(const Eigen::Vector2d & p1_bf, const Eigen::Vector2d & p2_bf){
      double yaw_angle = atan2(p1_bf[1] - p2_bf[1], p1_bf[0] - p2_bf[0]);
      return yaw_angle;
    }

    double EdgeDetection::computeStepHeight(const Eigen::Vector2d & p1_wf, const Eigen::Vector2d & p2_wf, double & z){
      Eigen::Vector2d middle_point_wf = (p1_wf + p2_wf)/2.0;
      double edge_yaw = computeEdgeOrientation(p1_wf, p2_wf);
      Eigen::Vector2d edge_normal = Eigen::Vector2d(sin(edge_yaw), cos(edge_yaw));
      double epsilon = 0.1;
      Eigen::Vector2d middle_point_wf_plus = middle_point_wf + edge_normal*epsilon;
      Eigen::Vector2d middle_point_wf_minus = middle_point_wf - edge_normal*epsilon;
      double z1 = GetHeight(middle_point_wf_plus[0], middle_point_wf_plus[1]);
      double z2 = GetHeight(middle_point_wf_minus[0], middle_point_wf_minus[1]);
      z = std::max(z1, z2);
      //std::cout<<"[EdgeDetection::computeStepHeight] edge_yaw: "<<edge_yaw<<std::endl;
      //std::cout<<"[EdgeDetection::computeStepHeight] edge_normal: "<<edge_normal.transpose()<<std::endl;
      //std::cout<<"[EdgeDetection::computeStepHeight] middle_point_wf_plus: "<<middle_point_wf_plus.transpose()<<std::endl;
      //std::cout<<"[EdgeDetection::computeStepHeight] middle_point_wf_minus: "<<middle_point_wf_minus.transpose()<<std::endl;
      //std::cout<<"[EdgeDetection::computeStepHeight] z1: "<<z1<<std::endl;
      //std::cout<<"[EdgeDetection::computeStepHeight] z2: "<<z2<<std::endl;
      return z1 - z2;
    }

    double EdgeDetection::computeDistanceFromBase(const Eigen::Vector2d & p1_bf, const Eigen::Vector2d & p2_bf){
      double num = fabs(p2_bf[0]*p1_bf[1] - p2_bf[1]*p1_bf[0]);
      double denum = computeLength(p1_bf, p2_bf);
      return num/denum;
    }

    double EdgeDetection::computeDistanceBtwEdgeAndBaseInWorldFrame(const Eigen::Vector2d & p1_wf,
            const Eigen::Vector2d & p2_wf,
            const Eigen::Vector2d & base_pos){

      double num = fabs((p2_wf[1] - p1_wf[1])*base_pos[0] - (p2_wf[0] - p1_wf[0])*base_pos[1] + p2_wf[0]*p1_wf[1] - p2_wf[1]*p1_wf[0]);
      double denum = computeLength(p1_wf, p2_wf);
      return num/denum;
    }

    bool EdgeDetection::isEdgeRedundant(const Eigen::Vector2d & p1_wf, const Eigen::Vector2d & p2_wf){
      //std::cout<<"[EdgeDetection::isEdgeRedundant] edges_.size()"<<edges_.size()<<std::endl;
      //std::cout<<"[EdgeDetection::isEdgeRedundant] p1_wf"<<p1_wf.transpose()<<std::endl;
      //std::cout<<"[EdgeDetection::isEdgeRedundant] p2_wf"<<p2_wf.transpose()<<std::endl;
      for( size_t i = 0; i < edges_.size(); i++ )
      {
        //std::cout<<"[EdgeDetection::isEdgeRedundant] edges_.at(i).point1_wf"<<edges_.at(i).point1_wf.transpose()<<std::endl;
        //std::cout<<"[EdgeDetection::isEdgeRedundant] edges_.at(i).point2_wf"<<edges_.at(i).point2_wf.transpose()<<std::endl;
        bool d11 = isInsideEllipse(edges_.at(i).yaw, edges_.at(i).point1_wf, p1_wf);
        bool d12 = isInsideEllipse(edges_.at(i).yaw, edges_.at(i).point2_wf, p1_wf);
        bool d21 = isInsideEllipse(edges_.at(i).yaw, edges_.at(i).point1_wf, p2_wf);
        bool d22 = isInsideEllipse(edges_.at(i).yaw, edges_.at(i).point2_wf, p2_wf);
        if((d11&&d22)||(d12&&d21)){
          //std::cout<<"[EdgeDetection::isEdgeRedundant] edge is redundant!"<<std::endl;
          return true;
        }
      }
      std::cout<<"[EdgeDetection::isEdgeRedundant] edge is not redundant!"<<std::endl;
      return false;
    }

    double EdgeDetection::getEdgeDistanceFromBase(const Eigen::Vector3d & base_pose){
      Eigen::Vector2d base_pos = Eigen::Vector2d(base_pose[0], base_pose[1]);
      double distance_from_base = computeDistanceBtwEdgeAndBaseInWorldFrame(edges_.at(closest_orthogonal_edge_index_).point1_wf,
              edges_.at(closest_orthogonal_edge_index_).point2_wf,
              base_pos);
      return distance_from_base;
    }

    Eigen::Vector2d EdgeDetection::getPointAlongEdgeInWorldFrame(){
      Eigen::Vector2d middle_point_wf = (edges_.at(closest_orthogonal_edge_index_).point1_wf + edges_.at(closest_orthogonal_edge_index_).point2_wf)/2.0;
      return middle_point_wf;
    }

    Eigen::Vector2d EdgeDetection::getEdgeDirectionInBaseFrame(){
      return edge_direction_;
    }

    double EdgeDetection::getNextStepHeight(){
      return edges_.at(closest_orthogonal_edge_index_).height;
    }

    double EdgeDetection::getEdgeYawAngleInWorldFrame(){
      return computeEdgeOrientation(edges_.at(closest_orthogonal_edge_index_).point1_wf, edges_.at(closest_orthogonal_edge_index_).point2_wf);
    }

    bool EdgeDetection::edgeFound(){
      if(edges_.size()==0){
        return false;
      }else{
        return true;
      }
    }
}
