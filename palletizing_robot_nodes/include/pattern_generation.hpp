#ifndef PATTERN_GENERATION_HPP
#define PATTERN_GENERATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <tuple>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace pattern_generation {

class PatternGenerator{
    public:
    PatternGenerator(geometry_msgs::msg::Pose approach_pick, std::string generation_type) 
        : approach_pick{approach_pick}, generation_type{generation_type}
    {
        amount_of_layers = 1;
        amount_per_layer = 0;
        if(generation_type == "Columns"){
            generateColumnLayer();
            generateColumnPattern();
        }
        else if(generation_type == "Brick"){
            generateBrickLayer();
            generateBrickPattern(false);
        }
        else if(generation_type == "Well"){
            generateWellLayer();
            generateWellPattern(false);
        }
        else if(generation_type == "Optimized"){ 
            generateOptimizedLayer(); 
            generateOptimizedPattern(false);
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "Error when chosing pattern generation type, please enter: Columns, Brick, Well or Optimized.");
        }
    }
    void generateNewLayer(){
        amount_of_layers++;
        z = 0.246 + (amount_of_layers - 1)*0.1;
        if(generation_type == "Columns"){
            generateColumnPattern();
        }
        else if(generation_type == "Brick"){
            if(amount_of_layers%2 == 0){
                generateBrickPattern(true);
            } else {
                generateBrickPattern(false);
            }
        }
        else if(generation_type == "Well"){
            if(amount_of_layers%2 == 0){
                generateWellPattern(true);
            } else {
                generateWellPattern(false);
            }
        }
        else if(generation_type == "Optimized"){ 
            if(amount_of_layers%2 == 0){
                generateOptimizedPattern(true);
            } else {
                generateOptimizedPattern(false);
            }
        }
        
    }
    // Getters
    geometry_msgs::msg::Pose getApproachPlacePoint(int index) const {
        return approach_place_points[index];
    }
    geometry_msgs::msg::Pose getMiddlePoint(int index) const {
        return middle_points[index];
    }
    geometry_msgs::msg::Pose getPlacePosePoint(int index) const {
        return place_pose_points[index];
    }
    int getAmountPerLayer(){ return amount_per_layer*amount_of_layers; }
    private:
    // Basic parameters
    geometry_msgs::msg::Pose approach_pick;
    geometry_msgs::msg::Pose approach_place;
    geometry_msgs::msg::Pose place_pose;
    std::vector<geometry_msgs::msg::Pose> approach_place_points;
    std::vector<geometry_msgs::msg::Pose> middle_points;
    std::vector<geometry_msgs::msg::Pose> place_pose_points;
    int amount_per_layer;
    int amount_of_layers;
    std::string generation_type;
    double z = 0.246;
    // Optimization parameters
    int Px = 1200;
    int Py = 800;
    int Bx = 300;
    int By = 200;
    int Hx = Bx/2;
    int Hy = By/2;
    int Vx = By/2;
    int Vy = Bx/2;
    std::vector<geometry_msgs::msg::Pose> point_array;
    // Generation functions
    void generateColumnLayer(){
        geometry_msgs::msg::Pose point;
        point.orientation.z = -0.7068252;
        point.orientation.w = 0.7073883;

        for(int y = 0; y<4; y++){
            for(int x = 0; x<4; x++){
                point.position.x = 150 + x*300;
                point.position.y = 100 + y*200;
                point.position.z = z;

                point_array.push_back(point);
                amount_per_layer++;
            }
        }
    }
    void generateBrickLayer(){
        geometry_msgs::msg::Pose point;
        
        for(int x = 0; x<6; x++){
            point.position.x = 100 + x*200;
            point.position.y = 150;
            point.position.z = z;

            if(x<3){
                point.orientation.z = -1.0;
                point.orientation.w = 0.0;
            } else{
                point.orientation.z = 0.0;
                point.orientation.w = 1.0;
            }

            point_array.push_back(point);
            amount_per_layer++;
        }

        point.orientation.z = -0.7068252;
        point.orientation.w = 0.7073883;

        for(int y = 0; y<2; y++){
            for(int x = 0; x<4; x++){
                point.position.x = 150 + x*300;
                point.position.y = 400 + y*200;
                point.position.z = z;

                point_array.push_back(point);
                amount_per_layer++;
            }
        }
    }
    void generateWellLayer(){
        geometry_msgs::msg::Pose point;
        point_array.clear();
        amount_per_layer = 0;
        point.orientation.z = -0.7068252;
        point.orientation.w = 0.7073883;
        for(int x = 0; x<6; x++){
            if(x<2){
                point.position.x = 150;
                point.position.y = 100 + x*200;
                point.position.z = z;
            } else{
                point.orientation.z = 0.0;
                point.orientation.w = 1.0;
                point.position.x = 400 + (x-2)*200;
                point.position.y = 150;
                point.position.z = z;
            }

            point_array.push_back(point);
            amount_per_layer++;
        }

        point.orientation.z = -0.7068252;
        point.orientation.w = 0.7073883;
        for(int x = 0; x<6; x++){
            if(x<2){
                point.position.x = 950;
                point.position.y = 400 + x*200;
                point.position.z = z;
            } else{
                point.orientation.z = -1.0;
                point.orientation.w = 0.0;

                point.position.x = 700 - (x-2)*200;
                point.position.y = 550;
                point.position.z = z;
            }

            point_array.push_back(point);
            amount_per_layer++;
        }
    }
    void generateMiddlePoint(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose finish){
        geometry_msgs::msg::Pose middle_point;
        tf2::Quaternion q_start, q_target;
        tf2::fromMsg(start.orientation, q_start);
        tf2::fromMsg(finish.orientation, q_target);
        tf2::Quaternion q_middle = q_start.slerp(q_target, 0.5);

        middle_point.position.x = (start.position.x + finish.position.x) / 2.0;
        middle_point.position.y = (start.position.y + finish.position.y) / 2.0;
        middle_point.position.z = (start.position.z + finish.position.z) / 2.0;
        middle_point.orientation = tf2::toMsg(q_middle);

        if(middle_point.position.y>=-0.32){
            middle_point.position.y = -0.4;
        }

        middle_points.push_back(middle_point);
    }
    // Optimization
    std::tuple<int, int, int> findBestFit(int l, int Bx, int By){
        int best_x = 0;
        int best_y = 0;
        int max_length = 0;

        for (int x = 1; x <= l / Bx; x++) {
            for (int y = 1; y <= l / By; y++) {
                int length = x * Bx + y * By;
                if (length <= l && length > max_length) {
                    max_length = length;
                    best_x = x;
                    best_y = y;
                }
            }
        }

        return std::make_tuple(best_x, best_y, max_length);
    }
    void convertPoints(std::vector<geometry_msgs::msg::Pose> &points_array, bool mirror, const std::array<double, 3>& offsets){
        const double dx = offsets[0];
        const double dy = offsets[1];
        const double dz = offsets[2];
    
        const double mirror_axis_y = -0.9;
    
        for (auto &point : points_array) {
            place_pose.position.x = (point.position.x / 1000) + dx;
            place_pose.position.y = (point.position.y / 1000) + dy;
            place_pose.position.z = z + dz;
            place_pose.orientation = point.orientation;
    
            if (mirror) {
                place_pose.position.y = 2 * mirror_axis_y - place_pose.position.y;
            }
            approach_place = place_pose;
            approach_place.position.z = place_pose.position.z + 0.3;
            generateMiddlePoint(approach_pick, approach_place);
            approach_place_points.push_back(approach_place);
            place_pose_points.push_back(place_pose);
        }
    }
    void generateOptimizedLayer(){
        auto [n1, m1, px] = findBestFit(Px, Bx, By);
        auto [n2, m2, py] = findBestFit(Py, Bx, By);
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "%d * %d + %d * %d = %d", n1, Bx, m1, By, px);
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "%d * %d + %d * %d = %d", n2, Bx, m2, By, py);
        Px = px;
        Py = py;

        bool overlap = false;
        bool vertical = false;
        int L1 = n1*Bx;
        int L2 = m1*By;
        int W1 = n2*Bx;
        int W2 = m2*By;

        if(L1>L2){
            if(W1>=W2){
                // 1 and 3 area are larger and non-overlaped
                overlap = false;
                vertical = false;
            } else if(W1<W2){
                // 1 and 3 area are larger and overlaped (Horizontal overlap)
                overlap = true;
                vertical = false;
            }
        }
        if(L1<=L2){
            if(W1<=W2){
                // 2 and 4 area are larger and non-overlaped
                overlap = false;
                vertical = false;
            } else if(W1>W2){
                // 2 and 4 area are larger and overlaped (Vertical overlap)
                overlap = true;
                vertical = true;
            }
        }

        int nprim = 0;
        int mprim = 0;
        if(overlap){
            if(vertical){
                nprim = static_cast<int>(std::ceil(abs((2.0*L2 - Px)/By)));
                mprim = static_cast<int>(std::ceil(abs((2.0*W2 - Py)/Bx) / 2.0));
            } else {
                nprim = static_cast<int>(std::ceil(abs((2.0*L1 - Px)/Bx) / 2.0));
                mprim = static_cast<int>(std::ceil(abs((2.0*W1 - Py)/By)));
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "%d", nprim);
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "%d", mprim);
        geometry_msgs::msg::Pose point;

        // Block 1
        if(overlap && !vertical){
            for(int i = 0; i<n1; i++){
                if(i<n1 - nprim){
                    for(int j = 0; j<m2; j++){
                        point.orientation.z = -0.7068252;
                        point.orientation.w = 0.7073883;
                        point.position.x = Hx + i*Bx;
                        point.position.y = Hy + j*By;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                } else {
                    for(int j = 0; j<m2 - mprim; j++){
                        point.orientation.z = -0.7068252;
                        point.orientation.w = 0.7073883;
                        point.position.x = Hx + i*Bx;
                        point.position.y = Hy + j*By;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                }
            }
        } else {
            for(int i = 0; i<n1; i++){
                for(int j = 0; j<m2; j++){
                    point.orientation.z = -0.7068252;
                    point.orientation.w = 0.7073883;
                    point.position.x = Hx + i*Bx;
                    point.position.y = Hy + j*By;
                    point.position.z = z;

                    point_array.push_back(point);
                    amount_per_layer++;
                }
            }
        }
        // Block 2
        if(overlap && vertical){
            for(int i = 0; i<m1; i++){
                if(i<m1 - mprim){
                    for(int j = 0; j<n2; j++){
                        point.orientation.z = 0.0;
                        point.orientation.w = 1.0;
                        point.position.x = (Px - Vx) - i*By;
                        point.position.y = Vy + j*Bx;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                } else {
                    for(int j = 0; j<n2 - nprim; j++){
                        point.orientation.z = 0.0;
                        point.orientation.w = 1.0;
                        point.position.x = (Px - Vx) - i*By;
                        point.position.y = Vy + j*Bx;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                }
            }
        } else {
            for(int i = 0; i<m1; i++){
                for(int j = 0; j<n2; j++){
                    point.orientation.z = 0.0;
                    point.orientation.w = 1.0;
                    point.position.x = (Px - Vx) - i*By;
                    point.position.y = Vy + j*Bx;
                    point.position.z = z;

                    point_array.push_back(point);
                    amount_per_layer++;
                }
            }
        }
        // Block 3
        if(overlap && !vertical){
            for(int i = 0; i<n1; i++){
                if(i<n1 - nprim){
                    for(int j = 0; j<m2; j++){
                        point.orientation.z = -0.7068252;
                        point.orientation.w = 0.7073883;
                        point.position.x = (Px - Hx) - i*Bx;
                        point.position.y = (Py - Hy) - j*By;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                } else {
                    for(int j = 0; j<m2 - mprim; j++){
                        point.orientation.z = -0.7068252;
                        point.orientation.w = 0.7073883;
                        point.position.x = (Px - Hx) - i*Bx;
                        point.position.y = (Py - Hy) - j*By;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                }
            }
        } else {
            for(int i = 0; i<n1; i++){
                for(int j = 0; j<m2; j++){
                    point.orientation.z = -0.7068252;
                    point.orientation.w = 0.7073883;
                    point.position.x = (Px - Hx) - i*Bx;
                    point.position.y = (Py - Hy) - j*By;
                    point.position.z = z;

                    point_array.push_back(point);
                    amount_per_layer++;
                }
            }
        }
        // Block 4
        if(overlap && vertical){
            for(int i = 0; i<m1; i++){
                if(i<m1 - mprim){
                    for(int j = 0; j<n2; j++){
                        point.orientation.z = -1.0;
                        point.orientation.w = 0.0;
                        point.position.x = Vx + i*By;
                        point.position.y = (Py - Vy) - j*Bx;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                } else {
                    for(int j = 0; j<n2 - nprim; j++){
                        point.orientation.z = -1.0;
                        point.orientation.w = 0.0;
                        point.position.x = Vx + i*By;
                        point.position.y = (Py - Vy) - j*Bx;
                        point.position.z = z;
    
                        point_array.push_back(point);
                        amount_per_layer++;
                    }
                }
            }
        } else {
            for(int i = 0; i<m1; i++){
                for(int j = 0; j<n2; j++){
                    point.orientation.z = -1.0;
                    point.orientation.w = 0.0;
                    point.position.x = Vx + i*By;
                    point.position.y = (Py - Vy) - j*Bx;
                    point.position.z = z;

                    point_array.push_back(point);
                    amount_per_layer++;
                }
            }
        }
    }

    void generateOptimizedPattern(bool mirror){
        convertPoints(point_array, mirror, {-0.6, -1.3, 0.0});
    }
    void generateColumnPattern(){
        convertPoints(point_array, false, {-0.6, -1.3, 0.0});
    }
    void generateBrickPattern(bool mirror){
        convertPoints(point_array, mirror, {-0.6, -1.25, 0.0});
    }
    void generateWellPattern(bool mirror){
        convertPoints(point_array, mirror, {-0.55, -1.25, 0.0});
    }
};

} // namespace pattern_generation

#endif // PATTERN_GENERATION_HPP