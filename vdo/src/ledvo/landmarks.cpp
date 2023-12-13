/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file landmarks.cpp
 * \date 08/12/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#include "include/ledvo_lib.h"

std::vector<gtsam::Point2> ledvo::LedvoLib::LED_extract_POI_alter(cv::Mat& frame)
{   
    std::vector<gtsam::Point2> pts_2d_detected;

    // convert to gray
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    // smooth
    cv::GaussianBlur(frame, frame, cv::Size(0,0), 1.0, 1.0, cv::BORDER_DEFAULT);

    // Blob method
    std::vector<cv::KeyPoint> keypoints_rgb_d;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 0.01;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(frame, keypoints_rgb_d);


	cv::drawKeypoints(frame, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    gtsam::Point2 temp;
    for(auto& what : keypoints_rgb_d)
    {
        temp.x() = what.pt.x;
        temp.y() = what.pt.y;

        pts_2d_detected.emplace_back(temp);
    }

    return pts_2d_detected;
}

std::vector<gtsam::Point3> ledvo::LedvoLib::pointcloud_generate(
    std::vector<gtsam::Point2>& pts_2d_detected, 
    cv::Mat depthimage
)
{
    std::vector<gtsam::Point2> pts_2d_detected_temp = pts_2d_detected;

    pts_2d_detected.clear();

    //get 9 pixels around the point of interest
    int no_pixels = 9;
    int POI_width = (sqrt(9) - 1 ) / 2;

    std::vector<gtsam::Point3> pointclouds;

    int x_pixel, y_pixel;
    gtsam::Point3 temp;

    depth_avg_of_all = 0;
    

    for(int i = 0; i < pts_2d_detected_temp.size(); i++)
    {

        x_pixel = pts_2d_detected[i].x();
        y_pixel = pts_2d_detected[i].y();
        
        cv::Point depthbox_vertice1 = cv::Point(x_pixel - POI_width, y_pixel - POI_width);
        cv::Point depthbox_vertice2 = cv::Point(x_pixel + POI_width, y_pixel + POI_width);
        cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);

        if(
            depthbox_vertice1.x < 1 || 
            depthbox_vertice1.y < 1 ||
            depthbox_vertice1.x > depthimage.cols ||
            depthbox_vertice1.y > depthimage.rows ||

            depthbox_vertice2.x < 1 ||
            depthbox_vertice2.y < 1 ||
            depthbox_vertice2.x > depthimage.cols ||
            depthbox_vertice2.y > depthimage.rows 
        )
        {
            continue;
        }
            

        cv::Mat ROI(depthimage, letsgetdepth);
        cv::Mat ROIframe;
        ROI.copyTo(ROIframe);
        std::vector<cv::Point> nonzeros;

        cv::findNonZero(ROIframe, nonzeros);
        std::vector<double> nonzerosvalue;
        for(auto temp : nonzeros)
        {
            double depth = ROIframe.at<ushort>(temp);
            nonzerosvalue.push_back(depth);
        }

        double depth_average = 0;
        if(nonzerosvalue.size() != 0)
            depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(),0.0)/nonzerosvalue.size();
        double z_depth = 0.001 * depth_average;

        z_depth = depthimage.at<ushort>(cv::Point(x_pixel, y_pixel)) * 0.001;
        

        if(z_depth == 0 || z_depth > 8.0)
            continue;

        temp.x() = x_pixel;
        temp.y() = y_pixel;
        temp.z() = 1;


        temp = z_depth * cameraMat.inverse() * temp;
        
        pts_2d_detected.emplace_back(pts_2d_detected_temp[i]);
        pointclouds.emplace_back(temp);
        
    }


    return pointclouds;
}

bool ledvo::LedvoLib::process_landmarks(
    std::vector<gtsam::Point2> pts_2d_detect,
    cv::Mat& depth,
    bool initialing
)
{
    std::vector<Eigen::Vector3d> pts_3d_detect = gen_pointcloud(pts_2d_detect, depth);

    if(firstFrame)
    {
        // no need correspondences search, just initialize
        std::cout<<"gan size here: "<<pts_2d_detect.size()<<std::endl;
        initial_no = pts_2d_detect.size();

        landmark lm_temp;
        for(int i = 0; i < pts_2d_detect.size(); i++)
        {
            lm_temp.tracked_id_no = i;
            
            lm_temp.pt3d = pts_3d_detect[i];
            lm_temp.pt2d = pts_2d_detect[i];
            lm_temp.pt2d_previous = pts_2d_detect[i];

            lm_temp.tracking = true;

            lm_dict.emplace_back(lm_temp);
        }

        firstFrame = false;
        initializer_counter++;

        raw_landmarks_pub(pts_3d_detect);// just for visualization purpose
        cout<<"end first frame"<<endl;
    }
    else
    {
        std::cout<<"size here: "<<pts_2d_detect.size()<<std::endl;
        std::cout<<"kaoakakakas"<<std::endl;
        vector<Eigen::Vector3d> pts_3d_detect_temp;
        // require correspondences search
        for(int i = 0; i < lm_dict.size(); i++)
        {
            double delta_max = INFINITY;
            Eigen::Vector3d pts_3d_temp;

            for(int j = 0; j < pts_2d_detect.size(); j++)
            {
                double delta_temp = (lm_dict[i].pt2d - pts_2d_detect[j]).norm();
                pts_3d_temp = lm_dict[i].pt3d;

                if(delta_temp < delta_max)
                {
                    lm_dict[i].pt2d = pts_2d_detect[j];
                    pts_3d_temp = landmark_ir_alpha * pts_3d_temp + (1 - landmark_ir_alpha) * pts_3d_detect[j];

                    delta_max = delta_temp;
                }

            }

            lm_dict[i].pt3d = pts_3d_temp;
            pts_3d_detect_temp.emplace_back(pts_3d_temp);

        }

        raw_landmarks_pub(pts_3d_detect_temp);// just for visualization purpose

        initializer_counter++;
    }

    if(initializer_counter >= 10)
        return true;
    else
        return false;
}

void ledvo::LedvoLib::raw_landmarks_pub(std::vector<gtsam::Point3> pts_3d_detect)
{
    traj_points.points.clear();
    cout<<"raw land pub"<<endl;

    geometry_msgs::Point posi_temp;

    for(int i = 0; i < pts_3d_detect.size(); i++)
    {
        // cout<<i<<endl;
        posi_temp.x = pts_3d_detect[i].x();
        posi_temp.y = pts_3d_detect[i].y();
        posi_temp.z = pts_3d_detect[i].z();
        
        traj_points.points.push_back(posi_temp);
    }

    posi_temp.x = pose_uav_inWorld_SE3.translation().x();
    posi_temp.y = pose_uav_inWorld_SE3.translation().y();
    posi_temp.z = pose_uav_inWorld_SE3.translation().z();

    traj_points.points.push_back(posi_temp);

    traj_points.header.stamp = ros::Time::now();
    lm_pub.publish(traj_points);
}

std::vector<Eigen::Vector3d> ledvo::LedvoLib::gen_pointcloud(
    std::vector<gtsam::Point2> pts_2d_detected, 
    cv::Mat& depthimage
)
{
    std::vector<gtsam::Point2> pts_2d_detected_temp = pts_2d_detected;

    std::vector<gtsam::Point3> pts_3d_detected;
    Eigen::Vector4d pts_3d_temp_homo;

    pts_2d_detected.clear();

    //get 9 pixels around the point of interest
    int no_pixels = 9;
    int POI_width = (sqrt(9) - 1 ) / 2;

    std::vector<gtsam::Point3> pointclouds;

    int x_pixel, y_pixel;
    gtsam::Point3 temp;

    depth_avg_of_all = 0;

    for(int i = 0; i < pts_2d_detected_temp.size(); i++)
    {

        x_pixel = pts_2d_detected[i].x();
        y_pixel = pts_2d_detected[i].y();
        
        cv::Point depthbox_vertice1 = cv::Point(x_pixel - POI_width, y_pixel - POI_width);
        cv::Point depthbox_vertice2 = cv::Point(x_pixel + POI_width, y_pixel + POI_width);
        cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);

        if(
            depthbox_vertice1.x < 1 || 
            depthbox_vertice1.y < 1 ||
            depthbox_vertice1.x > depthimage.cols ||
            depthbox_vertice1.y > depthimage.rows ||

            depthbox_vertice2.x < 1 ||
            depthbox_vertice2.y < 1 ||
            depthbox_vertice2.x > depthimage.cols ||
            depthbox_vertice2.y > depthimage.rows 
        )
        {
            continue;
        }
            

        cv::Mat ROI(depthimage, letsgetdepth);
        cv::Mat ROIframe;
        ROI.copyTo(ROIframe);
        std::vector<cv::Point> nonzeros;

        cv::findNonZero(ROIframe, nonzeros);
        std::vector<double> nonzerosvalue;
        for(auto temp : nonzeros)
        {
            double depth = ROIframe.at<ushort>(temp);
            nonzerosvalue.push_back(depth);
        }

        double depth_average = 0;
        if(nonzerosvalue.size() != 0)
            depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(),0.0)/nonzerosvalue.size();
        double z_depth = 0.001 * depth_average;

        z_depth = depthimage.at<ushort>(cv::Point(x_pixel, y_pixel)) * 0.001 - 0.14;
        

        if(z_depth == 0 || z_depth > 8.0)
            continue;

        ////////////
        Eigen::Vector3d pt_in_image_frame;
        pt_in_image_frame << 
            z_depth * x_pixel,
            z_depth * y_pixel,
            z_depth;

        Eigen::Vector4d homo_pt_temp;
        homo_pt_temp << cameraMat.inverse() * pt_in_image_frame, 1;
        ////// pass

        pts_3d_temp_homo = pose_cam_inWorld_SE3.matrix() * homo_pt_temp;

        pts_3d_detected.emplace_back(pts_3d_temp_homo.head(3));

    }

    return pts_3d_detected;
}


void ledvo::LedvoLib::set_correspondence_alter(
    std::vector<Eigen::Vector2d>& pts_2d_detected
)
{
    for(int i = 0; i < lm_dict.size(); i++)
    {



    }

}

void ledvo::LedvoLib::get_correspondence(
    std::vector<Eigen::Vector2d>& pts_2d_detected
)
{
    std::vector<Eigen::Vector2d> pts;
    std::vector<Eigen::Vector2d> pts_detected_previous_in_order;

    if(BA_error < 5.0)
    {
        Eigen::Vector2d reproject_temp;
        for(auto what : pts_on_body_frame)
        {
            reproject_temp = reproject_3D_2D(
                    what, 
                    pose_global_sophus
                );
            pts.emplace_back(reproject_temp);
            
        }

        if(
            pts_2d_detected.size() == LED_no 
            && 
            std::get<0>(corres_global_previous) == LED_no
        )
        {
            pts_detected_previous_in_order =  shift2D(
                pts,
                pts_2d_detected
            );
        }
        else
            pts_detected_previous_in_order = pts;
    }
    else
    {
        ROS_WARN("use previous detection 2D");

        for(auto what : std::get<1>(corres_global_previous))
            pts.emplace_back(what.pts_2d_correspond);

        if(
            pts_2d_detected.size() == LED_no 
            && 
            std::get<0>(corres_global_previous) == LED_no
        )
        {
            pts_detected_previous_in_order =  shift2D(
                pts,
                pts_2d_detected
            );
        }
        else
            pts_detected_previous_in_order = pts;

        // cv::imwrite("/home/patty/ledvo_ws/lala" + std::to_string(i) + ".jpg", frame_input);
        // pc::pattyDebug("check check");
        
    }

    correspondence_search_2D2DCompare(
        pts_2d_detected,
        pts_detected_previous_in_order
    );

    corres_global_previous = corres_global_current;
    
    pts_on_body_frame_in_corres_order.clear();
    pts_detected_in_corres_order.clear(); 

    for(int i = 0; i < std::get<1>(corres_global_current).size(); i++)
    {            
        if(std::get<1>(corres_global_current)[i].detected_ornot)
        {   
            // std::cout<<"=========="<<std::endl;
            // std::cout<<std::get<1>(corres_global_current)[i].pts_2d_correspond<<std::endl;
            // std::cout<<"=========="<<std::endl;
            pts_detected_in_corres_order.push_back(
                std::get<1>(corres_global_current)[i].pts_2d_correspond
            );             
            pts_on_body_frame_in_corres_order.push_back(pts_on_body_frame[i]);
            std::get<1>(corres_global_current)[i].detected_ornot = false; 
            // reset for next time step
        }        
    }

    
    
    // std::cout<<"i here, "<<i<<std::endl;
    i++;

        
}

std::vector<Eigen::Vector2d> ledvo::LedvoLib::shift2D(
    std::vector<Eigen::Vector2d>& pts_2D_previous,
    std::vector<Eigen::Vector2d>& pts_detect_current
)
{
    std::vector<Eigen::Vector2d> shifted_pts;
    int total_no = 0;

    // get average previous
    Eigen::Vector2d cg_previous;
    cg_previous.setZero();
    total_no = 0;
    for(auto& what : pts_2D_previous)
    {
        
        cg_previous += what;
        total_no++;
        
    }
    cg_previous /= total_no;

    // get average current
    Eigen::Vector2d cg_current;
    cg_current.setZero();
    total_no = 0;
    for(auto& what : pts_detect_current)
    {
        cv::circle(
            frame_input, 
            cv::Point(what(0), what(1)), 
            2.5, 
            CV_RGB(0,255,0),
            -1
        );
        cg_current += what;
        total_no++;
    }
    cg_current /= total_no;

    Eigen::Vector2d delta2D = cg_current - cg_previous;
    Eigen::Vector2d reproject_temp;

    // shift pts
    for(auto& what : pts_2D_previous)
    {
        reproject_temp = what + delta2D;
        shifted_pts.emplace_back(reproject_temp);
        cv::circle(
            frame_input, 
            cv::Point(reproject_temp(0), reproject_temp(1)), 
            1.0, 
            CV_RGB(0,0,255),
            -1
        );
    }
        
    return shifted_pts;
}


/* ================ POI Extraction utilities function below ================ */
std::vector<Eigen::Vector2d> ledvo::LedvoLib::LED_extract_POI(cv::Mat& frame, cv::Mat depth)
{   
    cv::Mat depth_mask_src = depth.clone(), depth_mask_dst1, depth_mask_dst2;

    cv::threshold(depth_mask_src, depth_mask_dst1, LANDING_DISTANCE * 1000, 50000, cv::THRESH_BINARY_INV);
    //filter out far depths

    cv::threshold(depth_mask_src, depth_mask_dst2, 0.5, 50000, cv::THRESH_BINARY); 
    //filter out zeros

    cv::bitwise_and(depth_mask_dst1, depth_mask_dst2, depth_mask_src);
    
    depth_mask_src.convertTo(depth_mask_src, CV_8U);
   
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::threshold(frame, frame, BINARY_THRES, 255, cv::THRESH_BINARY);
    frame_initial_thresholded = frame.clone();

    // detect frame after filter out background
    cv::bitwise_and(depth_mask_src, frame, frame); //filter out with depth information

    // Blob method
    std::vector<cv::KeyPoint> keypoints_rgb_d;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 0.01;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    detector->detect(frame, keypoints_rgb_d);
	// cv::drawKeypoints( frame, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    
    blobs_for_initialize = keypoints_rgb_d;

    min_blob_size = INFINITY;

    std::vector<Eigen::Vector2d> POI;
    for(auto what : keypoints_rgb_d)
    {
        min_blob_size =(what.size < min_blob_size ? what.size : min_blob_size);
        POI.push_back(Eigen::Vector2d(what.pt.x, what.pt.y));
    }

    return POI;
}





/* ================ Init. utilities function below ================ */





/* ================ k-means utilities function below ================ */

void ledvo::LedvoLib::correspondence_search_2D2DCompare(
    std::vector<Eigen::Vector2d>& pts_2d_detected,
    std::vector<Eigen::Vector2d>& pts_2d_detected_previous
)
{
    std::vector<cv::Point2f> pts;

    for(auto what : pts_2d_detected_previous)
        pts.emplace_back(cv::Point2f(what.x(), what.y()));

    // std::cout<<"in correspondence search kmeans"<<std::endl;
    // std::cout<<pts_2d_detected_previous.size()<<std::endl;
    
    //first emplace back pts_on_body_frame in 2D at this frame
    //in preset order

    for(auto what : pts_2d_detected)
        pts.emplace_back(cv::Point2f(what.x(), what.y()));
    
    std::vector<cv::Point2f> centers;
    cv::Mat labels;

    // std::cout<<pts_2d_detected.size()<<std::endl;
    // std::cout<<"+++++++++++++++"<<std::endl;

    cv::kmeans(pts, LED_no, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

    int error_count = 0;

    for(int i = 0; i < LED_no; i++)
    {
        int clusterIdx = labels.at<int>(i);
        for(int k = 0; k < pts.size(); k++)
        {
            if(i == k)
                continue;
            int clusterIdx_detected = labels.at<int>(k);
            if(clusterIdx == clusterIdx_detected)
            {
                if(k < LED_no)
                {
                    error_count++;
                    break;
                }  
                else
                {
                    std::get<1>(corres_global_current)[i].detected_ornot = true;
                    std::get<1>(corres_global_current)[i].pts_2d_correspond = pts_2d_detected[k - LED_no];
                }                                                    
            }
        }
    }    
}

double ledvo::LedvoLib::calculate_MAD(std::vector<double> norm_of_points)
{
    int n = norm_of_points.size();
    double mean = 0, delta_sum = 0, MAD;
    if(n != 0)
    {
        mean = accumulate(norm_of_points.begin(), norm_of_points.end(), 0.0) / n;
        for(int i = 0; i < n; i++)
            delta_sum = delta_sum + abs(norm_of_points[i] - mean);        
        MAD = delta_sum / n;
    }   

    return MAD;
}



// /* ================ Outlier Rejection utilities function below ================ */
//     /* in outlier rejection, we first calculate the MAD (mean average deviation)
//     to see whether there exists some outlier or not.
//     then, we try to do clustering with k-means algorithm.
//     as we are processing 3D points, at most time, 
//     the LED blobs should be close enough, 
//     while others being at some other coordinates that are pretty far away
//     hence, we set the clustering no. as 2.
//     we then calcullate the distance between the centroid of the cluster to the
//     center at previous time step(pcl_center_point_wo_outlier_previous)
//     and determine which cluster is the one that we want */

// void ledvo::LedvoLib::reject_outlier(std::vector<Eigen::Vector2d>& pts_2d_detect, cv::Mat depth)
// {
//     // std::vector<Eigen::Vector3d> pts_3d_detect = pointcloud_generate(pts_2d_detect, depth);
//     //what is this for?
//     //to get 3d coordinates in body frame, so that 
//     //outlier rejection could be performed
//     int n = pts_3d_detect.size();

//     std::vector<cv::Point3f> pts;
//     std::vector<double> norm_of_x_points;
//     std::vector<double> norm_of_y_points;
//     std::vector<double> norm_of_z_points;

//     for(auto what :  pts_3d_detect)
//     {
//         norm_of_x_points.push_back(what.x());
//         norm_of_y_points.push_back(what.y());
//         norm_of_z_points.push_back(what.z());

//         pts.push_back(cv::Point3f(what.x(), what.y(), what.z()));
//     }

//     cv::Mat labels;
//     std::vector<cv::Point3f> centers;

    
//     if(calculate_MAD(norm_of_x_points) > MAD_x_threshold  
//         || calculate_MAD(norm_of_y_points) > MAD_y_threshold
//         || calculate_MAD(norm_of_z_points) > MAD_z_threshold)
//     {   
//         ROS_WARN("GOT SOME REJECTION TO DO!");
//         // cout<<"got some rejection to do"<<endl;
//         cv::kmeans(pts, 2, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

//         double d0 = cv::norm(pcl_center_point_wo_outlier_previous - centers[0]);
//         double d1 = cv::norm(pcl_center_point_wo_outlier_previous - centers[1]);

//         std::vector<Eigen::Vector2d> pts_2d_result;
//         std::vector<Eigen::Vector3d> pts_3d_result;

//         if(d0 < d1) //then get index with 0
//         {           
//             for(int i = 0; i < labels.rows; i++)
//             {
//                 if(labels.at<int>(0,i) == 0)
//                 {
//                     pts_2d_result.push_back(pts_2d_detect[i]);
//                     pts_3d_result.push_back(pts_3d_detect[i]); 
//                 }                    
//             }            
//             pcl_center_point_wo_outlier_previous = centers[0];
//         }
//         else
//         {
//             for(int i = 0; i < labels.rows; i++)
//             {

//                 if(labels.at<int>(0,i) == 1)
//                 {                    
//                     pts_2d_result.push_back(pts_2d_detect[i]);
//                     pts_3d_result.push_back(pts_3d_detect[i]);                    
//                 }
//             }
//             pcl_center_point_wo_outlier_previous = centers[1];
//         }
            
//         pts_2d_detect.clear();
//         pts_2d_detect = pts_2d_result;

//         pts_3d_detect.clear();
//         pts_3d_detect = pts_3d_result;

//     }
//     else
//     {
//         cv::Mat temp;
        
//         cv::reduce(pts, temp, 01, CV_REDUCE_AVG);
//         pcl_center_point_wo_outlier_previous = cv::Point3f(temp.at<float>(0,0), temp.at<float>(0,1), temp.at<float>(0,2));

//     }

//     led_3d_posi_in_camera_frame_depth = Eigen::Vector3d(
//         pcl_center_point_wo_outlier_previous.x,
//         pcl_center_point_wo_outlier_previous.y,
//         pcl_center_point_wo_outlier_previous.z
//     );
// }
