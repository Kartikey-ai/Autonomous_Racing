#!/usr/bin/env python3

#Noise Removal  X
#?Downsampling  X
#ROI
#Ransac
#Euclidian

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import  sensor_msgs.point_cloud2 as pc2
import pcl_helper
from vehicle_msgs.msg import TrackCone, Track

#Noise
def do_statistical_outlier_filtering(cloud , mean_k = 10 , thresh = 5.0):
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(thresh)
    return outlier_filter.filter()

#Downsampling
def do_voxel_grid_downsampling(pcl_data, leaf_size = 0.1): 
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    return vox.filter()

#ROI
def do_passthrough(pcl_data, filter_axis, axis_min, axis_max):
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

#Ransac
def do_ransac_plane_normal_segmentation(point_cloud, input_max_distance):
    segmenter = point_cloud.make_segmenter()

    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_distance_threshold(input_max_distance)
   
    # segmenter = point_cloud.make_segmenter_normals(ksearch = 50)
    
    # segmenter.set_optimize_coefficients(True)
    
    # segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    
    # segmenter.set_normal_distance_weight(0.1)
    
    # segmenter.set_method_type(pcl.SAC_RANSAC)
    
    # segmenter.set_max_iterations(2000)
    
    # segmenter.set_distance_threshold(input_max_distance)
    
    indices, coefficients = segmenter.segment()

    inliners = point_cloud.extract(indices, negative=False)
    outliers = point_cloud.extract(indices, negative=True)

    return indices, inliners, outliers

#Euclidian
def do_euclidian_clustering(cloud):
    # Euclidean Clustering
    white_cloud = pcl_helper.XYZRGB_to_XYZ(cloud) 
    tree = white_cloud.make_kdtree() 
    ec = white_cloud.make_EuclideanClusterExtraction()

    ec.set_ClusterTolerance(0.3) #0.14, 0.08
    ec.set_MinClusterSize(1)
    ec.set_MaxClusterSize(3500)

    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract() 

    cluster_color = pcl_helper.random_color_gen()
    #cluster_color = pcl_helper.get_color_list(len(cluster_indices))
    
    #print("No. of cones visible at this moment ", len(cluster_indices))
    
    #print(cluster_indices)
    
    cluster_coords = Track()
    #cone = TrackCone()
    #cluster_coords.cones = []
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        x, y, z = 0, 0, 0
        cone_clusters = []
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], pcl_helper.rgb_to_float(cluster_color)])
            x = x + white_cloud[indice][0]
            y = y + white_cloud[indice][1]
            z = z + white_cloud[indice][2]

        cone_clusters.append(x/len(indices))
        cone_clusters.append(y/len(indices))
        #print(cone_clusters)
        #print(color_cluster_point_list) 
        #cone.x = x/len(indices)     
        #cone.y = y/len(indices)
        #cone.type = f'cone{j+1}'
        

        #cluster_coords.cones = cone_clusters
        #cluster_coords.data.append(TrackCone(data = cone_clusters))
        cluster_coords.data.append(TrackCone(x = x/len(indices),     
                                             y = y/len(indices)))
        #cluster_coords.cones[j].x = x/len(indices)
        #cluster_coords.cones[j].y = y/len(indices)
        #cluster_coords.cones[j].type = 'cone{k}' 

    #print(len(cluster_coords.cones))
    #print(len(cluster_coords.cones[0])) 
    
    #print("No. of cones visible at this moment ", len(color_cluster_point_list))
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    ros_cluster_cloud = pcl_helper.pcl_to_ros(cluster_cloud)

    return cluster_cloud, cluster_coords


    # tolerance = 0.05
    # min_size = 20
    # max_size = 1500

    # tree = cloud.make_kdtree()
    # extraction_object = cloud.make_EuclideanClusterExtraction()

    # extraction_object.set_ClusterTolerance(tolerance)
    # extraction_object.set_MinClusterSize(min_size)
    # extraction_object.set_MaxClusterSize(max_size)
    # extraction_object.set_SearchMethod(tree)

    # number_of_clusters = len(clusters)
    # colors = random_color_gen()

    # colored_points = []

    # for cluster_id, cluster in enumerate(clusters):
    #     for c, i in enumerate(cluster):
    #         x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
    #         color = rgb_to_float(colors)
    #         colored_points.append([x, y, z, color])
    
    # return colored_points

    



def callback(input_ros_msg):
    cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    #cloud = pcl_helper.XYZRGB_to_XYZ(cloud)
    
    #cloud_denoised = do_statistical_outlier_filtering(cloud)

    #cloud_downsampled = do_voxel_grid_downsampling(cloud)

    cloud_roi_x = do_passthrough(cloud, 'x', 0.0, 30.0)

    cloud_roi_y = do_passthrough(cloud_roi_x, 'y', -5.0, 5.0)

    _, _, cloud_ransaced = do_ransac_plane_normal_segmentation(cloud_roi_y, 0.007)

    cloud_clustered, cluster_coords = do_euclidian_clustering(cloud_ransaced)
    pub2.publish(cluster_coords)

    cloud_new = pcl_helper.pcl_to_ros(cloud_clustered)
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node("final" , anonymous= True)
    rospy.Subscriber("/velodyne_points" , PointCloud2 , callback)
    pub = rospy.Publisher("/velodyne_final" , PointCloud2 , queue_size=1)
    pub2 = rospy.Publisher("/track" , Track , queue_size=1)
    rospy.spin()