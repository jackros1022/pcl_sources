/*
- 欧式聚类分割 SACSegmentation 分割
- 07_conditional_euclidean_clustering 欧式聚类
*/
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>


int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("/home/jack/learning_projects/pcl/pcl_sources/1_segmentation/03_cluster_extraction/table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  /* Create the filtering object: downsample the dataset using a leaf size of 1cm
   * 体素化[欧式距离提取必须要体素化]
  */
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  /* Create the segmentation object for the planar model and set all the parameters
   * 采样一致性分割　特点：只能分割出一个目标
  */
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);  
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  //★　和自身比较　0.3倍，这样合理吗？
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    /* Segment the largest planar component from the remaining cloud
     * 分割出最大的平面部分
    */ 
    seg.setInputCloud (cloud_filtered);
     //求内点集和系数（模型估计）
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);   //迭代某一次求好的内点集
    extract.setNegative (false);    

    /* Get the points associated with the planar surface
     * 迭代某一次提取最大平面的内点集
    */
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    //Remove the planar inliers, extract the rest
    //迭代某一次平面以外的点集
    extract.setNegative (true);
    extract.filter (*cloud_f);
    /*
     * ★　迭代某一次平面以外的内点集cloud_f赋值给cloud_filtered
     *  迭代某一次cloud_filtered不一定包含目标物体　＝＝》估计求的目标点集是原点集的三分之一，直到条件成立为止。
    */
    *cloud_filtered = *cloud_f;

  }

  /* Creating the KdTree object for the search method of the extraction
   * 为最大平面以外的点集　创建kdtree结构
   *  定义searchTree结构
   */
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  /* ★　树里放了点云（估计求的目标点集）
   * 1.使用欧式聚类提取要遍历点云，建立搜索结构。
  */
  tree->setInputCloud(cloud_filtered);
//  tree->setInputCloud (cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  //★　cluster_indices是向量容器，一组数据
  ec.extract (cluster_indices);

  //遍历容器数据　ｊ代表容器单位个数
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //容器的单位数据，每个单位的数据点提取出来。也就是说，把每个单位聚类提取出来
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

    //★★　思考orgnaized　cloudpoint的特性？？？
    cloud_cluster->width = cloud_cluster->points.size ();   
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(cloud_cluster);
    while (!viewer.wasStopped ())
    {
    }

    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
