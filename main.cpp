#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
 

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
main(int argc, char** argv)
{
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

 for (int i=0;i<200;i++)
  {
      pcl::PointXYZRGB point;
      point.x = i;
      point.y = i;
      point.z = i;
      uint32_t rgb = (static_cast<uint32_t>(i%255) << 16 |
              static_cast<uint32_t>((i+100)%255) << 8 | static_cast<uint32_t>((i+200)%255));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
  }
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = rgbVis(point_cloud_ptr);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
