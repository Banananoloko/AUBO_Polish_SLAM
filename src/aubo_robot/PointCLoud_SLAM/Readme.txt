depth camera
  -> /camera/color/image_raw
  -> /camera/aligned_depth_to_color/image_raw
  -> /camera/depth/color/points

rtabmap_ros
  -> 订阅相机图像/深度/TF
  -> 输出 /rtabmap/cloud_map 或局部点云

PointCLoud_SLAM/tabletop_segmentation_node
  -> 订阅 /rtabmap/cloud_map 或 /camera/depth/color/points
  -> 下采样 / ROI / RANSAC 桌面分割 / 聚类
  -> 发布 /table_plane, /objects_cloud, /clusters

PointCLoud_SLAM/poisson_reconstruction_node
  -> 订阅 /clusters
  -> 法向估计 + Poisson
  -> 发布 /object_mesh_marker 或保存 .ply/.stl
