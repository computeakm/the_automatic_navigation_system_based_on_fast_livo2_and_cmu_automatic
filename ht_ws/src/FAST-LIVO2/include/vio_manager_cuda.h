#ifndef VIO_MANAGER_CUDA_H
#define VIO_MANAGER_CUDA_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <omp.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>



using namespace std;
using namespace Eigen;

struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points;
  vector<double> inv_expo_list;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};


class Warp
{
public:
  Matrix2d A_cur_ref;
  int search_level;
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};



class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};


class VIOManager {
public:
    
    VIOManager();
    ~VIOManager();

    // 初始化，分配零拷贝内存
    void initializeVIO();

    // 主处理函数
    void processFrame(cv::Mat &img, vector<pointWithVar> &pg, 
                     const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, 
                     double img_time);

private:
    // GPU 资源
    cv::cuda::Stream stream_;
    cv::cuda::GpuMat d_img_;          // 当前帧 GPU 图像
    cv::cuda::GpuMat d_img_gray_;     // 当前帧 GPU 灰度图
    cv::cuda::GpuMat d_img_resized_;  // 当前帧 GPU 缩放图
    
    // 零拷贝主机内存 (Jetson 优化)
    // 用于存储从摄像头获取的原始数据或中间结果，CPU/GPU 共享
    cv::cuda::HostMem h_mem_img_;     
    cv::Mat h_img_;                   // 指向 h_mem_img_ 的 Mat 头

    // 其他原有成员变量保持不变
    // ...
    StatesGroup *state;
    std::shared_ptr<Frame> new_frame_;
    vk::AbstractCamera *cam;
    vk::PinholeCamera *pinhole_cam;
  StatesGroup *state;
  StatesGroup *state_propagat;
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  V3D Pli, Pci, Pcl, Pcw;
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false, colmap_output_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<V3D>> rays_with_sample_points;

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  Eigen::MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  unordered_map<int, Warp *> warp_map;
  vector<VisualPoint *> retrieve_voxel_points;
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

   
    // 重写的 GPU 函数
    void getImagePatchGPU(const cv::cuda::GpuMat& d_img, V2D pc, float *patch_tmp, int level);
    
    // 其他辅助函数保持不变，但内部可能需要适配 GpuMat
    void retrieveFromVisualSparseMap(const cv::cuda::GpuMat& d_img, vector<pointWithVar> &pg, 
                                     const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
    void computeJacobianAndUpdateEKF(const cv::cuda::GpuMat& d_img);
    // ...
};

#endif // VIO_MANAGER_CUDA_H
