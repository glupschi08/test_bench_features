/**
 * time consumption comparsion analysis for 3D features in Point Cloud Library
 * use uniform sampling as keypoint detector
 * adapted version based on work from @author Kanzhi Wu by @author Florian Auinger
 */


#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/rsd.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/shot.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/rift.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/point_types_conversion.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/uniform_sampling.h>


#include <iostream>
#include <string>

//addional libs
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/keypoints/harris_3d.h>
#include <stdlib.h>
#include <pcl/console/time.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/usc.h>

//own_libs
//#include "overlap.h"
//#include "keypoints.h"

//for rejection
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/keypoints/iss_3d.h>
#define RANSAC_Inlier_Threshold 3.//1.5 //0.1
#define RANSAC_Iterations 5000


/// normal vector extraction
///
/// cloud       -- input point cloud
/// kpts        -- input keypoints
///
pcl::PointCloud<pcl::Normal>::Ptr normal_extraction( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts, float normal_radius) {
  pcl::console::TicToc tt;
  tt.tic();
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud( kpts );
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
  ne.setSearchMethod( tree );
  //ne.setSearchSurface( cloud );
  pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal>() );
  ne.setRadiusSearch( normal_radius); //0.1
  ne.compute( *normals );
  double t = tt.toc();
  pcl::console::print_value( "Normal extraction takes %.3f\n", t );
  return normals;
}

/// normal vector extraction in omp
///
/// cloud       -- input point cloud
/// kpts        -- input keypoints
///
pcl::PointCloud<pcl::Normal>::Ptr normal_extraction_omp( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts, float normal_radius) {
  pcl::console::TicToc tt;
  tt.tic();
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud( kpts );
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
  ne.setSearchMethod( tree );
  ne.setSearchSurface( cloud );
  pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal>() );
  ne.setRadiusSearch( normal_radius );//0.1
  ne.compute( *normals );
  double t = tt.toc();
  pcl::console::print_value( "Normal extraction in OMP takes %.3f\n", t );
  return normals;
}

/// normal vector extraction using integral image
///
/// cloud       -- input point cloud
///
pcl::PointCloud<pcl::Normal>::Ptr normal_extraction_integral_image(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float MaxDepthChangeFactor) {
  pcl::console::TicToc tt;
  tt.tic();
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(MaxDepthChangeFactor);//0.03f
//  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud);
  pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal>() );
  ne.compute( *normals );
  double t = tt.toc();
  pcl::console::print_value( "Normal extraction using integral image(%d points) takes %.3f\n", (int)normals->size(), t );
  return normals;
}

/// Persistent Feature Histogram
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
///
pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_extraction(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals ,
    float feature_radius){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr kpts( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::copyPointCloud( *rgb_cloud, *cloud );
  pcl::copyPointCloud( *rgb_kpts, *kpts );
  pcl::console::TicToc tt;
  tt.tic();
  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_extraction;
  pfh_extraction.setSearchSurface( cloud );
  pfh_extraction.setInputCloud( kpts );
  pfh_extraction.setInputNormals( normals );
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pfh_extraction.setSearchMethod( tree );
  pfh_extraction.setRadiusSearch( feature_radius); //0.05;
  pcl::PointCloud<pcl::PFHSignature125>::Ptr descrs( new pcl::PointCloud<pcl::PFHSignature125>() );
  pfh_extraction.compute( *descrs );
  double t = tt.toc();
  pcl::console::print_value( "Persistent Feature Histogram takes %.3f\n", t );
  return descrs;
}



void compute_PFHRGB_features(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud,
                             pcl::PointCloud <pcl::Normal>::Ptr &normals,
                             pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
                             pcl::PointCloud <pcl::PFHRGBSignature250>::Ptr &descriptors_out,
                             double feature_radius) {

    // copy only XYZ data of keypoints for use in estimating features
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfhrgbEstimation;

    pfhrgbEstimation.setInputCloud(keypoints_xyzrgb);
    pfhrgbEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure
    pfhrgbEstimation.setInputNormals(normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pfhrgbEstimation.setSearchMethod(tree);
    //pfhrgbEstimation.setKSearch(100);

    // Use all neighbors in a sphere of radius radius
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfhrgbEstimation.setRadiusSearch(feature_radius);

    // Compute the features
    pfhrgbEstimation.compute(*descriptors_out);

}



/// Persistent Feature Histogram
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhrgb_extraction(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch){
  pcl::console::TicToc tt;
  tt.tic();
  pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfh_extraction;
  pfh_extraction.setSearchSurface( cloud );
  pfh_extraction.setInputCloud( kpts );
  pfh_extraction.setInputNormals( normals );
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pfh_extraction.setSearchMethod( tree );
  pfh_extraction.setRadiusSearch( setRadiusSearch );//0.05
  pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descrs( new pcl::PointCloud<pcl::PFHRGBSignature250>() );
  pfh_extraction.compute( *descrs );
  double t = tt.toc();
  pcl::console::print_value( "Persistent Feature Histogram RGB takes %.3f\n", t );
  return descrs;
}

/// Fast Persistent Feature Histogram
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_extraction(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch) {
  pcl::console::TicToc tt;
  tt.tic();
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33 > fpfh_extraction;
  fpfh_extraction.setSearchSurface( cloud );
  fpfh_extraction.setInputCloud( kpts );
  fpfh_extraction.setInputNormals( normals );
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  fpfh_extraction.setSearchMethod( tree );
  fpfh_extraction.setRadiusSearch( setRadiusSearch );//0.05
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr descrs( new pcl::PointCloud<pcl::FPFHSignature33>() );
  fpfh_extraction.compute( *descrs );
  double t = tt.toc();
  pcl::console::print_value( "Fast Persistent Feature Histogram takes %.3f\n", t );
  return descrs;
}


/// Fast Persistent Feature Histogram in OMP
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_extraction_omp(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch) {
  pcl::console::TicToc tt;
  tt.tic();
  pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33 > fpfh_extraction;
  fpfh_extraction.setSearchSurface( cloud );
  fpfh_extraction.setInputCloud( kpts );
  fpfh_extraction.setInputNormals( normals );
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  fpfh_extraction.setSearchMethod( tree );
  fpfh_extraction.setRadiusSearch( setRadiusSearch );//0.05
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr descrs( new pcl::PointCloud<pcl::FPFHSignature33>() );
  fpfh_extraction.compute( *descrs );
  double t = tt.toc();
  pcl::console::print_value( "Fast Persistent Feature Histogram in OMP takes %.3f\n", t );
  return descrs;
}


/// Signature of Hitograms of OrientTation, XYZ
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::SHOT352>::Ptr shot_extraction(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch_SHOT) {
  // convert point cloud type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr kpts( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::copyPointCloud( *rgb_cloud, *cloud );
  pcl::copyPointCloud( *rgb_kpts, *kpts );

  pcl::console::TicToc tt;
  tt.tic();

  pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot_extraction;
  shot_extraction.setInputCloud( kpts );
  shot_extraction.setSearchSurface( cloud );
  shot_extraction.setInputNormals( normals );
  shot_extraction.setRadiusSearch( setRadiusSearch_SHOT );//0.05

  pcl::PointCloud<pcl::SHOT352>::Ptr descrs( new pcl::PointCloud<pcl::SHOT352>() );
  shot_extraction.compute( *descrs );

  double t = tt.toc();
  pcl::console::print_value( "Signature of Hitograms of OrientTation takes %.3f\n", t );

  return descrs;
}


/// Signature of Hitograms of OrientTation, XYZ in OMP
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::SHOT352>::Ptr shot_extraction_omp(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch_SHOT) {
  // convert point cloud type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr kpts( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::copyPointCloud( *rgb_cloud, *cloud );
  pcl::copyPointCloud( *rgb_kpts, *kpts );

  pcl::console::TicToc tt;
  tt.tic();

  pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot_extraction;
  shot_extraction.setInputCloud( kpts );
  shot_extraction.setSearchSurface( cloud );
  shot_extraction.setInputNormals( normals );
  shot_extraction.setRadiusSearch(setRadiusSearch_SHOT );//0.05

  pcl::PointCloud<pcl::SHOT352>::Ptr descrs( new pcl::PointCloud<pcl::SHOT352>() );
  shot_extraction.compute( *descrs );

  double t = tt.toc();
  pcl::console::print_value( "Signature of Hitograms of OrientTation in OMP takes %.3f\n", t );

  return descrs;
}


/// Signature of Hitograms of OrientTation, XYZRGB
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::SHOT1344>::Ptr cshot_extraction(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch_SHOT) {
  pcl::console::TicToc tt;
  tt.tic();

  pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot_extraction;
  shot_extraction.setInputCloud( kpts );
  shot_extraction.setSearchSurface( cloud );
  shot_extraction.setInputNormals( normals );
  shot_extraction.setRadiusSearch( setRadiusSearch_SHOT );  //0.05

  pcl::PointCloud<pcl::SHOT1344>::Ptr descrs( new pcl::PointCloud<pcl::SHOT1344>() );
  shot_extraction.compute( *descrs );

  double t = tt.toc();
  pcl::console::print_value( "Color Signature of Hitograms of OrientTation takes %.3f\n", t );

  return descrs;
}


/// Signature of Hitograms of OrientTation, XYZRGB in OMP
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::SHOT1344>::Ptr cshot_extraction_omp(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch_SHOT) {
  pcl::console::TicToc tt;
  tt.tic();

  pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot_extraction;
  shot_extraction.setInputCloud( kpts );
  shot_extraction.setSearchSurface( cloud );
  shot_extraction.setInputNormals( normals );
  shot_extraction.setRadiusSearch( setRadiusSearch_SHOT );  //0.05

  pcl::PointCloud<pcl::SHOT1344>::Ptr descrs( new pcl::PointCloud<pcl::SHOT1344>() );
  shot_extraction.compute( *descrs );

  double t = tt.toc();
  pcl::console::print_value( "Color Signature of Hitograms of OrientTation in OMP takes %.3f\n", t );

  return descrs;
}


/// Radius-Based Surface Descriptor
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsd_extraction(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch_RSD,
    double setPlaneRadius_RSD) {
  pcl::console::TicToc tt;
  tt.tic();

  pcl::RSDEstimation< pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalRadiiRSD > rsd_extraction;
  rsd_extraction.setInputCloud( kpts );
  rsd_extraction.setSearchSurface(cloud);
  rsd_extraction.setInputNormals( normals );

  rsd_extraction.setRadiusSearch( setRadiusSearch_RSD );//0.05
  rsd_extraction.setPlaneRadius( setPlaneRadius_RSD );//0.01
  rsd_extraction.setSaveHistograms( false );

  pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descrs( new pcl::PointCloud<pcl::PrincipalRadiiRSD>() );
  rsd_extraction.compute( *descrs );
  double t = tt.toc();
  pcl::console::print_value( "Radius-Based Surface Descriptor takes %.3f\n", t );

  return descrs;
}

/// 3D Shape Context
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::ShapeContext1980>::Ptr sc_extraction(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
    pcl::PointCloud<pcl::Normal>::Ptr normals,
    double setRadiusSearch_Shape,
    double setMinimalRadius_div,
    double setPointDensityRadius_div) {

  pcl::console::TicToc tt;
  tt.tic();
  pcl::ShapeContext3DEstimation< pcl::PointXYZRGB, pcl::Normal, pcl::ShapeContext1980 > sc_extraction;
  sc_extraction.setInputCloud( kpts );//kpts
  sc_extraction.setSearchSurface(cloud);//cloud
  sc_extraction.setInputNormals( normals );//normals

  sc_extraction.setRadiusSearch( setRadiusSearch_Shape );//0.05
  sc_extraction.setMinimalRadius(setRadiusSearch_Shape / setMinimalRadius_div);//0.05/10.0
  sc_extraction.setPointDensityRadius(setRadiusSearch_Shape / setPointDensityRadius_div);//0.05 / 5.0

  pcl::PointCloud<pcl::ShapeContext1980>::Ptr descrs( new pcl::PointCloud<pcl::ShapeContext1980>() );
  sc_extraction.compute( *descrs );
  double t = tt.toc();
  pcl::console::print_value( "3D Shape Context takes %.3f\n", t );

  return descrs;

}

/// Unique Shape Context descriptor
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr usc_extraction(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        double RadiusSearch,
        double setMinimalRadius_div,
        double setPointDensityRadius_div,
        double LocalRadiusSearch) {

    pcl::console::TicToc tt;
    tt.tic();

    // USC estimation object.
    pcl::UniqueShapeContext<pcl::PointXYZRGB, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc;
    //usc.setInputCloud(kpts);
    usc.setInputCloud(cloud);
    // Search radius, to look for neighbors. It will also be the radius of the support sphere.
    usc.setRadiusSearch(RadiusSearch);//0.05
    // The minimal radius value for the search sphere, to avoid being too sensitive
    // in bins close to the center of the sphere.
    usc.setMinimalRadius(RadiusSearch / setMinimalRadius_div);//0.05 / 10.0
    // Radius used to compute the local point density for the neighbors
    // (the density is the number of points within that radius).
    usc.setPointDensityRadius(RadiusSearch / setPointDensityRadius_div);//0.05 / 5.0
    // Set the radius to compute the Local Reference Frame.
    usc.setLocalRadius(LocalRadiusSearch);//0.05
    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr descrs( new pcl::PointCloud<pcl::UniqueShapeContext1960>() );
    usc.compute(*descrs);

    double t = tt.toc();
    pcl::console::print_value( "3D Shape Context takes %.3f\n", t );
    return descrs;
}


/// Rotation-Invariant Feature Transform
///
/// cloud         -- input point cloud
/// kpts          -- keypoints
/// normals       -- normals
///
typedef pcl::Histogram<32> RIFT32;
pcl::PointCloud<RIFT32 >::Ptr rift_extraction( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts,
        pcl::PointCloud<pcl::Normal>::Ptr normals,
        double GradientEstimationsetRadiusSearch,
        double RadiusSearch,
        double NrDistanceBins,
        double NrGradientBins) {

  pcl::console::TicToc tt;
  tt.tic();

  //pcl::console::print_value( "test\n" );
  // Convert the RGB to intensity.
  pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloudXYZRGBtoXYZI(*cloud, *intensity_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_kpts(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloudXYZRGBtoXYZI(*kpts, *intensity_kpts);

  //pcl::console::print_value( "test\n");
  // Compute the intensity gradients.
  pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
  pcl::IntensityGradientEstimation< pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient,
      pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;

  ge.setInputCloud( intensity_kpts );
  //ge.setInputCloud( intensity_kpts );
  ge.setSearchSurface( intensity_cloud );
  //ge.setSearchSurface( intensity_kpts );
  ge.setInputNormals(normals);
  ge.setRadiusSearch(GradientEstimationsetRadiusSearch);//1000.00001//0.05
  ge.compute(*gradients);
  pcl::console::print_value( "gradients = %d, keypoints = %d\n", (int)gradients->size(), (int)intensity_kpts->size() );

  //pcl::console::print_value( "test\n");
  pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT32 > rift_extraction;
  rift_extraction.setInputCloud(intensity_kpts);
//  rift_extraction.setSearchSurface( intensity_cloud );
  rift_extraction.setInputGradient(gradients);
  rift_extraction.setRadiusSearch(RadiusSearch);//100.05 //0.5
  rift_extraction.setNrDistanceBins(NrDistanceBins);//4
  rift_extraction.setNrGradientBins(NrGradientBins);//8


  pcl::PointCloud<RIFT32 >::Ptr descrs( new pcl::PointCloud<RIFT32 >() );
  rift_extraction.compute( *descrs );
  double t = tt.toc();
  pcl::console::print_value( "Rotation-Invariant Feature Transform takes %.3f and %d features. \n", t,  descrs->size());

  return descrs;

}


void downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out) {
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
    vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud (points);
    vox_grid.filter (*downsampled_out);

    std::cout << "Num of points in :" << points->size() << std::endl;
    std::cout << "Num of points out :" << downsampled_out->size() << std::endl;
}


void find_feature_correspondences_shapeContext (pcl::PointCloud<pcl::ShapeContext1980>::Ptr &source_descriptors,
                                       pcl::PointCloud<pcl::ShapeContext1980>::Ptr &target_descriptors,
                                       std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    std::cout << "shapeContext source_descriptors size:" << source_descriptors->size () << std::endl;
    correspondences_out.resize (source_descriptors->size ());
    correspondence_scores_out.resize (source_descriptors->size ());
    std::cout << "shapeContext correspondences_out size:" << correspondences_out.size () << std::endl;
    std::cout << "shapeContext correspondence_scores_out size:" << correspondence_scores_out.size () << std::endl;

    // Use a KdTree to search for the nearest matches in feature space
    //pcl::PointCloud<pcl::ShapeContext1980>::Ptr descrs_ShapeContext1980_1( new pcl::PointCloud<pcl::ShapeContext1980>() );
    //pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
    pcl::search::KdTree<pcl::ShapeContext1980> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);

    for (size_t i = 0; i < source_descriptors->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}


void rejectBadCorrespondences(const pcl::CorrespondencesPtr &all_correspondences,
                              const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_src,
                              const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_tgt,
                              pcl::Correspondences &remaining_correspondences)
{
    // copy only XYZRGB data of keypoints for use in estimating features
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_src_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_tgt_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*keypoints_src, *keypoints_src_xyzrgb);
    pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_xyzrgb);


    // RandomSampleConsensus bad correspondence rejector
    pcl::registration::CorrespondenceRejectorSampleConsensus <pcl::PointXYZRGB> correspondence_rejector;
    correspondence_rejector.setInputSource (keypoints_src_xyzrgb);
    correspondence_rejector.setInputTarget (keypoints_tgt_xyzrgb);
    correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
    correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
    correspondence_rejector.setRefineModel(true);//false
    correspondence_rejector.setInputCorrespondences(all_correspondences);
    correspondence_rejector.getCorrespondences(remaining_correspondences);
}

void findCorrespondences_FPFHSignature33 (pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                                       pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_PFH_newshort (pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_src,
                                       pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_tgt,
                                       pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}
void findCorrespondences_PFHRGB(const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_src,
                                const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_tgt,
                                pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}
void findCorrespondences_SHOT352(const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_src,
                                const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_tgt,
                                pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}
void findCorrespondences_SHOT1344(const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_src,
                                 const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_tgt,
                                 pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}


/*  //didn*t work
void findCorrespondences_RIFT32(const pcl::PointCloud<pcl::Histogram<32>>::Ptr &fpfhs_src,
                                const pcl::PointCloud<pcl::Histogram<32>>::Ptr &fpfhs_tgt,
                                  pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::Histogram<32>, pcl::Histogram<32>> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}
*/

void findCorrespondences_ShapeContext1980(const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_src,
                                  const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_tgt,
                                  pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::ShapeContext1980, pcl::ShapeContext1980> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}


void findCorrespondences_PrincipalRadiiRSD(const pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr &fpfhs_src,
                                  const pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr &fpfhs_tgt,
                                  pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::PrincipalRadiiRSD, pcl::PrincipalRadiiRSD> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}


void findCorrespondences_UniqueShapeContext1960(const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_src,
                                          const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_tgt,
                                          pcl::Correspondences &all_correspondences) {
    pcl::registration::CorrespondenceEstimation<pcl::UniqueShapeContext1960, pcl::UniqueShapeContext1960> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}


void find_feature_correspondences_PFH (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    std::cout << "PFH source_descriptors size:" << source_descriptors->size () << std::endl;
    correspondences_out.resize (source_descriptors->size ());
    correspondence_scores_out.resize (source_descriptors->size ());
    std::cout << "shapeContext correspondences_out size:" << correspondences_out.size () << std::endl;
    std::cout << "shapeContext correspondence_scores_out size:" << correspondence_scores_out.size () << std::endl;


    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);

    for (size_t i = 0; i < source_descriptors->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}


pcl::PointCloud<pcl::PointWithScale>::Ptr iss3d_PointWithScale( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointWithScale> iss_detector;
    pcl::search::KdTree<pcl::PointWithScale>::Ptr tree( new pcl::search::KdTree<pcl::PointWithScale>());

    double cloud_resolution = compute_cloud_resolution( cloud );
    iss_detector.setSalientRadius (6 * cloud_resolution);
    iss_detector.setNonMaxRadius (4 * cloud_resolution);

    iss_detector.setThreshold21 (0.99);//0.975
    iss_detector.setThreshold32 (0.99);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (1);

    iss_detector.setInputCloud (cloud);//was without star
    pcl::PointCloud<pcl::PointWithScale>::Ptr kpts( new pcl::PointCloud<pcl::PointWithScale>() );
    iss_detector.compute(*kpts);
    return kpts;
}


/*
int main( int argc, char ** argv ) {
  pcl::console::setVerbosityLevel( pcl::console::L_ERROR );

  // convert to point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1( new pcl::PointCloud<pcl::PointXYZRGB>() );
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2( new pcl::PointCloud<pcl::PointXYZRGB>() );
  //pcl::io::loadPCDFile ("cloud_nocoor.pcd", *cloud);
  pcl::io::loadPCDFile ("../../data/cloud_overlap_2.pcd", *cloud_1);
  pcl::io::loadPCDFile ("../../data/cloud_overlap_1.pcd", *cloud_2);

  // keypoint extraction
    //double threshold_harris3d =1000.1;
    //pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr harris_detector_1(new pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>(  ) );
    //pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>::Ptr harris_detector_2(new pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI>(  ) );

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_1( new pcl::PointCloud<pcl::PointXYZRGB>() );
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_2( new pcl::PointCloud<pcl::PointXYZRGB>() );
    //pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_11( new pcl::PointCloud<pcl::PointXYZI>() );
    //pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_21( new pcl::PointCloud<pcl::PointXYZI>() );
  float set_radius =1.0,set_radius_search=1.0;
  //keypoints_1= harris3d( cloud_1, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius, set_radius_search);
  //keypoints_2= harris3d( cloud_2, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius, set_radius_search);

    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_1_withScale( new pcl::PointCloud<pcl::PointWithScale>() );
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_2_withScale( new pcl::PointCloud<pcl::PointWithScale>() );
    keypoints_1_withScale=iss3d_PointWithScale(cloud_1);
    keypoints_2_withScale=iss3d_PointWithScale(cloud_2);
    std::cout << "keypoints_1_withScale  :" << keypoints_1_withScale->size() << std::endl;
    std::cout << "keypoints_2_withScale :" << keypoints_2_withScale->size() << std::endl;


  keypoints_1=  iss3d(cloud_1);
  keypoints_2=  iss3d(cloud_2);


  pcl::PointCloud<pcl::Normal>::Ptr normals_1( new pcl::PointCloud<pcl::Normal>() );
  pcl::PointCloud<pcl::Normal>::Ptr normals_2( new pcl::PointCloud<pcl::Normal>() );

  // normal extraction
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    float voxel_grid_leaf_size = 1.8;
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_1);
    sor.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
    sor.filter(*downsampled1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.setInputCloud(cloud_2);
    sor.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
    sor.filter(*downsampled2);

    //--------------copy cloud into downsmalped cloud tmp
    //todo remove this after testing
    pcl::copyPointCloud( *cloud_1, *downsampled1);
    pcl::copyPointCloud( *cloud_2, *downsampled2 );


    std::cout << "Num of total cloud1  :" << cloud_1->size() << std::endl;
    std::cout << "Num of downsampled1 :" << downsampled1->size() << std::endl;
    std::cout << "Num of total cloud1 :" << cloud_2->size() << std::endl;
    std::cout << "Num of downsampled1 :" << downsampled2->size() << std::endl;


  //---------------get basic keypoint ratio---------------------
    Mapsample mapsample_1;
    mapsample_1.set_values(-100.0,-100.0,-100.0,100.0,10.0,10.0);
    Mapsample mapsample_2;
    mapsample_2.set_values(-200.0,-50.0,-120.0,50.0,10.0,0.0);
    Mapsample submap_overlap;
    get_overlap(mapsample_1, mapsample_2, submap_overlap);


    //calculate the overlapping points
    //visualize the keypoints off bodyFiltered_1 with lines
    int keypoint_Overlap_cnt=0, keypoint_Overlaparea_fit_cnt=0, keypoints_1_inOverlap=0, keypoints_2_inOverlap=0;
    for (size_t i = 0; i < keypoints_2->size(); ++i) {
        if( (keypoints_2->points[i].x > submap_overlap.minX) && (keypoints_2->points[i].x < submap_overlap.maxX) && (keypoints_2->points[i].y > submap_overlap.minY) && (keypoints_2->points[i].x < submap_overlap.maxY)){
            keypoints_2_inOverlap++;
        }

        for (size_t ii = 0; ii < keypoints_1->size(); ++ii) {
            //check if a coresponding keypoint exists
            //only check x and y since noise and offset is added to the second cloud/submap
            if( (keypoints_2->points[i].x==keypoints_1->points[ii].x) && (keypoints_2->points[i].y==keypoints_1->points[ii].y)){
                keypoint_Overlap_cnt++;
            }
            if( (keypoints_1->points[i].x > submap_overlap.minX) && (keypoints_1->points[i].x < submap_overlap.maxX) && (keypoints_1->points[i].y > submap_overlap.minY) && (keypoints_1->points[i].x < submap_overlap.maxY)){
                if( (keypoints_2->points[i].x==keypoints_1->points[ii].x) && (keypoints_2->points[i].y==keypoints_1->points[ii].y)){
                    keypoint_Overlaparea_fit_cnt++; //the points in the overalapping area which ahve the same coordinates
                }
            }

        }
    }
    for (size_t counter = 0; counter < keypoints_1->size(); ++counter) {
        if( (keypoints_1->points[counter].x > submap_overlap.minX) && (keypoints_1->points[counter].x < submap_overlap.maxX) && (keypoints_1->points[counter].y > submap_overlap.minY) && (keypoints_1->points[counter].x < submap_overlap.maxY)){
            keypoints_1_inOverlap++;
        }
    }
    for (size_t counter = 0; counter < keypoints_2->size(); ++counter) {
        if( (keypoints_2->points[counter].x > submap_overlap.minX) && (keypoints_2->points[counter].x < submap_overlap.maxX) && (keypoints_2->points[counter].y > submap_overlap.minY) && (keypoints_2->points[counter].x < submap_overlap.maxY)){
            keypoints_2_inOverlap++;
        }
    }


    std::cout << "----------------------------------------------------------------"  << std::endl;
    std::cout << "total Num of Overlapping Keypoints (coordinates): " << keypoint_Overlap_cnt << std::endl;
    std::cout << "Num of Overlapping Keypoints_1 in the overlapping area: " << keypoints_1_inOverlap << std::endl;
    std::cout << "Num of Overlapping Keypoints_2 in the overlapping area: " << keypoints_2_inOverlap << std::endl;
    std::cout << "Num Keypoints_1 in the region of interest fitting Keypoints_2 (coord): " << keypoint_Overlaparea_fit_cnt << std::endl;
    double overlap_rate=0, total_overlap_rate=0;

    if(keypoints_1_inOverlap<keypoints_2_inOverlap){
        overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1_inOverlap;
    }else{
        overlap_rate=(double) keypoint_Overlap_cnt/ (double) keypoints_2_inOverlap;
    }
    if(keypoints_1->size()<keypoints_2->size()){
        total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1->size();
    }else{
        total_overlap_rate=(double) keypoint_Overlap_cnt/ (double) keypoints_2->size();
    }
    std::cout << "Overlapping Rate of Keypoints: " << overlap_rate << std::endl;
    std::cout << "Overlapping Rate of Keypoints: " << total_overlap_rate << std::endl;
    std::cout << "----------------------------------------------------------------"  << std::endl;



    return (1);
}
*/