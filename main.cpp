
#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <ctime>
#include <vector>
#include <sstream>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>


#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>


#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <vector>
#include <pcl/filters/filter.h>
//#include "features.h"

//features
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

//own added
#include <random>
#include <chrono>
#include "overlap.h"
#include <pcl/keypoints/harris_3d.h>
#include "keypoints.h"
#include <cxxopts.hpp> //for arg input
#include <pcl/common/transforms.h>



#include <fstream> //write to file
#include <cmath> //to get absolute values
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/pcd_io.h>


//new keypoint methods
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/harris_6d.h>


//------------------------------------------------------------------------------------------------------------------------------
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


//--------------------------------
using namespace std;

// Hyper parameters
#define LEAF_SIZE .1    //for the sampling of the cloud -> 0.1 no filtering applied
//for computing the normals
//#define normal_radius 5;//1.2 //0.25  -> very good results with 0.25, 5, 10 and feature radius10.25
// for compute_PFHRGB_features
// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//#define feature_radius 5.25//3.25  //0.25


//#define RANSAC_Inlier_Threshold 3.//3.//1.5 //0.1
//#define RANSAC_Iterations 5000
#define CorrRejDist_Maximum_Distance 5
#define ICP_Max_Correspondence_Distance 0.15
//--------------------------------

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
                              pcl::Correspondences &remaining_correspondences,
                              double RANSAC_Inlier_Threshold,
                              int RANSAC_Iterations)
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
        std::cout << "inside findCorrespondences_UniqueShapeContext1960" << std::endl;
        pcl::registration::CorrespondenceEstimation<pcl::UniqueShapeContext1960, pcl::UniqueShapeContext1960> est;
        std::cout << "def findCorrespondences_UniqueShapeContext1960" << std::endl;
        est.setInputSource(fpfhs_src);
        est.setInputTarget(fpfhs_tgt);
        std::cout << "set in and target findCorrespondences_UniqueShapeContext1960" << std::endl;
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

//------------------------------------------------------------------------------------------------------------------------------



inline bool exists_file (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

void detect_keypoints_SIFT(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &points,
    pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints_out,
    double min_scale, int nr_octaves, int nr_scales_per_octave, double min_contrast) {

    pcl::SIFTKeypoint <pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
    // Use a FLANN-based KdTree to perform neighbourhood searches
    pcl::search::KdTree <pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZRGB>);
    sift_detect.setSearchMethod(tree);
    // Set the detection parameters
    sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    sift_detect.setMinimumContrast(min_contrast);
    // Set the input
    sift_detect.setInputCloud(points);
    // Detect the keypoints and store them in "keypoints.out"
    sift_detect.compute(*keypoints_out);
}

std::tuple<uint8_t, uint8_t, uint8_t> jet(double x){
    const double rone = 0.8;
    const double gone = 1.0;
    const double bone = 1.0;
    double r, g, b;

    x = (x < 0 ? 0 : (x > 1 ? 1 : x));

    if (x < 1. / 8.) {
        r = 0;
        g = 0;
        b = bone * (0.5 + (x) / (1. / 8.) * 0.5);
    } else if (x < 3. / 8.) {
        r = 0;
        g = gone * (x - 1. / 8.) / (3. / 8. - 1. / 8.);
        b = bone;
    } else if (x < 5. / 8.) {
        r = rone * (x - 3. / 8.) / (5. / 8. - 3. / 8.);
        g = gone;
        b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
    } else if (x < 7. / 8.) {
        r = rone;
        g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
        b = 0;
    } else {
        r = (rone - (x - 7. / 8.) / (1. - 7. / 8.) * 0.5);
        g = 0;
        b = 0;
    }
    return std::make_tuple(uint8_t(255.*r), uint8_t(255.*g), uint8_t(255.*b));
}


//does the prescalling for jet -> maps z to [0-1]:[1-0] in the area between 0 and threshold
//e.g. points along a linear line in z direction would get be: blue, green, yellow, red, yellow, green, blue, green,...
std::tuple<uint8_t, uint8_t, uint8_t> stacked_jet(double z, double threshold){
    pcl::PointXYZRGB pointrgb;
    std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
    double r, g, b, val;
    if(z<=0){
        while(z<0){
            z+=threshold;
        }
    }else{
        while(z>threshold){
            z-=threshold;
        }
    }
    if(z>threshold/2){
        z-=(threshold/2);
        val=-((z/(threshold/2))-1);
    }else{
        val=z/(threshold/2);
    }

    return jet(val);
}


void compute_normals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &points,
     pcl::PointCloud <pcl::Normal>::Ptr &normals_out,
     double normal_radius) {

    pcl::NormalEstimation <pcl::PointXYZRGB, pcl::Normal> norm_est;
    // Use a FLANN-based KdTree to perform neighbourhood searches
    norm_est.setSearchMethod(pcl::search::KdTree <pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree <pcl::PointXYZRGB>));

    norm_est.setRadiusSearch(normal_radius);
    norm_est.setInputCloud(points);
    norm_est.compute(*normals_out);
}

void add_noise_normal_distributedvoid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double mean, double stdv, double x_o, double y_o, double z_o, int red, int green, int blue){

    //generate seed and def dist. generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution(mean,stdv);

    //add noise
    double avg=mean;
    for (std::size_t i = 0; i < cloud->points.size (); ++i){
        double noise = distribution(generator);
        cloud->points[i].z = cloud->points[i].z + noise + z_o;
        cloud->points[i].x = cloud->points[i].x + x_o;  //add an offset to the x var
        cloud->points[i].y = cloud->points[i].y + y_o;  //add an offset to the x var

        avg=(avg+noise)/2;
    }
    //todo remove after testing for color
    if(red!=0 && green!=0 && blue!=0){
        for (std::size_t i = 0; i < cloud->points.size (); ++i) {
            cloud->points[i].r = red;
            cloud->points[i].g = green;
            cloud->points[i].b = blue;
        }

    }
    std::cout << "avg: " << avg << std::endl;
    std::cout << "normal_distributed noise added("<< mean << ","<< stdv << "):" << std::endl;
}


void compute_PFH_features(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud,
                             pcl::PointCloud <pcl::Normal>::Ptr &normals,
                             pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
                             pcl::PointCloud <pcl::PFHSignature125>::Ptr &descriptors_out,
                             double feature_radius_PFH) {


    // copy only XYZ data of keypoints for use in estimating features
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfhEstimation;


    pfhEstimation.setInputCloud(keypoints_xyzrgb);
    pfhEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure
    pfhEstimation.setInputNormals(normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pfhEstimation.setSearchMethod(tree);
    //pfhrgbEstimation.setKSearch(100);

    // Use all neighbors in a sphere of radius radius
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfhEstimation.setRadiusSearch(feature_radius_PFH);

    // Compute the features
    pfhEstimation.compute(*descriptors_out);

}


void findCorrespondences_PFH (pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_src,
                                       pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_tgt,
                                       pcl::Correspondences &all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
    est.setInputSource(fpfhs_src);
    est.setInputTarget(fpfhs_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}


void compute_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tgt,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_src_visualize_temp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_tgt_visualize_temp,
    std::string keypoints_meth,
    double min_scale_SIFT, int nr_octaves_SIFT, int nr_scales_per_octave_SIFT, double min_contrast_SIFT,
    float set_radius_harris, float set_radius_search_harris, std::string HarrisRosponseMethod,
    int SalientRad_muliplier_ISS, int NonMaxMultiplier_ISS, double Threshold21_ISS, double Threshold32_ISS, int setMinNeighbors_ISS, int setNumberOfThreads_ISS,
    double normal_radius) {


    // ESTIMATING KEY POINTS
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_src(new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_tgt(new pcl::PointCloud<pcl::PointWithScale>);


    //  COMPUTING NORMALS
    pcl::PointCloud <pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud <pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>);


    compute_normals(src, src_normals, normal_radius);
    compute_normals(tgt, tgt_normals, normal_radius);

    if(keypoints_meth=="ISS"){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_iss_src( new pcl::PointCloud<pcl::PointXYZRGB>() );
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_iss_tgt( new pcl::PointCloud<pcl::PointXYZRGB>() );
        cout << "chosen Method is ISS" << endl;
        keypoints_iss_src=  iss3d(src, SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS, Threshold32_ISS, setMinNeighbors_ISS, setNumberOfThreads_ISS);
        keypoints_iss_tgt=  iss3d(tgt, SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS, Threshold32_ISS, setMinNeighbors_ISS, setNumberOfThreads_ISS);

        pcl::copyPointCloud(*keypoints_iss_src, *keypoints_src);
        pcl::copyPointCloud(*keypoints_iss_tgt, *keypoints_tgt);
        cout << "No of ISS points in the src are " << keypoints_iss_src->points.size() << endl;
        cout << "No of ISS points in the tgt are " << keypoints_iss_tgt->points.size() << endl;
    }else if(keypoints_meth=="Harris"){
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris_scr( new pcl::PointCloud<pcl::PointXYZI>() );
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris_tgt( new pcl::PointCloud<pcl::PointXYZI>() );
        //float set_radius =1.7,set_radius_search=1.7;
        cout << "chosen Method is Harris3D" << endl;

        cout << "HarrisRosponseMethod: " << HarrisRosponseMethod << endl;
        if (HarrisRosponseMethod=="Harris") {
            keypoints_harris_scr = harris3d(src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS, set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt = harris3d(tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS, set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("CURVATURE"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::CURVATURE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::CURVATURE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("NOBLE"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::NOBLE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::NOBLE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("TOMASI"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else if("LOWE"){
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }else{
            keypoints_harris_scr= harris3d( src, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            keypoints_harris_tgt= harris3d( tgt, pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE ,set_radius_harris, set_radius_search_harris);
            cout << "No of Harris3D points in the src are " << keypoints_harris_scr->points.size() << endl;
            cout << "No of Harris3D points in the tgt are " << keypoints_harris_tgt->points.size() << endl;
        }

        pcl::copyPointCloud(*keypoints_harris_scr, *keypoints_src);
        pcl::copyPointCloud(*keypoints_harris_tgt, *keypoints_tgt);
    }else{
        cout << "chosen Method is SWIFT" << endl;
        detect_keypoints_SIFT(src, keypoints_src, min_scale_SIFT, nr_octaves_SIFT, nr_scales_per_octave_SIFT, min_contrast_SIFT);
        detect_keypoints_SIFT(tgt, keypoints_tgt, min_scale_SIFT, nr_octaves_SIFT, nr_scales_per_octave_SIFT, min_contrast_SIFT);

        cout << "No of SIFT points in the src are " << keypoints_src->points.size() << endl;
        cout << "No of SIFT points in the tgt are " << keypoints_tgt->points.size() << endl;
    }

    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::copyPointCloud(*keypoints_src, *keypoints_src_visualize_temp);
    pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_visualize_temp);
}

void keypoint_evaluation(Mapsample& submap_overlap, const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_1, const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_2){
    double overlap_rate=0, total_overlap_rate=0;
    int keypoint_Overlap_cnt=0, keypoint_Overlaparea_fit_cnt=0, keypoints_1_inOverlap=0, keypoints_2_inOverlap=0;

    //calculate the overlapping points
    //visualize the keypoints off bodyFiltered_1 with lines
    for (size_t i = 0; i < keypoints_2->size(); ++i) {
        for (size_t ii = 0; ii < keypoints_1->size(); ++ii) {
            //check if a coresponding keypoint exists
            //only check x and y since noise and offset is added to the second cloud/submap
            if( (keypoints_2->points[i].x==keypoints_1->points[ii].x) && (keypoints_2->points[i].y==keypoints_1->points[ii].y)){
                keypoint_Overlap_cnt++;
            }
            //is in area and overlapping
            if( (keypoints_1->points[i].x > submap_overlap.minX) && (keypoints_1->points[i].x < submap_overlap.maxX) && (keypoints_1->points[i].y > submap_overlap.minY) && (keypoints_1->points[i].x < submap_overlap.maxY)){
                if( (keypoints_2->points[i].x==keypoints_1->points[ii].x) && (keypoints_2->points[i].y==keypoints_1->points[ii].y)){
                    keypoint_Overlaparea_fit_cnt++; //the points in the overalapping area which have the same coordinates
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
    std::cout << "Num of Keypoints_1 in the overlapping area: " << keypoints_1_inOverlap << std::endl;
    std::cout << "Num of Keypoints_2 in the overlapping area: " << keypoints_2_inOverlap << std::endl;
    std::cout << "Total Keypoints 1: " <<  keypoints_1->size() << std::endl;
    std::cout << "Total Keypoints 2: " <<  keypoints_2->size() << std::endl;

    if(keypoints_1_inOverlap<keypoints_2_inOverlap){
        overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1_inOverlap;
    }else{
        overlap_rate=(double) keypoint_Overlap_cnt/ (double) keypoints_2_inOverlap;
    }
    if(keypoints_1->size()<keypoints_2->size()){
        total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1->size();
    }else{
        total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_2->size();
    }

    std::cout << "total Num of Overlapping Keypoints (coordinates): " << keypoint_Overlap_cnt << std::endl;
;
    std::cout << "Num Keypoints_1 in the region of interest fitting Keypoints_2 (coord): " << keypoint_Overlaparea_fit_cnt << std::endl;
    overlap_rate= (double) keypoint_Overlaparea_fit_cnt / (double) keypoints_1_inOverlap;
    std::cout << "Overlapping Rate of Keypoints 1: " << overlap_rate << std::endl;
    overlap_rate= (double) keypoint_Overlaparea_fit_cnt / (double) keypoints_2_inOverlap;
    std::cout << "Overlapping Rate of Keypoints 2: " << overlap_rate << std::endl;
    total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_1->size();
    std::cout << "Total Overlapping Rate of Keypoints 1: " << total_overlap_rate << std::endl;
    total_overlap_rate= (double) keypoint_Overlap_cnt / (double) keypoints_2->size();
    std::cout << "Total Overlapping Rate of Keypoints 2: " << total_overlap_rate << std::endl;
    std::cout << "----------------------------------------------------------------"  << std::endl;
}



int* correspondance_evaluation(const pcl::CorrespondencesPtr &good_correspondences,
                               const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_src_visualize_temp,
                               const pcl::PointCloud<pcl::PointWithScale>::Ptr &tgt_transformed_eval,
                               double &counter_correctFmatch,
                               double &counter_wrongFmatch,
                               double &counter_validationR,
                               const double validation_radius_in,
                               const double validation_radius_out,
                               int distance_bin_results[10]){

    double evaluation_distances[9] = {0.01, 0.025, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1};
    //int distance_bin_results[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double x_diff=0, y_diff=0, distance=0;

    for (int i = 0; i < good_correspondences->size(); ++i){

        pcl::PointWithScale & src_idx = keypoints_src_visualize_temp->points[(*good_correspondences)[i].index_query];
        ////pcl::PointXYZ & tgt_idx = keypoints_tgt_visualize_temp->points[(*good_correspondences)[i].index_match];
        pcl::PointWithScale & tgt_eval_idx = tgt_transformed_eval->points[(*good_correspondences)[i].index_match];
        string lineID = to_string(i);

        //move this to a seperate function later
        //calc the distance for the bin evaluation
        x_diff=src_idx.x-(tgt_eval_idx.x);
        y_diff=src_idx.y-(tgt_eval_idx.y);
        distance=std::sqrt(x_diff * x_diff + y_diff * y_diff);

        for(int counter=0;counter<10;counter++){
            if(distance <evaluation_distances [counter]){
                distance_bin_results[counter]+=1;
                break;
            }else if(counter==9){
                distance_bin_results[9]+=1;
            }
        }

        if ((src_idx.x - tgt_eval_idx.x) * (src_idx.x - tgt_eval_idx.x) +(src_idx.y - tgt_eval_idx.y) * (src_idx.y - tgt_eval_idx.y) <= validation_radius_in * validation_radius_in){
                counter_correctFmatch++;
        }else{
            counter_wrongFmatch++;
            // check if the corresponding point is at least within a certain validation radius
            if ((src_idx.x - tgt_eval_idx.x) * (src_idx.x - tgt_eval_idx.x) +(src_idx.y - tgt_eval_idx.y) * (src_idx.y - tgt_eval_idx.y) <= validation_radius_out * validation_radius_out){
                counter_validationR++;
            }
        }
    }
    if(1){
        //output the bin results
        std::cout << "--------------------------FEATURE--------------------------------------"  << std::endl;
        std::cout << "counter_correctFmatch" << counter_correctFmatch << std::endl;
        std::cout << "counter_wrongFmatch" << counter_wrongFmatch << std::endl;
        std::cout << "counter_validationR" << counter_validationR << std::endl;

        std::cout << "bin results of the distance: " << std::endl;
        std::cout << "bin 0 [0<=" << evaluation_distances [0] << "] :" << distance_bin_results[0]<< std::endl;
        for(int counter=1;counter<9;counter++) {
            std::cout << "bin " << counter << " [" << evaluation_distances [counter-1] << "<=" << evaluation_distances [counter] << "] :" << distance_bin_results[counter]<< std::endl;
        }
        std::cout << "bin 9 [" << evaluation_distances [8] << "<=...] :" << distance_bin_results[9]<< std::endl;
    }
    //arr=distance_bin_results;
    return distance_bin_results;
    //return arr;
}




void feature_evaluation_and_recording(int counter_correctFmatch, int counter_wrongFmatch, int counter_validationR,
        const string src_file, const string tgt_file, const double noise_offset, const double noise_var,
        const double x_o, const double y_o, const double z_o,
        const string keypoint_method, const double jet_flag, const double jet_stacking_threshold,
        Mapsample submap_overlap, int distance_bin_results[10],
        const int src_original_size, const int tgt_original_size,
        const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_tgt_visualize_temp,
        const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_src_visualize_temp,
        const int grid_flag, const string measure_file_str,
        const int SalientRad_muliplier_ISS, const int NonMaxMultiplier_ISS,
        const double Threshold21_ISS, const double Threshold32_ISS, const int setMinNeighbors_ISS,
        const double time_elapsed_computation,
        const double num_good_correspondances,
        const double exact_match_rate, const double evaluation_match_rate, const double validation_radius_in, const double validation_radius_out) {


        double evaluation_distances[9] = {0.01, 0.025, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1};
        if(0){


            //output the bin results
            std::cout << "--------------------------FEATURE--------------------------------------" << std::endl;
            std::cout << "counter_correctFmatch" << counter_correctFmatch << std::endl;
            std::cout << "counter_wrongFmatch" << counter_wrongFmatch << std::endl;
            std::cout << "counter_validationR" << counter_validationR << std::endl;

            std::cout << "bin results of the distance: " << std::endl;
            std::cout << "bin 0 [0<=" << evaluation_distances[0] << "] :" << distance_bin_results[0] << std::endl;
            for (int counter = 1; counter < 9; counter++) {
                std::cout << "bin " << counter << " [" << evaluation_distances[counter - 1] << "<="
                          << evaluation_distances[counter] << "] :" << distance_bin_results[counter] << std::endl;
            }
            std::cout << "bin 9 [" << evaluation_distances[8] << "<=...] :" << distance_bin_results[9] << std::endl;


        }


    //int counter_correctFmatch=0, counter_wrongFmatch=0, counter_validationR=0;
    double x_diff = 0, y_diff = 0;
    double distance = 0;


    //write key information to the measurement file in case e.g. for grid search
    if (grid_flag == 1) {
        std::cout << "write to file " << std::endl;
        std::ofstream out;

        //do the keyoint evaluation here instead
        double overlap_rate = 0, total_overlap_rate = 0;
        int keypoint_Overlap_cnt = 0, keypoint_Overlaparea_fit_cnt = 0, keypoints_1_inOverlap = 0, keypoints_2_inOverlap = 0;

        //calculate the overlapping points
        //visualize the keypoints off bodyFiltered_1 with lines
        for (size_t i = 0; i < keypoints_tgt_visualize_temp->size(); ++i) {
            for (size_t ii = 0; ii < keypoints_src_visualize_temp->size(); ++ii) {
                //check if a coresponding keypoint exists
                //only check x and y since noise and offset is added to the second cloud/submap
                if ((keypoints_tgt_visualize_temp->points[i].x == keypoints_src_visualize_temp->points[ii].x) &&
                    (keypoints_tgt_visualize_temp->points[i].y == keypoints_src_visualize_temp->points[ii].y)) {
                    keypoint_Overlap_cnt++;
                }
                //is in area and overlapping
                if ((keypoints_src_visualize_temp->points[i].x > submap_overlap.minX) &&
                    (keypoints_src_visualize_temp->points[i].x < submap_overlap.maxX) &&
                    (keypoints_src_visualize_temp->points[i].y > submap_overlap.minY) &&
                    (keypoints_src_visualize_temp->points[i].x < submap_overlap.maxY)) {
                    if ((keypoints_tgt_visualize_temp->points[i].x == keypoints_src_visualize_temp->points[ii].x) &&
                        (keypoints_tgt_visualize_temp->points[i].y == keypoints_src_visualize_temp->points[ii].y)) {
                        keypoint_Overlaparea_fit_cnt++; //the points in the overalapping area which have the same coordinates
                    }
                }
            }
        }

        for (size_t counter = 0; counter < keypoints_src_visualize_temp->size(); ++counter) {
            if ((keypoints_src_visualize_temp->points[counter].x > submap_overlap.minX) &&
                (keypoints_src_visualize_temp->points[counter].x < submap_overlap.maxX) &&
                (keypoints_src_visualize_temp->points[counter].y > submap_overlap.minY) &&
                (keypoints_src_visualize_temp->points[counter].x < submap_overlap.maxY)) {
                keypoints_1_inOverlap++;
            }
        }
        for (size_t counter = 0; counter < keypoints_tgt_visualize_temp->size(); ++counter) {
            if ((keypoints_tgt_visualize_temp->points[counter].x > submap_overlap.minX) &&
                (keypoints_tgt_visualize_temp->points[counter].x < submap_overlap.maxX) &&
                (keypoints_tgt_visualize_temp->points[counter].y > submap_overlap.minY) &&
                (keypoints_tgt_visualize_temp->points[counter].x < submap_overlap.maxY)) {
                keypoints_2_inOverlap++;
            }
        }
        std::cout << "----------------------------------------------------------------" << std::endl;
        std::cout << "Num of Keypoints_1 in the overlapping area: " << keypoints_1_inOverlap << std::endl;
        std::cout << "Num of Keypoints_2 in the overlapping area: " << keypoints_2_inOverlap << std::endl;
        std::cout << "Total Keypoints 1: " << keypoints_src_visualize_temp->size() << std::endl;
        std::cout << "Total Keypoints 2: " << keypoints_tgt_visualize_temp->size() << std::endl;

        if (keypoints_1_inOverlap < keypoints_2_inOverlap) {
            overlap_rate = (double) keypoint_Overlap_cnt / (double) keypoints_1_inOverlap;
        } else {
            overlap_rate = (double) keypoint_Overlap_cnt / (double) keypoints_2_inOverlap;
        }
        if (keypoints_src_visualize_temp->size() < keypoints_src_visualize_temp->size()) {
            total_overlap_rate = (double) keypoint_Overlap_cnt / (double) keypoints_src_visualize_temp->size();
        } else {
            total_overlap_rate = (double) keypoint_Overlap_cnt / (double) keypoints_tgt_visualize_temp->size();
        }

        std::cout << "total Num of Overlapping Keypoints (coordinates): " << keypoint_Overlap_cnt << std::endl;
        std::cout << "Num Keypoints_1 in the region of interest fitting Keypoints_2 (coord): "
                  << keypoint_Overlaparea_fit_cnt << std::endl;
        std::cout << "----------------------------------------------------------------" << std::endl;


        if (exists_file(measure_file_str)) {
            out.open(measure_file_str, std::ios::app);
        } else {
            out.open(measure_file_str, std::ios::app);
            //add the header in the csv file
            out << "src_file," << "tgt_file," << "Src_points," << "Tgt-points," << "noise_offest," << "noise_stdv,"
                << "x_o," << "y_o," << "z_o," << "keypoint_method," << "Jetstatus," << "Stacked_height_threshold,"
                << "SalientRad_muliplier_ISS," << "NonMaxMultiplier_ISS," << "Threshold21_ISS," << "Threshold32_ISS,"
                << "setMinNeighbors_ISS," << "setNumberOfThreads_ISS," << "___,";
            out << "comp_time[s],";
            out << "keypoints_1_inOverlap," << "keypoints_2_inOverlap," << "Total_Keypoints_1," << "Total_Keypoints_2,"
                << "keypoint_Overlap_cnt," << "keypoint_Overlaparea_fit_cnt,";
            out << "good_correspondences"<< "," <<"counter_correctFmatch"<< "," <<"counter_validationR"<< "," << "exact_match_rate"<< "," <<"evaluation_match_rate"<< "," <<"validation_radius_in"<< "," <<"<<validation_radius_out"<< std::endl;

        }


        //add measurment conditions
        out << src_file << "," << tgt_file << "," << src_original_size << "," << tgt_original_size << ","
            << noise_offset << "," << noise_var << "," << x_o << "," << y_o << "," << z_o << "," << keypoint_method
            << "," << jet_flag << "," << jet_stacking_threshold << "," << SalientRad_muliplier_ISS << ","
            << NonMaxMultiplier_ISS << "," << Threshold21_ISS << "," << Threshold32_ISS << "," << setMinNeighbors_ISS;
        out << ",___,";
         //todo enter here the time needed for the calculation
        out << time_elapsed_computation << "," << keypoints_1_inOverlap << "," << keypoints_2_inOverlap << ","
            << keypoints_src_visualize_temp->size() << "," << keypoints_tgt_visualize_temp->size() << ","
            << keypoint_Overlap_cnt << "," << keypoint_Overlaparea_fit_cnt<< ",";
        out << num_good_correspondances<< "," <<counter_correctFmatch<< "," <<counter_validationR<< "," << exact_match_rate<< "," <<evaluation_match_rate<< "," <<validation_radius_in<< "," <<validation_radius_out<< std::endl;


    }
}


int main(int argc, char** argv) {
    int roation_flag = 0, z_rejection_flag = 0, cloud_filter_flag = 0;
    int red = 0, green = 0, blue = 0, jet_flag = 0, grid_flag = 0;
    //parse input para
    double validation_radius_in = 0.0, validation_radius_out = 0.0;
    double noise_offset = 0.0, noise_var = 0.0, x_o = 0.0, y_o = 0.0, z_o = 0.0, rot_alpha = 0.0, rot_betha = 0.0, rot_gamma = 0.0;
    double visu_dist = 0.0, jet_stacking_threshold = 30.0;

    //Parameter for Keypoint methods
    double min_scale_SIFT = 0.2;
    int nr_octaves_SIFT = 4;
    int nr_scales_per_octave_SIFT = 5;
    double min_contrast_SIFT = 0.25;
    float set_radius_harris = 1.7, set_radius_search_harris = 1.7;
    int SalientRad_muliplier_ISS = 6;
    int NonMaxMultiplier_ISS = 4;
    double Threshold21_ISS = 0.99, Threshold32_ISS = 0.99;
    int setMinNeighbors_ISS = 5, setNumberOfThreads_ISS = 1;
    float voxel_grid_leaf_size = 1.8;

    //Parameter for Feature descriptor methods
    int omp_on_flag = 0; //set to on for parallel computing of certain parameter
    double RANSAC_Inlier_Threshold = 3.0;//3.//1.5 //0.1 for Correspndance Rejection
    int RANSAC_Iterations = 5000;   //for Correspndance Rejection
    double GradientEstimationsetRadiusSearch_RIFT = 5; //1000.001
    double RadiusSearch_RIFT = 5.25;//100.05
    double NrDistanceBins_RIFT = 4;
    double NrGradientBins_RIFT = 8;

    double setRadiusSearch_RSD = 0.05;//15.25;//0.5;
    double setPlaneRadius_RSD = 0.01;//10.25;//0.01;

    double feature_radius_PFHRGB = 5.25;//0.05
    float feature_radius_PFH = 15.66;
    double searchRadius_FPFH = 5.25;

    double setRadiusSearch_Shape_3DSC = 5.25;//0.01;
    double setMinimalRadius_div_3DSC = 5;
    double setPointDensityRadius_div_3DSC = 10;

    double RadiusSearch_USC = 5.25;//0.01;
    double setMinimalRadius_div_USC = 5;
    double setPointDensityRadius_div_USC = 10;
    double LocalRadiusSearch_USC = 5.25;
    double setRadiusSearch_SHOT = 5.25;

    //here are all possible input parameter listed
    cxxopts::Options options("test", "A brief description");
    options.add_options()
            ("e,validation_in",
             "The radius in which a feature point will count as (exact) correct if not fitting excatly",
             cxxopts::value<double>(validation_radius_in)->default_value("0.0"))
            ("v,validation_out", "The radius in which a feature point will count as correct if not fitting excatly",
             cxxopts::value<double>(validation_radius_out)->default_value("0.0"))
            ("q,grid", "set for minimal terminnal output in e.g. grid search ",
             cxxopts::value<int>(grid_flag)->default_value("0"))

            ("f,output_filename", "filename to save the measurement data in", cxxopts::value<std::string>())
            ("input_filename_1", "input file 1 with multibeam data", cxxopts::value<std::string>())
            ("input_filename_2", "input file 2 with multibeam data", cxxopts::value<std::string>())
            ("w,z_rejection_flag", "enable a second rejection cycle",
             cxxopts::value<int>(z_rejection_flag)->default_value("0"))

            ("o,noiseoff", "Param foo", cxxopts::value<double>(noise_offset)->default_value("0.0"))
            ("n,noisestdv", "Param foo", cxxopts::value<double>(noise_var)->default_value("0.0"))
            ("x,xoffset", "Param foo", cxxopts::value<double>(x_o)->default_value("0.0"))
            ("y,yoffset", "Param foo", cxxopts::value<double>(y_o)->default_value("0.0"))
            ("z,zoffset", "Param foo", cxxopts::value<double>(z_o)->default_value("0.0"))
            ("d,visdistance", "visualization distance", cxxopts::value<double>(visu_dist)->default_value("0.0"))
            ("a,alpha", "roation around z axis (insert in radiant)",
             cxxopts::value<double>(rot_alpha)->default_value("0.0"))
            ("b,betha", "roation around y axis (insert in radiant)",
             cxxopts::value<double>(rot_betha)->default_value("0.0"))
            ("c,gamma", "roation around x axis (insert in radiant)",
             cxxopts::value<double>(rot_gamma)->default_value("0.0"))

            ("voxel_grid_leaf_size", "size of the voxel_grid leave size for filtering the cloud",
             cxxopts::value<float>(voxel_grid_leaf_size)->default_value("1.8"))
            ("cloud_filter_flag",
             "if set to one the cloud is filtered via an voxel-grid before any further processing is done (less points->faster keypoint detection)",
             cxxopts::value<int>(cloud_filter_flag)->default_value("0"))

            ("feature_method", "method used to build the descriptor (PFHRGB, FPFH, PFH, 3DSC, USC, SHOT, CSHOT, RSD)",
             cxxopts::value<std::string>())
            ("omp_on_flag", "use parallel computing to speed up the calculation (for SHOT, CSHOT, FPFH)",
             cxxopts::value<int>(omp_on_flag)->default_value("0"))
            ("RANSAC_Inlier_Threshold", "Parameter for Correspondance Rejection",
             cxxopts::value<double>(RANSAC_Inlier_Threshold)->default_value("3.0"))
            ("RANSAC_Iterations", "Parameter for Correspondance Rejection",
             cxxopts::value<int>(RANSAC_Iterations)->default_value("5000"))

            ("GradientEstimationsetRadiusSearch_RIFT", "Parameter for RIFT",
             cxxopts::value<double>(GradientEstimationsetRadiusSearch_RIFT)->default_value("5.0"))
            ("RadiusSearch_RIFT", "Parameter for RIFT",
             cxxopts::value<double>(RadiusSearch_RIFT)->default_value("5.25"))
            ("NrDistanceBins_RIFT", "Parameter for RIFT",
             cxxopts::value<double>(NrDistanceBins_RIFT)->default_value("4.0"))
            ("NrGradientBins_RIFT", "Parameter for RIFT",
             cxxopts::value<double>(NrGradientBins_RIFT)->default_value("8.0"))
            ("feature_radius_PFHRGB", "Parameter for PFHRGB",
             cxxopts::value<double>(feature_radius_PFHRGB)->default_value("5.25"))
            ("feature_radius_PFH", "Parameter for PFH",
             cxxopts::value<float>(feature_radius_PFH)->default_value("15.66"))
            ("setRadiusSearch_Shape_3DSC", "Parameter for 3DSC",
             cxxopts::value<double>(setRadiusSearch_Shape_3DSC)->default_value("5.25"))
            ("setMinimalRadius_div_3DSC", "Parameter for 3DSC",
             cxxopts::value<double>(setMinimalRadius_div_3DSC)->default_value("5.0"))
            ("setPointDensityRadius_div_3DSC", "Parameter for 3DSC",
             cxxopts::value<double>(setPointDensityRadius_div_3DSC)->default_value("10.0"))
            ("RadiusSearch_USC", "Parameter for USC", cxxopts::value<double>(RadiusSearch_USC)->default_value("5.25"))
            ("setMinimalRadius_div_USC", "Parameter for USC",
             cxxopts::value<double>(setMinimalRadius_div_USC)->default_value("5.0"))
            ("setPointDensityRadius_div_USC", "Parameter for USC",
             cxxopts::value<double>(setPointDensityRadius_div_USC)->default_value("10.0"))
            ("LocalRadiusSearch_USC", "Parameter for USC",
             cxxopts::value<double>(LocalRadiusSearch_USC)->default_value("5.25"))
            ("setRadiusSearch_SHOT", "Parameter for SHOT",
             cxxopts::value<double>(setRadiusSearch_SHOT)->default_value("5.25"))
            ("setRadiusSearch_RSD", "Parameter for RSD",
             cxxopts::value<double>(setRadiusSearch_RSD)->default_value("0.05"))
            ("setPlaneRadius_RSD", "Parameter for RSD",
             cxxopts::value<double>(setPlaneRadius_RSD)->default_value("0.01"))
            ("searchRadius_FPFH", "Parameter for FPFH",
             cxxopts::value<double>(searchRadius_FPFH)->default_value("5.25"))


            ("m,method", "method to search keypoints (ISS, Harris, otherwise SWIFT)", cxxopts::value<std::string>())
            ("min_scale_SIFT", "input parameter min_scale_SIFT for SIFT",
             cxxopts::value<double>(min_scale_SIFT)->default_value("0.2"))
            ("nr_octaves_SIFT", "input parameter nr_octaves_SIFT for SIFT",
             cxxopts::value<int>(nr_octaves_SIFT)->default_value("4"))
            ("nr_scales_per_octave_SIFT", "input parameter nr_scales_per_octave_SIFT for SIFT",
             cxxopts::value<int>(nr_scales_per_octave_SIFT)->default_value("5"))
            ("min_contrast_SIFT", "input parameter min_contrast_SIFT for SIFT",
             cxxopts::value<double>(min_contrast_SIFT)->default_value("0.25"))

            ("set_radius_harris", "input parameter set_radius_harris for 3DHarris",
             cxxopts::value<float>(set_radius_harris)->default_value("1.7"))
            ("set_radius_search_harris", "input parameter set_radius_search_harris for 3DHarris",
             cxxopts::value<float>(set_radius_search_harris)->default_value("1.7"))
            ("HarrisRosponseMethod",
             "Response method for Harris, choose betweeen HARRIS,TOMASI,NOBLE,CURVATURE and LOWE (default)",
             cxxopts::value<std::string>())

            ("SalientRad_muliplier_ISS", "input parameter SalientRad_muliplier_ISS for ISS",
             cxxopts::value<int>(SalientRad_muliplier_ISS)->default_value("6"))
            ("NonMaxMultiplier_ISS", "input parameter NonMaxMultiplier_ISS for ISS",
             cxxopts::value<int>(NonMaxMultiplier_ISS)->default_value("4"))
            ("Threshold21_ISS", "input parameter min_scale_SIFT for ISS",
             cxxopts::value<double>(Threshold21_ISS)->default_value("0.99"))
            ("Threshold32_ISS", "input parameter Threshold32_ISS for ISS",
             cxxopts::value<double>(Threshold32_ISS)->default_value("0.99"))
            ("setMinNeighbors_ISS", "input parameter setMinNeighbors_ISS for ISS",
             cxxopts::value<int>(setMinNeighbors_ISS)->default_value("5"))
            ("setNumberOfThreads_ISS", "input parameter setNumberOfThreads_ISS for ISS",
             cxxopts::value<int>(setNumberOfThreads_ISS)->default_value("1"))

            ("r,red", "set one color red", cxxopts::value<int>(red)->default_value("0"))
            ("g,green", "set one color green", cxxopts::value<int>(green)->default_value("0"))
            ("u,blue", "set one color blue", cxxopts::value<int>(blue)->default_value("0"))
            ("j,jet", "apply jet function to color", cxxopts::value<int>(jet_flag)->default_value("0"))
            ("i,staked_height", "the height for the stacked color",
             cxxopts::value<double>(jet_stacking_threshold)->default_value("30.0"))
            ("h,help", "Print usage");
    auto result = options.parse(argc, argv);

    std::cout << "validation_radius_in: " << validation_radius_in << std::endl;
    std::cout << "validation_radius_out: " << validation_radius_out << std::endl;
    if (result.count("n") || result.count("o")) {
        std::cout << "noise offest: " << noise_offset << std::endl;
        std::cout << "noise stdv: " << noise_var << std::endl;
    }

    if (result.count("x") || result.count("y") || result.count("z")) {
        std::cout << "x_o: " << x_o << std::endl;
        std::cout << "y_o: " << y_o << std::endl;
        std::cout << "z_o: " << z_o << std::endl;
    }

    if (result.count("a") || result.count("b") || result.count("c")) {
        std::cout << "rot_alpha: " << rot_alpha << std::endl;
        std::cout << "rot_betha: " << rot_betha << std::endl;
        std::cout << "rot_gamma: " << rot_gamma << std::endl;
        roation_flag = 1;
    }
    std::cout << "grid_flag: " << grid_flag << std::endl;

    std::string feature_method;
    if (result.count("feature_method")) {
        std::cout << "feature method = " << result["feature_method"].as<std::string>() << std::endl;
        feature_method = result["feature_method"].as<std::string>();
    } else {
        feature_method = "PFH";
    }
    std::string keypoint_method;
    if (result.count("method")) {
        std::cout << "keypoint method = " << result["method"].as<std::string>() << std::endl;
        keypoint_method = result["method"].as<std::string>();
    } else {
        keypoint_method = "SWIFT";
    }
    std::string HarrisRosponseMethod;
    if (result.count("HarrisRosponseMethod")) {

        std::cout << "HarrisRosponseMethod = " << result["HarrisRosponseMethod"].as<std::string>() << std::endl;
        HarrisRosponseMethod = result["HarrisRosponseMethod"].as<std::string>();
    } else {
        HarrisRosponseMethod = "LOWE";
    }
    if (result.count("i")) {
        std::cout << "Stacked height threshold: " << jet_stacking_threshold << std::endl;
    }
    std::string measure_file_str;
    if (result.count("output_filename")) {
        std::cout << "Output filename for measurement = " << result["output_filename"].as<std::string>() << std::endl;
        measure_file_str = result["output_filename"].as<std::string>();
    } else {
        measure_file_str = "measurement.csv";
    }
    string src_file;
    if (result.count("input_filename_1")) {
        std::cout << "filename for input 1 = " << result["input_filename_1"].as<std::string>() << std::endl;
        src_file = result["input_filename_1"].as<string>();
    } else {
        src_file = "../../data/cloud_nocoor.pcd";
    }
    string tgt_file;
    if (result.count("input_filename_2")) {
        std::cout << "filename for input 2 = " << result["input_filename_2"].as<std::string>() << std::endl;
        tgt_file = result["input_filename_2"].as<string>();
    } else {
        tgt_file = "../../data/cloud_overlap__big_2.pcd";
    }

    if (keypoint_method == "SWIFT") {
        std::cout << "SIFT----Parameter--- " << std::endl;
        std::cout << "min_scale_SIFT: " << min_scale_SIFT << std::endl;
        std::cout << "nr_octaves_SIFT: " << nr_octaves_SIFT << std::endl;
        std::cout << "nr_scales_per_octave_SIFT: " << nr_scales_per_octave_SIFT << std::endl;
        std::cout << "min_contrast_SIFT: " << min_contrast_SIFT << std::endl;
        std::cout << "----End of Parameter--- " << std::endl;
    }
    if (keypoint_method == "Harris") {
        std::cout << "Harris----Parameter--- " << std::endl;
        std::cout << "set_radius_harris: " << set_radius_harris << std::endl;
        std::cout << "set_radius_search_harris: " << set_radius_search_harris << std::endl;
        std::cout << "----End of Parameter--- " << std::endl;
    }
    if (keypoint_method == "ISS") {
        std::cout << "ISS----Parameter--- " << std::endl;
        std::cout << "SalientRad_muliplier_ISS: " << SalientRad_muliplier_ISS << std::endl;
        std::cout << "NonMaxMultiplier_ISS: " << NonMaxMultiplier_ISS << std::endl;
        std::cout << "Threshold21_ISS: " << Threshold21_ISS << std::endl;
        std::cout << "Threshold32_ISS: " << Threshold32_ISS << std::endl;
        std::cout << "setMinNeighbors_ISS: " << setMinNeighbors_ISS << std::endl;
        std::cout << "setNumberOfThreads_ISS: " << setNumberOfThreads_ISS << std::endl;
        std::cout << "----End of Parameter--- " << std::endl;
    }

    // Time start (main function)

    //time_t start_computation, end_computation, start_total, end_total;
    //time(&start_total);
    //time(&start_computation);

    //string src_file = "../../data/cloud_nocoor.pcd";        //<--
    //string src_file = "../cloud_overlap_1.pcd";
    //string tgt_file = "../cloud_overlap_2.pcd";
    //string src_file = "../cloud_overlap_sbig_1.pcd";
    //string tgt_file = "../../data/cloud_overlap__big_2.pcd";//<--

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_original(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_original(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(src_file, *src_original) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(tgt_file, *tgt_original) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(tgt_file, *tgt_transformed) == -1) {
        PCL_ERROR("Couldn't read src or tgt file");
        return -1;
    }
    cout << "Src points: " << src_original->points.size() << endl;
    cout << "Tgt points: " << tgt_transformed->points.size() << endl;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_downsampled_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_downsampled_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_downsampled_2_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (cloud_filter_flag == 1) {
        // Create the filtering object

        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(src_original);
        sor.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
        sor.filter(*src_downsampled_1);

        sor.setInputCloud(tgt_transformed);
        sor.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
        sor.filter(*tgt_downsampled_2);

        sor.setInputCloud(tgt_transformed);
        sor.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
        sor.filter(*tgt_downsampled_2_transformed);

        cout << "src_downsampled_1 points: " << src_downsampled_1->size() << endl;
        cout << "tgt_downsampled_2 points: " << tgt_downsampled_2->size() << endl;
        cout << "tgt_downsampled_2_transformed points: " << tgt_downsampled_2_transformed->size() << endl;
    } else {
        copyPointCloud(*src_original, *src_downsampled_1);
        copyPointCloud(*tgt_original, *tgt_downsampled_2);
        copyPointCloud(*tgt_transformed, *tgt_downsampled_2_transformed);

    }

    add_noise_normal_distributedvoid(tgt_downsampled_2_transformed, noise_offset, noise_var, x_o, y_o, z_o, red, green,
                                     blue);


    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_1_inverse = Eigen::Matrix4f::Identity();
    if (roation_flag == 1) {
        transform_1(0, 0) = std::cos(rot_alpha) * std::cos(rot_betha);
        transform_1(0, 1) =
                std::cos(rot_alpha) * sin(rot_betha) * sin(rot_gamma) - sin(rot_alpha) * std::cos(rot_gamma);
        transform_1(0, 2) =
                std::cos(rot_alpha) * sin(rot_betha) * std::cos(rot_gamma) + sin(rot_alpha) * sin(rot_gamma);

        transform_1(1, 0) = sin(rot_alpha) * std::cos(rot_betha);
        transform_1(1, 1) =
                sin(rot_alpha) * sin(rot_betha) * sin(rot_gamma) + std::cos(rot_alpha) * std::cos(rot_gamma);;
        transform_1(1, 2) =
                std::cos(rot_alpha) * sin(rot_betha) * std::cos(rot_gamma) - sin(rot_alpha) * sin(rot_gamma);

        transform_1(2, 0) = -sin(rot_betha);
        transform_1(2, 1) = std::cos(rot_betha) * sin(rot_gamma);
        transform_1(2, 2) = std::cos(rot_betha) * std::cos(rot_gamma);

        pcl::transformPointCloud(*tgt_downsampled_2_transformed, *tgt_downsampled_2_transformed, transform_1);
    }
    transform_1_inverse = transform_1.inverse();

    if (jet_flag == 1 || jet_flag == 2) {
        double total_min_z, total_max_z;
        pcl::PointXYZRGB minPt1, maxPt1, minPt2, maxPt2;;
        pcl::getMinMax3D(*tgt_downsampled_2_transformed, minPt1, maxPt1);
        pcl::getMinMax3D(*src_downsampled_1, minPt2, maxPt2);

        total_min_z = minPt1.z < minPt2.z ? minPt1.z : minPt2.z;
        total_max_z = maxPt1.z > maxPt2.z ? maxPt1.z : maxPt2.z;

        std::cout << "Total min z: " << total_min_z << std::endl;
        std::cout << "Total max z: " << total_max_z << std::endl;

        // Fill the cloud with some points
        if (jet_flag == 2) {
            //regular stacked jet section (was chosen this style to not loop over the if)
            //double jet_stacking_threshold=30;
            for (std::size_t i = 0; i < src_downsampled_1->points.size(); ++i) {
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = stacked_jet(src_downsampled_1->points[i].z, jet_stacking_threshold);

                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float *>(&rgb);
                src_downsampled_1->points[i].r = pointrgb.r;
                src_downsampled_1->points[i].g = pointrgb.g;
                src_downsampled_1->points[i].b = pointrgb.b;
            }
            // Fill the cloud with some points
            for (std::size_t i = 0; i < tgt_downsampled_2_transformed->points.size(); ++i) {
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = stacked_jet(tgt_downsampled_2_transformed->points[i].z, jet_stacking_threshold);
                //colors_rgb = jet(( tgt_original->points[i].z -  total_min_z)/(total_max_z - total_min_z));
                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float *>(&rgb);
                tgt_downsampled_2_transformed->points[i].r = pointrgb.r;
                tgt_downsampled_2_transformed->points[i].g = pointrgb.g;
                tgt_downsampled_2_transformed->points[i].b = pointrgb.b;
            }
        } else {
            //regular section
            for (std::size_t i = 0; i < src_downsampled_1->points.size(); ++i) {
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = jet((src_downsampled_1->points[i].z - total_min_z) / (total_max_z - total_min_z));
                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float *>(&rgb);
                src_downsampled_1->points[i].r = pointrgb.r;
                src_downsampled_1->points[i].g = pointrgb.g;
                src_downsampled_1->points[i].b = pointrgb.b;
            }
            // Fill the cloud with some points
            for (std::size_t i = 0; i < tgt_downsampled_2_transformed->points.size(); ++i) {
                pcl::PointXYZRGB pointrgb;
                std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
                colors_rgb = jet(
                        (tgt_downsampled_2_transformed->points[i].z - total_min_z) / (total_max_z - total_min_z));
                std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                                     static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                                     static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
                pointrgb.rgb = *reinterpret_cast<float *>(&rgb);
                tgt_downsampled_2_transformed->points[i].r = pointrgb.r;
                tgt_downsampled_2_transformed->points[i].g = pointrgb.g;
                tgt_downsampled_2_transformed->points[i].b = pointrgb.b;
            }
        }
    }


    // Filtered point cloud copy
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
    src = src_downsampled_1;
    tgt = tgt_downsampled_2_transformed;

    // Obtain the initial transformation matirx by using the key-points
    Eigen::Matrix4f transform;
    double normal_radius = 5.25;

    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_1_withScale(new pcl::PointCloud<pcl::PointWithScale>());
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_2_withScale(new pcl::PointCloud<pcl::PointWithScale>());

    keypoints_1_withScale = iss3d_PointWithScale(src_downsampled_1);
    keypoints_2_withScale = iss3d_PointWithScale(tgt_downsampled_2_transformed);


    std::cout << "src_downsampled_1  :" << src_downsampled_1->size() << std::endl;
    std::cout << "tgt_downsampled_2_transformed :" << tgt_downsampled_2_transformed->size() << std::endl;
    std::cout << "keypoints_1_withScale  :" << keypoints_1_withScale->size() << std::endl;
    std::cout << "keypoints_2_withScale :" << keypoints_2_withScale->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_2(new pcl::PointCloud<pcl::PointXYZRGB>());
    keypoints_1 = iss3d(src_downsampled_1, SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS,
                        Threshold32_ISS, setMinNeighbors_ISS, setNumberOfThreads_ISS);
    keypoints_2 = iss3d(tgt_downsampled_2_transformed, SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS,
                        Threshold32_ISS, setMinNeighbors_ISS, setNumberOfThreads_ISS);

    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*keypoints_1, *keypoints_src_visualize_temp);
    pcl::copyPointCloud(*keypoints_2, *keypoints_tgt_visualize_temp);
    std::cout << "keypoints_src_visualize_temp  :" << keypoints_src_visualize_temp->size() << std::endl;
    std::cout << "keypoints_1  :" << keypoints_1->size() << std::endl;
    std::cout << "keypoints_2 :" << keypoints_2->size() << std::endl;


    pcl::PointCloud<pcl::Normal>::Ptr normals_1(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr normals_2(new pcl::PointCloud<pcl::Normal>());


    //float normal_radius =5.;//100.1; //0.1
    //normals = normal_extraction( cloud, keypoints, normal_radius);
    //normals_1 = normal_extraction( cloud_1, src_downsampled_1, normal_radius);
    //normals_2 = normal_extraction( cloud_2, tgt_downsampled_2_transformed, normal_radius);
    //---alternative with parallel calculation for normal calculation
    normals_1 = normal_extraction_omp(src_downsampled_1, src_downsampled_1,
                                      normal_radius); //works fine with the features
    normals_2 = normal_extraction_omp(tgt_downsampled_2_transformed, tgt_downsampled_2_transformed,
                                      normal_radius); //works fine with the features

    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_transformed_eval(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_2_withScale_eval(new pcl::PointCloud<pcl::PointWithScale>());
    copyPointCloud(*keypoints_tgt_visualize_temp, *tgt_transformed_eval);
    std::cout << "Derotation evaluation is on" << std::endl;
    if (roation_flag == 1) {
        pcl::transformPointCloud(*tgt_transformed_eval, *tgt_transformed_eval, transform_1_inverse);
        pcl::transformPointCloud(*keypoints_2_withScale, *keypoints_2_withScale_eval, transform_1_inverse);

    }
    for (std::size_t i = 0; i < tgt_transformed_eval->points.size(); ++i) {
        tgt_transformed_eval->points[i].x -= x_o;
        tgt_transformed_eval->points[i].y -= y_o;
        tgt_transformed_eval->points[i].z -= z_o;
    }

    //---------------get basic keypoint ratio---------------------
    pcl::PointXYZRGB minPt1, maxPt1, minPt2, maxPt2;;
    pcl::getMinMax3D(*src, minPt1, maxPt1);
    pcl::getMinMax3D(*tgt, minPt2, maxPt2);

    Mapsample mapsample_1, mapsample_2, submap_overlap;
    mapsample_1.set_values(minPt1.x, minPt1.y, minPt1.z, maxPt1.x, maxPt1.y, maxPt1.z);
    mapsample_2.set_values(minPt2.x, minPt2.y, minPt2.z, maxPt2.x, maxPt2.y, maxPt2.z);
    get_overlap(mapsample_1, mapsample_2, submap_overlap);

/*
    std::cout << "Overlap details: " << "Submap 1 details: "<< "Submap 1 details: " << std::endl;
    std::cout << "Max x: " << submap_overlap.maxX << "   Max x: " << maxPt1.x << "   Max x: " << maxPt2.x << std::endl;
    std::cout << "Max y: " << submap_overlap.maxY << "   Max y: " << maxPt1.y << "   Max x: " << maxPt2.y << std::endl;
    std::cout << "Max z: " << submap_overlap.maxZ << "   Max z: " << maxPt1.z << "   Max x: " << maxPt2.z <<std::endl;
    std::cout << "Min x: " << submap_overlap.minX << "  Min x: " << minPt1.x << "  Min x: " << minPt2.x <<std::endl;
    std::cout << "Min y: " << submap_overlap.minY << "  Min y: " << minPt1.y << "  Min y: " << minPt2.y <<std::endl;
    std::cout << "Min z: " << submap_overlap.minZ << "  Min z: " << minPt1.z << "  Min z: " << minPt2.z <<std::endl;
    */
    if (grid_flag != 1) {
        keypoint_evaluation(submap_overlap, keypoints_src_visualize_temp,
                            keypoints_tgt_visualize_temp); //potherwise done later
    }

    //------------------------------------------------------------
    //------calc features-------
    //------------------------------------------------------------
    /*
    double GradientEstimationsetRadiusSearch_RIFT=5; //1000.001
    double RadiusSearch_RIFT=5.25;//100.05
    double NrDistanceBins_RIFT=4;
    double NrGradientBins_RIFT=8;
    double feature_radius_PFHRGB=5.25;//0.05
    float feature_radius_PFH=15.66;
    double setRadiusSearch_Shape_3DSC=5.25;//0.01;
    double setMinimalRadius_div_3DSC=5;
    double setPointDensityRadius_div3DSC=10;
    double RadiusSearch_USC=5.25;//0.01;
    double setMinimalRadius_div_USC=5;
    double setPointDensityRadius_div_USC=10;
    double LocalRadiusSearch_USC=5.25;
    double setRadiusSearch_SHOT=5.25;
    */
    // ----- RIFT feature -----
    if (feature_method == "RIFT" || feature_method == "ALL" || feature_method == "ALL_no_USC") {
        // ----- RIFT32 feature -----
        pcl::PointCloud<RIFT32>::Ptr descrs_RIFT32_1(new pcl::PointCloud<RIFT32>());
        descrs_RIFT32_1 = rift_extraction(src_downsampled_1, src_downsampled_1, normals_1,
                                          GradientEstimationsetRadiusSearch_RIFT, RadiusSearch_RIFT,
                                          NrDistanceBins_RIFT, NrGradientBins_RIFT); //cloud, keypoints, normals
        pcl::console::print_value("RIFT_1 returned features %d \n", descrs_RIFT32_1->size());

        pcl::PointCloud<RIFT32>::Ptr descrs_RIFT32_2(new pcl::PointCloud<RIFT32>());
        descrs_RIFT32_2 = rift_extraction(tgt_downsampled_2_transformed, tgt_downsampled_2_transformed, normals_2,
                                          GradientEstimationsetRadiusSearch_RIFT, RadiusSearch_RIFT,
                                          NrDistanceBins_RIFT, NrGradientBins_RIFT); //cloud, keypoints, normals
        pcl::console::print_value("RIFT_2 returned features %d \n", descrs_RIFT32_2->size());
    }
    if (feature_method == "3DSC" || feature_method == "ALL" || feature_method == "ALL_no_USC") {
        // ----- 3DSC feature -----
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descrs_ShapeContext1980_1(
                new pcl::PointCloud<pcl::ShapeContext1980>());
        descrs_ShapeContext1980_1 = sc_extraction(src_downsampled_1, keypoints_1, normals_1, setRadiusSearch_Shape_3DSC,
                                                  setMinimalRadius_div_3DSC,
                                                  setPointDensityRadius_div_3DSC);  // cloud, keypoints, normals  //works with normal_radius 100, but not with 10 or 1
        pcl::console::print_value("ShapeContext1980_1 returned features %d \n", descrs_ShapeContext1980_1->size());

        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descrs_ShapeContext1980_2(
                new pcl::PointCloud<pcl::ShapeContext1980>());
        descrs_ShapeContext1980_2 = sc_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                                  setRadiusSearch_Shape_3DSC, setMinimalRadius_div_3DSC,
                                                  setPointDensityRadius_div_3DSC);  // cloud, keypoints, normals  //works with normal_radius 100, but not with 10 or 1
        pcl::console::print_value("ShapeContext1980_2 returned features %d \n", descrs_ShapeContext1980_2->size());

        pcl::CorrespondencesPtr all_correspondences_ShapeContext1980(new pcl::Correspondences);
        pcl::CorrespondencesPtr good_correspondences_ShapeContext1980(new pcl::Correspondences);
        findCorrespondences_ShapeContext1980(descrs_ShapeContext1980_1, descrs_ShapeContext1980_2,
                                             *all_correspondences_ShapeContext1980);
        rejectBadCorrespondences(all_correspondences_ShapeContext1980, keypoints_1_withScale, keypoints_2_withScale,
                                 *good_correspondences_ShapeContext1980,RANSAC_Inlier_Threshold, RANSAC_Iterations);
        cout << "End of rejectBadCorrespondences! " << endl;
        cout << "ShapeContext1980 All correspondences size: " << all_correspondences_ShapeContext1980->size() << endl;
        cout << "ShapeContext1980 Good correspondences size: " << good_correspondences_ShapeContext1980->size() << endl;

        double counter_correctFmatch_3DSC, counter_wrongFmatch_3DSC, counter_validationR_3DSC;
        int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int *distance_bin_results_3DSC = correspondance_evaluation(good_correspondences_ShapeContext1980,
                                                                   keypoints_1_withScale, keypoints_2_withScale_eval,
                                                                   counter_correctFmatch_3DSC, counter_wrongFmatch_3DSC,
                                                                   counter_validationR_3DSC, validation_radius_in,
                                                                   validation_radius_out, arr);



        //feature_evaluation_and_recording(arr[0], counter_wrongFmatch, counter_validationR, src_file, tgt_file, noise_offset, noise_var, x_o, y_o, z_o, keypoint_method, jet_flag, jet_stacking_threshold, submap_overlap, distance_bin_results,src_original_size, tgt_original_size, keypoints_tgt_visualize_temp, keypoints_src_visualize_temp, grid_flag, measure_file_str, SalientRad_muliplier_ISS, NonMaxMultiplier_ISS, Threshold21_ISS, Threshold32_ISS, setMinNeighbors_ISS, time_elapsed_computation, num_good_correspondances, exact_match_rate, evaluation_match_rate, validation_radius_in, validation_radius_out)


    }
    if (feature_method == "USC" || feature_method == "ALL") {
        // ----- USC feature -----
        pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr descrs_UniqueShapeContext1960_1(
                new pcl::PointCloud<pcl::UniqueShapeContext1960>());
        descrs_UniqueShapeContext1960_1 = usc_extraction(src_downsampled_1, keypoints_1, normals_1, RadiusSearch_USC,
                                                         setMinimalRadius_div_USC, setPointDensityRadius_div_USC,
                                                         LocalRadiusSearch_USC);  // cloud, keypoints, normals  //works with normal_radius 100, but not with 10 or 1
        pcl::console::print_value("UniqueShapeContext1960_1 returned features %d \n",
                                  descrs_UniqueShapeContext1960_1->size());

        pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr descrs_UniqueShapeContext1960_2(
                new pcl::PointCloud<pcl::UniqueShapeContext1960>());
        descrs_UniqueShapeContext1960_2 = usc_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                                         RadiusSearch_USC, setMinimalRadius_div_USC,
                                                         setPointDensityRadius_div_USC,
                                                         LocalRadiusSearch_USC);  // cloud, keypoints, normals  //works with normal_radius 100, but not with 10 or 1
        pcl::console::print_value("UniqueShapeContext1960_2 returned features %d \n",
                                  descrs_UniqueShapeContext1960_2->size());

        pcl::CorrespondencesPtr all_correspondences_UniqueShapeContext1960(new pcl::Correspondences);
        pcl::CorrespondencesPtr good_correspondences_UniqueShapeContext1960(new pcl::Correspondences);
        cout << "before  findCorrespondences_UniqueShapeContext1960" << endl;
        findCorrespondences_UniqueShapeContext1960(descrs_UniqueShapeContext1960_1, descrs_UniqueShapeContext1960_2,
                                                   *all_correspondences_UniqueShapeContext1960);
        cout << "before  rejectBadCorrespondences" << endl;
        rejectBadCorrespondences(all_correspondences_UniqueShapeContext1960, keypoints_1_withScale,
                                 keypoints_2_withScale, *good_correspondences_UniqueShapeContext1960,RANSAC_Inlier_Threshold, RANSAC_Iterations);
        cout << "End of rejectBadCorrespondences! " << endl;
        cout << "UniqueShapeContext1960 All correspondences size: "
             << all_correspondences_UniqueShapeContext1960->size() << endl;
        cout << "UniqueShapeContext1960 Good correspondences size: "
             << good_correspondences_UniqueShapeContext1960->size() << endl;

        double counter_correctFmatch_USC, counter_wrongFmatch_USC, counter_validationR_USC;
        int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int *distance_bin_results_USC = correspondance_evaluation(good_correspondences_UniqueShapeContext1960,
                                                                  keypoints_1_withScale, keypoints_2_withScale_eval,
                                                                  counter_correctFmatch_USC, counter_wrongFmatch_USC,
                                                                  counter_validationR_USC, validation_radius_in,
                                                                  validation_radius_out, arr);

    }
    if (feature_method == "RSD" || feature_method == "ALL" || feature_method == "ALL_no_USC") {
        // ----- RSD feature -----
        pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descrs_rsd_1(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
        descrs_rsd_1 = rsd_extraction(src_downsampled_1, keypoints_1, normals_1, setRadiusSearch_RSD,
                                      setPlaneRadius_RSD); //cloud, keypoints, normals
        pcl::console::print_value("PrincipalRadiiRSD_1 (RSD) returned features %d \n", descrs_rsd_1->size());

        pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descrs_rsd_2(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
        descrs_rsd_2 = rsd_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2, setRadiusSearch_RSD,
                                      setPlaneRadius_RSD); //cloud, keypoints, normals
        pcl::console::print_value("PrincipalRadiiRSD_2 (RSD) returned features %d \n", descrs_rsd_2->size());

        pcl::CorrespondencesPtr all_correspondences_PrincipalRadiiRSD(new pcl::Correspondences);
        pcl::CorrespondencesPtr good_correspondences_PrincipalRadiiRSD(new pcl::Correspondences);
        findCorrespondences_PrincipalRadiiRSD(descrs_rsd_1, descrs_rsd_2, *all_correspondences_PrincipalRadiiRSD);
        rejectBadCorrespondences(all_correspondences_PrincipalRadiiRSD, keypoints_1_withScale, keypoints_2_withScale,
                                 *good_correspondences_PrincipalRadiiRSD,RANSAC_Inlier_Threshold, RANSAC_Iterations);
        cout << "End of rejectBadCorrespondences! " << endl;
        cout << "PrincipalRadiiRSD All correspondences size: " << all_correspondences_PrincipalRadiiRSD->size() << endl;
        cout << "PrincipalRadiiRSD Good correspondences size: " << good_correspondences_PrincipalRadiiRSD->size()
             << endl;

        double counter_correctFmatch_RSD, counter_wrongFmatch_RSD, counter_validationR_RSD;
        int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int *distance_bin_results_RSD = correspondance_evaluation(good_correspondences_PrincipalRadiiRSD,
                                                                  keypoints_1_withScale, keypoints_2_withScale_eval,
                                                                  counter_correctFmatch_RSD, counter_wrongFmatch_RSD,
                                                                  counter_validationR_RSD, validation_radius_in,
                                                                  validation_radius_out, arr);

    }
    if (feature_method == "SHOT" || feature_method == "ALL" || feature_method == "ALL_no_USC") {
        // ----- SHOT feature -----
        if (omp_on_flag == 0 || omp_on_flag == 2) {
            pcl::PointCloud<pcl::SHOT352>::Ptr descrs_shot_1(new pcl::PointCloud<pcl::SHOT352>());
            descrs_shot_1 = shot_extraction(src_downsampled_1, keypoints_1, normals_1,
                                            setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("SHOT_1 returned features %d \n", descrs_shot_1->size());

            pcl::PointCloud<pcl::SHOT352>::Ptr descrs_shot_2(new pcl::PointCloud<pcl::SHOT352>());
            descrs_shot_2 = shot_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                            setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("SHOT_2 returned features %d \n", descrs_shot_2->size());

            pcl::CorrespondencesPtr all_correspondences_SHOT352(new pcl::Correspondences);
            pcl::CorrespondencesPtr good_correspondences_SHOT352(new pcl::Correspondences);
            findCorrespondences_SHOT352(descrs_shot_1, descrs_shot_2, *all_correspondences_SHOT352);
            rejectBadCorrespondences(all_correspondences_SHOT352, keypoints_1_withScale, keypoints_2_withScale,
                                     *good_correspondences_SHOT352,RANSAC_Inlier_Threshold, RANSAC_Iterations);
            cout << "End of rejectBadCorrespondences! " << endl;
            cout << "SHOT352 All correspondences size: " << all_correspondences_SHOT352->size() << endl;
            cout << "SHOT352 Good correspondences size: " << good_correspondences_SHOT352->size() << endl;

            double counter_correctFmatch_SHOT, counter_wrongFmatch_SHOT, counter_validationR_SHOT;
            int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int *distance_bin_results_SHOT = correspondance_evaluation(good_correspondences_SHOT352,
                                                                       keypoints_1_withScale,
                                                                       keypoints_2_withScale_eval,
                                                                       counter_correctFmatch_SHOT,
                                                                       counter_wrongFmatch_SHOT,
                                                                       counter_validationR_SHOT, validation_radius_in,
                                                                       validation_radius_out, arr);

        }
        if (omp_on_flag == 1 || omp_on_flag == 2) {
            pcl::PointCloud<pcl::SHOT352>::Ptr descrs_shot_omp_1(new pcl::PointCloud<pcl::SHOT352>());
            descrs_shot_omp_1 = shot_extraction_omp(src_downsampled_1, keypoints_1, normals_1,
                                                    setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("SHOT_OMP_1 returned features %d \n", descrs_shot_omp_1->size());

            pcl::PointCloud<pcl::SHOT352>::Ptr descrs_shot_omp_2(new pcl::PointCloud<pcl::SHOT352>());
            descrs_shot_omp_2 = shot_extraction_omp(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                                    setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("SHOT_OMP_2 returned features %d \n", descrs_shot_omp_2->size());

            pcl::CorrespondencesPtr all_correspondences_SHOT352(new pcl::Correspondences);
            pcl::CorrespondencesPtr good_correspondences_SHOT352(new pcl::Correspondences);
            findCorrespondences_SHOT352(descrs_shot_omp_1, descrs_shot_omp_2, *all_correspondences_SHOT352);
            rejectBadCorrespondences(all_correspondences_SHOT352, keypoints_1_withScale, keypoints_2_withScale,
                                     *good_correspondences_SHOT352,RANSAC_Inlier_Threshold, RANSAC_Iterations);
            cout << "End of rejectBadCorrespondences! " << endl;
            cout << "SHOT352 All correspondences size: " << all_correspondences_SHOT352->size() << endl;
            cout << "SHOT352 Good correspondences size: " << good_correspondences_SHOT352->size() << endl;

            double counter_correctFmatch_SHOT, counter_wrongFmatch_SHOT, counter_validationR_SHOT;
            int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int *distance_bin_results_SHOT = correspondance_evaluation(good_correspondences_SHOT352,
                                                                       keypoints_1_withScale,
                                                                       keypoints_2_withScale_eval,
                                                                       counter_correctFmatch_SHOT,
                                                                       counter_wrongFmatch_SHOT,
                                                                       counter_validationR_SHOT, validation_radius_in,
                                                                       validation_radius_out, arr);

        }
    }
    if (feature_method == "CSHOT" || feature_method == "ALL" || feature_method == "ALL_no_USC") {
        if (omp_on_flag == 0 || omp_on_flag == 2) {
            pcl::PointCloud<pcl::SHOT1344>::Ptr descrs_cshot_1(new pcl::PointCloud<pcl::SHOT1344>());
            descrs_cshot_1 = cshot_extraction(src_downsampled_1, keypoints_1, normals_1,
                                              setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("CSHOT_1 returned features %d \n", descrs_cshot_1->size());

            pcl::PointCloud<pcl::SHOT1344>::Ptr descrs_cshot_2(new pcl::PointCloud<pcl::SHOT1344>());
            descrs_cshot_2 = cshot_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                              setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("CSHOT_2 returned features %d \n", descrs_cshot_2->size());

            pcl::CorrespondencesPtr all_correspondences_SHOT1344(new pcl::Correspondences);
            pcl::CorrespondencesPtr good_correspondences_SHOT1344(new pcl::Correspondences);
            findCorrespondences_SHOT1344(descrs_cshot_1, descrs_cshot_2, *all_correspondences_SHOT1344);
            rejectBadCorrespondences(all_correspondences_SHOT1344, keypoints_1_withScale, keypoints_2_withScale,
                                     *good_correspondences_SHOT1344,RANSAC_Inlier_Threshold, RANSAC_Iterations);
            cout << "End of rejectBadCorrespondences! " << endl;
            cout << "SHOT1344 All correspondences size: " << all_correspondences_SHOT1344->size() << endl;
            cout << "SHOT1344 Good correspondences size: " << good_correspondences_SHOT1344->size() << endl;

            double counter_correctFmatch_CSHOT, counter_wrongFmatch_CSHOT, counter_validationR_CSHOT;
            int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int *distance_bin_results_CSHOT = correspondance_evaluation(good_correspondences_SHOT1344,
                                                                        keypoints_1_withScale,
                                                                        keypoints_2_withScale_eval,
                                                                        counter_correctFmatch_CSHOT,
                                                                        counter_wrongFmatch_CSHOT,
                                                                        counter_validationR_CSHOT, validation_radius_in,
                                                                        validation_radius_out, arr);

        }
        if (omp_on_flag == 1 || omp_on_flag == 2) {
            pcl::PointCloud<pcl::SHOT1344>::Ptr descrs_cshot_omp_1(new pcl::PointCloud<pcl::SHOT1344>());
            descrs_cshot_omp_1 = cshot_extraction_omp(src_downsampled_1, keypoints_1, normals_1,
                                                      setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("CSHOT_OMP_1 returned features %d \n", descrs_cshot_omp_1->size());

            pcl::PointCloud<pcl::SHOT1344>::Ptr descrs_cshot_omp_2(new pcl::PointCloud<pcl::SHOT1344>());
            descrs_cshot_omp_2 = cshot_extraction_omp(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                                      setRadiusSearch_SHOT);//cloud, keypoints, normals
            pcl::console::print_value("CSHOT_OMP_2 returned features %d \n", descrs_cshot_omp_2->size());

            pcl::CorrespondencesPtr all_correspondences_SHOT1344(new pcl::Correspondences);
            pcl::CorrespondencesPtr good_correspondences_SHOT1344(new pcl::Correspondences);
            findCorrespondences_SHOT1344(descrs_cshot_omp_1, descrs_cshot_omp_2, *all_correspondences_SHOT1344);
            rejectBadCorrespondences(all_correspondences_SHOT1344, keypoints_1_withScale, keypoints_2_withScale,
                                     *good_correspondences_SHOT1344,RANSAC_Inlier_Threshold, RANSAC_Iterations);
            cout << "End of rejectBadCorrespondences! " << endl;
            cout << "SHOT1344 All correspondences size: " << all_correspondences_SHOT1344->size() << endl;
            cout << "SHOT1344 Good correspondences size: " << good_correspondences_SHOT1344->size() << endl;

            double counter_correctFmatch_CSHOT, counter_wrongFmatch_CSHOT, counter_validationR_CSHOT;
            int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int *distance_bin_results_CSHOT = correspondance_evaluation(good_correspondences_SHOT1344,
                                                                        keypoints_1_withScale,
                                                                        keypoints_2_withScale_eval,
                                                                        counter_correctFmatch_CSHOT,
                                                                        counter_wrongFmatch_CSHOT,
                                                                        counter_validationR_CSHOT, validation_radius_in,
                                                                        validation_radius_out, arr);

        }
    }
    if (feature_method == "FPFH" || feature_method == "ALL" || feature_method == "ALL_no_USC") {

        // ----- FPFH feature -----
        if (omp_on_flag == 0 || omp_on_flag == 2) {
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr descrs_fpfh_1(new pcl::PointCloud<pcl::FPFHSignature33>());
            descrs_fpfh_1 = fpfh_extraction(src_downsampled_1, keypoints_1, normals_1,
                                            searchRadius_FPFH);//cloud, keypoints, normals
            pcl::console::print_value("FPFH_1 returned features %d \n", descrs_fpfh_1->size());

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr descrs_fpfh_2(new pcl::PointCloud<pcl::FPFHSignature33>());
            descrs_fpfh_2 = fpfh_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                            searchRadius_FPFH);//cloud, keypoints, normals
            pcl::console::print_value("FPFH_2 returned features %d \n", descrs_fpfh_2->size());

            pcl::CorrespondencesPtr all_correspondences_FPFHSignature33(new pcl::Correspondences);
            pcl::CorrespondencesPtr good_correspondences_FPFHSignature33(new pcl::Correspondences);
            findCorrespondences_FPFHSignature33(descrs_fpfh_1, descrs_fpfh_2,*all_correspondences_FPFHSignature33);
            //findCorrespondences_FPFHSignature33(descrs_fpfh_1, descrs_fpfh_2,*all_correspondences_FPFHSignature33);
            rejectBadCorrespondences(all_correspondences_FPFHSignature33, keypoints_1_withScale, keypoints_2_withScale,*good_correspondences_FPFHSignature33,RANSAC_Inlier_Threshold, RANSAC_Iterations);
            cout << "End of rejectBadCorrespondences! " << endl;
            cout << "FPFHSignature33 All correspondences size: " << all_correspondences_FPFHSignature33->size() << endl;
            cout << "FPFHSignature33 Good correspondences size: " << good_correspondences_FPFHSignature33->size()
                 << endl;

            double counter_correctFmatch_FPFH, counter_wrongFmatch_FPFH, counter_validationR_FPFH;
            int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int *distance_bin_results_FPFH = correspondance_evaluation(good_correspondences_FPFHSignature33,
                                                                       keypoints_1_withScale,
                                                                       keypoints_2_withScale_eval,
                                                                       counter_correctFmatch_FPFH,
                                                                       counter_wrongFmatch_FPFH,
                                                                       counter_validationR_FPFH, validation_radius_in,
                                                                       validation_radius_out, arr);
        }
        if (omp_on_flag == 1 || omp_on_flag == 2) {
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr descrs_fpfh_omp_1(new pcl::PointCloud<pcl::FPFHSignature33>());
            descrs_fpfh_omp_1 = fpfh_extraction_omp(src_downsampled_1, keypoints_1, normals_1, searchRadius_FPFH);//cloud, keypoints, normals
            pcl::console::print_value("FPFH_OMP_1 returned features %d \n", descrs_fpfh_omp_1->size());

            pcl::PointCloud<pcl::FPFHSignature33>::Ptr descrs_fpfh_omp_2(new pcl::PointCloud<pcl::FPFHSignature33>());
            descrs_fpfh_omp_2 = fpfh_extraction_omp(tgt_downsampled_2_transformed, keypoints_2, normals_2, searchRadius_FPFH);//cloud, keypoints, normals
            pcl::console::print_value("FPFH_OMP_2 returned features %d \n", descrs_fpfh_omp_2->size());

            pcl::CorrespondencesPtr all_correspondences_FPFHSignature33_omp(new pcl::Correspondences);
            pcl::CorrespondencesPtr good_correspondences_FPFHSignature33_omp(new pcl::Correspondences);
            //findCorrespondences_FPFHSignature33(src_downsampled_1, descrs_fpfh_omp_2,*all_correspondences_FPFHSignature33_omp);
            findCorrespondences_FPFHSignature33(descrs_fpfh_omp_1, descrs_fpfh_omp_2,*all_correspondences_FPFHSignature33_omp);
            rejectBadCorrespondences(all_correspondences_FPFHSignature33_omp, keypoints_1_withScale, keypoints_2_withScale,
                                     *good_correspondences_FPFHSignature33_omp,RANSAC_Inlier_Threshold, RANSAC_Iterations);
            cout << "End of rejectBadCorrespondences! " << endl;
            cout << "FPFHSignature33 All correspondences size: " << all_correspondences_FPFHSignature33_omp->size() << endl;
            cout << "FPFHSignature33 Good correspondences size: " << good_correspondences_FPFHSignature33_omp->size()
                 << endl;

            double counter_correctFmatch_FPFH, counter_wrongFmatch_FPFH, counter_validationR_FPFH;
            int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int *distance_bin_results_FPFH = correspondance_evaluation(good_correspondences_FPFHSignature33_omp,
                                                                       keypoints_1_withScale,
                                                                       keypoints_2_withScale_eval,
                                                                       counter_correctFmatch_FPFH,
                                                                       counter_wrongFmatch_FPFH,
                                                                       counter_validationR_FPFH, validation_radius_in,
                                                                       validation_radius_out, arr);
        }


    }
    if (feature_method == "PFH" || feature_method == "ALL" || feature_method == "ALL_no_USC") {

        // ----- PFH feature -----
        pcl::PointCloud<pcl::PFHSignature125>::Ptr descrs_pfh_1(new pcl::PointCloud<pcl::PFHSignature125>());
        descrs_pfh_1 = pfh_extraction(src_downsampled_1, keypoints_1, normals_1,
                                      feature_radius_PFH);//cloud, keypoints, normals
        pcl::console::print_value("PFH_1 returned features %d \n", descrs_pfh_1->size());

        pcl::PointCloud<pcl::PFHSignature125>::Ptr descrs_pfh_2(new pcl::PointCloud<pcl::PFHSignature125>());
        descrs_pfh_2 = pfh_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                      feature_radius_PFH);//cloud, keypoints, normals
        pcl::console::print_value("PFH_2 returned features %d \n", descrs_pfh_2->size());

        // Find correspondences between keypoints in PFH space
        pcl::CorrespondencesPtr all_correspondences_PFH(new pcl::Correspondences);
        findCorrespondences_PFH_newshort(descrs_pfh_1, descrs_pfh_2, *all_correspondences_PFH);
        pcl::CorrespondencesPtr good_correspondences_PFH(new pcl::Correspondences);
        rejectBadCorrespondences(all_correspondences_PFH, keypoints_1_withScale, keypoints_2_withScale,
                                 *good_correspondences_PFH,RANSAC_Inlier_Threshold, RANSAC_Iterations);
        cout << "PFH All correspondences size: " << all_correspondences_PFH->size() << endl;
        cout << "PFH Good correspondences size: " << good_correspondences_PFH->size() << endl;

        double counter_correctFmatch_PFH, counter_wrongFmatch_PFH, counter_validationR_PFH;
        int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int *distance_bin_results_PFH = correspondance_evaluation(good_correspondences_PFH, keypoints_1_withScale,
                                                                  keypoints_2_withScale_eval, counter_correctFmatch_PFH,
                                                                  counter_wrongFmatch_PFH, counter_validationR_PFH,
                                                                  validation_radius_in, validation_radius_out, arr);

    }
    if (feature_method == "PFHRGB" || feature_method == "ALL" || feature_method == "ALL_no_USC") {
        // ----- PFHRGB feature -----
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descrs_pfhrgb_1(new pcl::PointCloud<pcl::PFHRGBSignature250>());
        descrs_pfhrgb_1 = pfhrgb_extraction(src_downsampled_1, keypoints_1, normals_1,
                                            feature_radius_PFHRGB);//cloud, keypoints, normals
        pcl::console::print_value("PFHRGB_1 returned features %d \n", descrs_pfhrgb_1->size());

        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descrs_pfhrgb_2(new pcl::PointCloud<pcl::PFHRGBSignature250>());
        descrs_pfhrgb_2 = pfhrgb_extraction(tgt_downsampled_2_transformed, keypoints_2, normals_2,
                                            feature_radius_PFHRGB);//cloud, keypoints, normals
        pcl::console::print_value("PFHRGB_2 returned features %d \n", descrs_pfhrgb_2->size());

        // Find correspondences between keypoints in PFHRGB space
        pcl::CorrespondencesPtr all_correspondences_PFHRGB(new pcl::Correspondences);
        pcl::CorrespondencesPtr good_correspondences_PFHRGB(new pcl::Correspondences);
        findCorrespondences_PFHRGB(descrs_pfhrgb_1, descrs_pfhrgb_1, *all_correspondences_PFHRGB);
        //cout << "PFHRGB All correspondences size: " << all_correspondences_PFHRGB->size() << endl;
        rejectBadCorrespondences(all_correspondences_PFHRGB, keypoints_1_withScale, keypoints_2_withScale,
                                 *good_correspondences_PFHRGB,RANSAC_Inlier_Threshold, RANSAC_Iterations);
        cout << "End of rejectBadCorrespondences! " << endl;
        cout << "PFHRGB All correspondences size: " << all_correspondences_PFHRGB->size() << endl;
        cout << "PFHRGB Good correspondences size: " << good_correspondences_PFHRGB->size() << endl;

        double counter_correctFmatch_PFHRGB, counter_wrongFmatch_PFHRGB, counter_validationR_PFHRGB;
        int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int *distance_bin_results_PFHRGB = correspondance_evaluation(good_correspondences_PFHRGB, keypoints_1_withScale,
                                                                     keypoints_2_withScale_eval,
                                                                     counter_correctFmatch_PFHRGB,
                                                                     counter_wrongFmatch_PFHRGB,
                                                                     counter_validationR_PFHRGB, validation_radius_in,
                                                                     validation_radius_out, arr);

    }
    if (feature_method == "NEW_PFHRGB" || feature_method == "ALL" || feature_method == "ALL_no_USC") {
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr new_pfdrgb_1(new pcl::PointCloud<pcl::PFHRGBSignature250>);
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr new_pfdrgb_2(new pcl::PointCloud<pcl::PFHRGBSignature250>);

        //feature_radius_PFH=5.25;
        compute_PFHRGB_features(src_downsampled_1, normals_1, keypoints_1_withScale, new_pfdrgb_1,
                                feature_radius_PFHRGB);
        compute_PFHRGB_features(tgt_downsampled_2_transformed, normals_2, keypoints_2_withScale, new_pfdrgb_2,
                                feature_radius_PFHRGB);
        pcl::console::print_value("NEW_PFHRGB_1 returned features %d \n", new_pfdrgb_1->size());
        pcl::console::print_value("NEW_PFHRGB_2 returned features %d \n", new_pfdrgb_2->size());

        // PFHRGB Estimation
        pcl::CorrespondencesPtr all_correspondences_PFHRGB_new(new pcl::Correspondences);
        pcl::CorrespondencesPtr good_correspondences_PFHRGB_new(new pcl::Correspondences);
        findCorrespondences_PFHRGB(new_pfdrgb_1, new_pfdrgb_2, *all_correspondences_PFHRGB_new);
        rejectBadCorrespondences(all_correspondences_PFHRGB_new, keypoints_1_withScale, keypoints_2_withScale,
                                 *good_correspondences_PFHRGB_new,RANSAC_Inlier_Threshold, RANSAC_Iterations);
        cout << "End of rejectBadCorrespondences! " << endl;
        cout << "NEW PFHRGB All correspondences size: " << all_correspondences_PFHRGB_new->size() << endl;
        cout << "NEW PFHRGB Good correspondences size: " << good_correspondences_PFHRGB_new->size() << endl;

        double counter_correctFmatch_NEW_PFHRGB, counter_wrongFmatch_NEW_PFHRGB, counter_validationR_NEW_PFHRGB;
        int arr[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int *distance_bin_results_NEW_PFHRGB = correspondance_evaluation(good_correspondences_PFHRGB_new,
                                                                         keypoints_1_withScale,
                                                                         keypoints_2_withScale_eval,
                                                                         counter_correctFmatch_NEW_PFHRGB,
                                                                         counter_wrongFmatch_NEW_PFHRGB,
                                                                         counter_validationR_NEW_PFHRGB,
                                                                         validation_radius_in, validation_radius_out,
                                                                         arr);

    }
}


        //std::cout<<"jet_stacking_threshold: " << jet_stacking_threshold<<std::endl;

        //add perormance results
        //out <<time_elapsed_computation<< "," <<good_correspondences->size()<< "," <<counter_correctFmatch<< "," <<counter_validationR<< "," <<exact_match_rate<< "," <<evaluation_match_rate<< "," <<validation_radius_in<< "," <<validation_radius_out<< ",";
        //out << distance_bin_results[0];
        //for(int counter=1;counter<9;counter++) {out << ","<< distance_bin_results[counter];}
        //out << ","<< distance_bin_results[9]<< std::endl;

        /*
        out << "Src points: |" << src_original->points.size();
        out << "Tgt points: |" << tgt_transformed->points.size() << endl;
        out << "validation_radius_in: |" << validation_radius_in << std::endl;
        out << "validation_radius_in: |"  << validation_radius_out << std::endl;
        out << "noise offest: |" << noise_offset << std::endl;
        out << "noise stdv: |" << noise_var << std::endl;
        out << "x_o: |" << x_o << std::endl;___,
        out << "y_o: |" << y_o << std::endl;
        out << "z_o: |" << z_o << std::endl;
        out << "rot_alpha: |" << rot_alpha << std::endl;
        out << "rot_betha: |" << rot_betha << std::endl;
        out << "rot_gamma: |" << rot_gamma << std::endl;
        if (result.count("method"))    {
            out << "keypoint method |" << result["method"].as<std::string>() << std::endl;
        }else{
            out << "keypoint method |" << "SWIFT" << std::endl;
        }
        out << "Stacked height threshold: |" << jet_stacking_threshold << std::endl;
        out << "---"  << std::endl;
        */




