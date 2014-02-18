/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/vfh_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/cvfh_estimator.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include "soc_msg_and_serv/segment_and_classify.h"
#include "strands_qsr_msgs/ObjectClassification.h"
#include "segmenter.h"
#include <pcl/apps/dominant_plane_segmentation.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <stdlib.h>
#include <stdio.h>

#include <tf/transform_listener.h>

#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#define SOC_VISUALIZE

class ShapeClassifier
{
  private:
    typedef pcl::PointXYZ PointT;
    std::string models_dir_;
    std::string training_dir_;
    std::string desc_name_;
    int NN_;
    float chop_at_z_;
    tf::TransformListener _tf_listener;

    std::vector<std::string> text_3d_;

    boost::shared_ptr<pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, PointT, pcl::ESFSignature640> > classifier_;
    ros::ServiceServer segment_and_classify_service_;
    ros::NodeHandle n_;
#ifdef SOC_VISUALIZE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
#endif


  ////////////////////////////////////////////
  // TF Stuff


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
transformPointCloud (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,
                     sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }

  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");

  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i)
  {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;
    
    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
    {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr))  // Invalid point
      {
        pt_out = pt;
      }
      else  // max range point
      {
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    }
    else
    {
      pt_out = transform * pt;
    }

    if (max_range_point)
    {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));
  
    
    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1)
  {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i)
    {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
  double mv[12];
  bt.getBasis ().getOpenGLSubMatrix (mv);

  tf::Vector3 origin = bt.getOrigin ();

  out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];
                                                                     
  out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  out_mat (0, 3) = origin.x ();
  out_mat (1, 3) = origin.y ();
  out_mat (2, 3) = origin.z ();
}

  ///////////////////////////////////////////

    bool segmentAndClassify(soc_msg_and_serv::segment_and_classify::Request & req,
                               soc_msg_and_serv::segment_and_classify::Response & response)
    {

      // Get the TF transform
      tf::StampedTransform transform;
      try
        {
          _tf_listener.waitForTransform("/table", req.cloud.header.frame_id, ros::Time(0), ros::Duration(10.0) );
          _tf_listener.lookupTransform ("/table", req.cloud.header.frame_id, ros::Time(0), transform);
          ROS_INFO("Got the transform to the table....");
        }
      catch (tf::LookupException &e)
        {
          ROS_ERROR ("%s", e.what ());
        }
      catch (tf::ExtrapolationException &e)
        {
          ROS_ERROR ("%s", e.what ());
        } 
      

      pcl::PointCloud<PointT>::Ptr frame (new pcl::PointCloud<PointT>);


      pcl::fromROSMsg (req.cloud, *frame);

      /*std::vector<pcl::PointIndices> indices;
      Eigen::Vector4f table_plane;
      doSegmentation<PointT>(frame, indices, table_plane, chop_at_z_);*/

      //float tolerance = 0.005f;
      float tolerance = 0.02f;
      pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
      dps.setInputCloud (frame);
      dps.setMaxZBounds (chop_at_z_);
      dps.setObjectMinHeight (tolerance);
      dps.setMinClusterSize (500);
      dps.setWSize (9);
      dps.setDistanceBetweenClusters (0.1f);
      std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
      std::vector<pcl::PointIndices> indices;
      dps.setDownsamplingSize (0.01f);
      dps.compute_fast (clusters);
      dps.getIndicesClusters(indices);
      Eigen::Vector4f table_plane;
      dps.getTableCoefficients (table_plane);

      // Convert the TF transform to Eigen format
      Eigen::Matrix4f eigen_transform;
      transformAsMatrix (transform, eigen_transform);
      sensor_msgs::PointCloud2 pc_table;
      transformPointCloud (eigen_transform, req.cloud, pc_table);
      ROS_INFO("Transformed to the table....");

      pc_table.header.frame_id = "/table";
      pcl::fromROSMsg (pc_table, *frame);


#ifdef SOC_VISUALIZE
      //vis_->spin();
      for(size_t i=0; i < text_3d_.size(); i++)
      {
        vis_->removeText3D(text_3d_[i]);
      }
      text_3d_.clear();
      vis_->removeAllShapes();
      vis_->removeAllPointClouds();
      vis_->addPointCloud(frame, "scene_cloud"); //TODO: comment
      vis_->addCoordinateSystem(0.2f);

      //show table plane
      std::vector<int> plane_indices;
      for(size_t i=0; i < frame->points.size(); i++)
      {
        Eigen::Vector3f xyz_p = frame->points[i].getVector3fMap ();


        if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
          continue;

        float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

        if (val <= tolerance && val >= -tolerance)
        {
          plane_indices.push_back(i);
        }
      }

      pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*frame, plane_indices, *plane);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> random_handler (plane, 0, 255, 0);
      vis_->addPointCloud<PointT> (plane, random_handler, "table plane");
      vis_->spinOnce();
#endif

      classifier_->setInputCloud(frame);

      for(size_t i=0; i < indices.size(); i++)
      {
        std::cout << "==========" << std::endl;
        std::cout << "CLUSTER " << i << std::endl;
        std::cout << "==========" << std::endl;

        strands_qsr_msgs::ObjectClassification oc;
        response.classification.push_back(oc);


        pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*frame, indices[i], *cluster);

        sensor_msgs::PointCloud2  pc2;
        pcl::toROSMsg (*cluster, pc2);
        response.cloud.push_back(pc2);

#ifdef SOC_VISUALIZE
        std::stringstream cluster_name;
        cluster_name << "cluster_" << i;
        pcl::visualization::PointCloudColorHandlerRandom<PointT> random_handler (cluster);
        vis_->addPointCloud<PointT> (cluster, random_handler, cluster_name.str ());
        


#endif
        classifier_->setIndices(indices[i].indices);
        classifier_->classify ();

        std::vector < std::string > categories;
        std::vector<float> conf;
        classifier_->getCategory (categories);
        classifier_->getConfidence (conf);


        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*frame, indices[i].indices, centroid);

        
#ifdef SOC_VISUALIZE

        float text_scale = 0.015f;
        float dist_ = 0.03f;

        pcl::PointXYZ pos2;
        pos2.x = centroid[0] + table_plane[0] * static_cast<float> (- 1) * dist_;
        pos2.y = centroid[1] + table_plane[1] * static_cast<float> (- 1) * dist_;;
        pos2.z = centroid[2] + table_plane[2] * static_cast<float> (- 1) * dist_;;
        
        std::stringstream cluster_text;
        cluster_text << "C_" << i;
        text_3d_.push_back(cluster_text.str());
        vis_->addText3D (cluster_text.str(), pos2, text_scale * 2, 1, 0, 1, cluster_text.str(), 0);

        /*
        for (size_t kk = 0; kk < categories.size (); kk++)
        {
          pcl::PointXYZ pos;
          pos.x = centroid[0] + table_plane[0] * static_cast<float> (kk + 1) * dist_;
          pos.y = centroid[1] + table_plane[1] * static_cast<float> (kk + 1) * dist_;
          pos.z = centroid[2] + table_plane[2] * static_cast<float> (kk + 1) * dist_;

          std::ostringstream prob_str;
          prob_str.precision (2);
          prob_str << categories[kk] << " [" << conf[kk] << "]";

          std::stringstream cluster_text;
          cluster_text << "cluster_" << i << "_" << kk << "_text";
          text_3d_.push_back(cluster_text.str());
          vis_->addText3D (prob_str.str(), pos, text_scale, 1, 0, 1, cluster_text.str (), 0);
          
        }
        */
#endif
        
        std::cout << "CENTROID: (" << centroid[0] << "," << centroid[1] << "," << centroid[2] << ")" << std::endl;
        std::cout << "TABLE: (" << table_plane[0] << "," << table_plane[1] << "," << table_plane[2] << "," << table_plane[3] << " )" << std::endl;
        
        geometry_msgs::Point32 cent;
        cent.x = centroid[0];
        cent.y = centroid[1];
        cent.z = centroid[2];
        response.centroid.push_back(cent);
        
        Eigen::Vector4f min; 
        Eigen::Vector4f max;   
        pcl::getMinMax3D (*frame, indices[i].indices, min, max); 
        
        strands_qsr_msgs::BBox bbox;
        geometry_msgs::Point32 pt;
        pt.x = min[0]; pt.y = min[1]; pt.z = min[2]; bbox.point.push_back(pt);
        pt.x = min[0]; pt.y = min[1]; pt.z = max[2]; bbox.point.push_back(pt);
        pt.x = min[0]; pt.y = max[1]; pt.z = min[2]; bbox.point.push_back(pt);
        pt.x = min[0]; pt.y = max[1]; pt.z = max[2]; bbox.point.push_back(pt);
        pt.x = max[0]; pt.y = min[1]; pt.z = min[2]; bbox.point.push_back(pt);
        pt.x = max[0]; pt.y = min[1]; pt.z = max[2]; bbox.point.push_back(pt);
        pt.x = max[0]; pt.y = max[1]; pt.z = min[2]; bbox.point.push_back(pt);
        pt.x = max[0]; pt.y = max[1]; pt.z = max[2]; bbox.point.push_back(pt);
        response.bbox.push_back(bbox);

        if(categories.size() > 0)
        {
          //at least 1 category
          std::vector< std::pair<float, std::string> > conf_categories_map_;
          for (size_t kk = 0; kk < categories.size (); kk++)
            {
              conf_categories_map_.push_back(std::make_pair(conf[kk], categories[kk]));
            }
          
          std::sort (conf_categories_map_.begin (), conf_categories_map_.end (),
                     boost::bind (&std::pair<float, std::string>::first, _1) > boost::bind (&std::pair<float, std::string>::first, _2));
          
          std::string cls ("cluster_");
          response.classification[i].object_id = cls.append(boost::lexical_cast<std::string>(i));
          
          for (size_t kk = 0; kk < categories.size (); kk++)
            {
              
              std::cout << conf_categories_map_[kk].first << " " << conf_categories_map_[kk].second << std::endl;
            
              std::string st = conf_categories_map_[kk].second.substr(0, conf_categories_map_[kk].second.size()-1);
              response.classification[i].type.push_back(st);
              response.classification[i].confidence.push_back(conf_categories_map_[kk].first);
            }
        }
      }

#ifdef SOC_VISUALIZE
      vis_->spin();
#endif

      return true;
    }
  public:
    ShapeClassifier()
    {
      //default values
      desc_name_ = "esf";
      NN_ = 50;
      chop_at_z_ = 1.f;
#ifdef SOC_VISUALIZE
      vis_.reset(new pcl::visualization::PCLVisualizer("classifier visualization"));
#endif
    }

    void initialize(int argc, char ** argv)
    {

      pcl::console::parse_argument (argc, argv, "-models_dir", models_dir_);
      pcl::console::parse_argument (argc, argv, "-training_dir", training_dir_);
      pcl::console::parse_argument (argc, argv, "-descriptor_name", desc_name_);
      pcl::console::parse_argument (argc, argv, "-nn", NN_);
      pcl::console::parse_argument (argc, argv, "-chop_z", chop_at_z_);

      if(models_dir_.compare("") == 0)
      {
        PCL_ERROR("Set -models_dir option in the command line, ABORTING");
        return;
      }

      if(training_dir_.compare("") == 0)
      {
        PCL_ERROR("Set -training_dir option in the command line, ABORTING");
        return;
      }

      boost::shared_ptr<pcl::rec_3d_framework::MeshSource<PointT> > mesh_source (new pcl::rec_3d_framework::MeshSource<PointT>);
      mesh_source->setPath (models_dir_);
      mesh_source->setResolution (150);
      mesh_source->setTesselationLevel (0);
      mesh_source->setViewAngle (57.f);
      mesh_source->setRadiusSphere (1.f);
      mesh_source->setModelScale (1.f);
      mesh_source->generate (training_dir_);

      boost::shared_ptr<pcl::rec_3d_framework::Source<PointT> > cast_source;
      cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<PointT> > (mesh_source);

      boost::shared_ptr<pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640> > estimator;
      estimator.reset (new pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640>);

      boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<PointT, pcl::ESFSignature640> > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640> > (estimator);

      classifier_.reset(new pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, PointT, pcl::ESFSignature640>);
      classifier_->setDataSource (cast_source);
      classifier_->setTrainingDir (training_dir_);
      classifier_->setDescriptorName (desc_name_);
      classifier_->setFeatureEstimator (cast_estimator);
      classifier_->setNN (NN_);
      classifier_->initialize (false);

      segment_and_classify_service_ = n_.advertiseService("segment_and_classify", &ShapeClassifier::segmentAndClassify, this);

      ros::spin();
    }
};

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "master_demo");

  ShapeClassifier m;
  m.initialize (argc, argv);

  return 0;
}
