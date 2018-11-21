/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  point_cloud2_base_assembler.h
 *
 *  Created on: 01.07.2015
 *   Authors: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef LASER_ASSEMBLER_POINT_CLOUD2_BASE_ASSEMBLER_H
#define LASER_ASSEMBLER_POINT_CLOUD2_BASE_ASSEMBLER_H

//! \author Sebastian Puetz (spuetz@uni-osnabrueck.de)

#include "base_assembler.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_field_conversion.h"

// Service
#include "laser_assembler/AssembleScans2.h"

namespace laser_assembler
{

  /**
   * \brief Maintains a history of point clouds and generates an aggregate point cloud upon request
   */
  template<class T>
    class PointCloud2BaseAssembler : public BaseAssembler<T, sensor_msgs::PointCloud2>
  {
    public:
      PointCloud2BaseAssembler(const std::string& max_size_param_name);
      ~PointCloud2BaseAssembler() ;

      /** \brief Returns the number of points in the current scan
       * \param scan The scan for for which we want to know the number of points
       * \return the number of points in scan
       */
      virtual unsigned int GetPointsInScan(const T& scan) = 0 ;

      /** \brief Converts the current scan into a cloud in the specified fixed frame
       *
       * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions. These exceptions are caught by
       * BaseAssembler, and will be counted for diagnostic information
       * \param fixed_frame_id The name of the frame in which we want cloud_out to be in
       * \param scan_in The scan that we want to convert
       * \param cloud_out The result of transforming scan_in into a cloud in frame fixed_frame_id
       */
      virtual void Convert(const std::string& fixed_frame_id, const T& scan_in, sensor_msgs::PointCloud2& cloud_out) = 0 ;

    private:
      // ROS Input/Ouptut Handling
      ros::ServiceServer build_cloud_server2_;
      ros::ServiceServer assemble_scans_server2_;

      //! \brief Service Callback function called whenever we need to build a cloud
      bool buildCloud2(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;
      bool assembleScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;
      bool assembleScans2Organized(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;
      bool assembleScans2Unorganized(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;


      sensor_msgs::PointField* getPointFieldByName(std::vector<sensor_msgs::PointField>& fields, const std::string& name);

      /** \brief Fills-up index holes in an organized point cloud
       *
       * \param data_count The current index of the bytebuffer of the point cloud 2, the value will be modified
       * \param pc The output PointCloud2 object 
       * \param index the current scan ray index which will be insertet, the value will be modified
       * \param previous_index the previous read scan ray index
       * \param point_step The step of one combined field in bytes in the buffer
       * \param fields A vector of all fields
       * \param idx_offset The byte offset of one scan ray index
       * \param idx_datatype The type of one scan ray index 
       */
      inline void fillUpIndexHoles(unsigned int& data_count,
          sensor_msgs::PointCloud2 &pc,
          int& previous_index,
          int index,
          const unsigned int point_step,
          const std::vector<sensor_msgs::PointField>& fields,
          const unsigned int idx_offset,
          const unsigned int idx_datatype);

      /** \brief checks the equality of two scans headers
       *
       * \param first_scan The first scan to assemble
       * \param current_scan The current scan to assemble
       * \return true, if the two scans are equal
       */
      bool hasEqualScanHeader(sensor_msgs::PointCloud2 &first_scan, sensor_msgs::PointCloud2 &current_scan);

      //! \enables the organized mode. 
      bool organized;
  } ;

  template <class T>
    PointCloud2BaseAssembler<T>::PointCloud2BaseAssembler(const std::string& max_size_param_name)
    : BaseAssembler<T, sensor_msgs::PointCloud2>(max_size_param_name)
    {
      this->private_ns_.param("organized", organized, false);

      // ***** Start Services *****
      std::string build_cloud2_service_name = ros::names::resolve("build_cloud2");
      std::string assemble_scans2_service_name = ros::names::resolve("assemble_scans2");
      build_cloud_server2_    = BaseAssembler<T, sensor_msgs::PointCloud2>::n_.advertiseService("build_cloud2", &PointCloud2BaseAssembler<T>::buildCloud2,    this);
      assemble_scans_server2_ = BaseAssembler<T, sensor_msgs::PointCloud2>::n_.advertiseService("assemble_scans2", &PointCloud2BaseAssembler<T>::assembleScans2, this); 
    }

  template <class T>
    PointCloud2BaseAssembler<T>::~PointCloud2BaseAssembler()
    {
    }

  template <class T>
    bool PointCloud2BaseAssembler<T>::hasEqualScanHeader(sensor_msgs::PointCloud2 &first_scan, sensor_msgs::PointCloud2 &current_scan){
      bool malformed = false;
      for (unsigned int chan_ind = 0; chan_ind < first_scan.fields.size(); chan_ind++){
        std::string field_name = first_scan.fields[chan_ind].name;
        sensor_msgs::PointField* field = getPointFieldByName(first_scan.fields, field_name);

        if (!field){
          ROS_FATAL("field for name \"%s\", existing in the first scan, not found in the other scan!", field_name.c_str());
          return false;
        }
        if (field->offset != first_scan.fields[chan_ind].offset){
          ROS_FATAL("Field \"%s\" has not the same offset as the first scan!", field_name.c_str());
          malformed = true;
        }
        if (field->datatype != first_scan.fields[chan_ind].datatype){
          ROS_FATAL("Field \"%s\" has not the same datatype as the first scan!", field_name.c_str());
          malformed = true;
        }
        if (field->count != first_scan.fields[chan_ind].count){
          ROS_FATAL("Field \"%s\" has not the same number of elements as the first scan!", field_name.c_str());
          malformed = true;
        }
      }
      return !malformed;
    }

  template <class T>
    bool PointCloud2BaseAssembler<T>::buildCloud2(AssembleScans2::Request& req, AssembleScans2::Response& resp)
    {
      ROS_WARN("Service 'build_cloud' is deprecated. Call 'assemble_scans' instead");
      return assembleScans2(req, resp);
    }

  template <class T>
    sensor_msgs::PointField* PointCloud2BaseAssembler<T>::getPointFieldByName(std::vector<sensor_msgs::PointField>& fields, const std::string& name){
      for(unsigned int i=0; i<fields.size(); i++){
        if(fields[i].name.compare(name) == 0){
          return &fields[i];
        }
      }
      return 0; 
    }
  template <class T>
    void PointCloud2BaseAssembler<T>::fillUpIndexHoles(unsigned int& data_count, sensor_msgs::PointCloud2 &pc, int& previous_index, int index, const unsigned int point_step, const std::vector<sensor_msgs::PointField>& fields, const unsigned int idx_offset, const unsigned int idx_type)
    {
      
      // fill up for not existing indices
      while(previous_index + this->downsample_factor_ < index){
        for (unsigned int chan_ind = 0; chan_ind < fields.size(); chan_ind++)
        {
          const sensor_msgs::PointField field = fields[chan_ind];
          switch(field.datatype){
            case sensor_msgs::PointField::FLOAT32 :
              // write NaN values for float datatypes
              sensor_msgs::writePointCloud2BufferValue<sensor_msgs::PointField::FLOAT32, float>(
                &(pc.data[data_count+field.offset]), nanf("")) ;
              break ;
            case sensor_msgs::PointField::FLOAT64 :
              // write NaN values for double datatypes
              sensor_msgs::writePointCloud2BufferValue<sensor_msgs::PointField::FLOAT64, double>(
                &(pc.data[data_count+field.offset]), nan("")) ;
              break ;
            default:
              // write zero values for other datatypes
              sensor_msgs::writePointCloud2BufferValue<int>(
                &(pc.data[data_count+field.offset]), field.datatype, 0) ;
              break;
          }
        }
        previous_index += this->downsample_factor_ ;
        // write index value
        sensor_msgs::writePointCloud2BufferValue<unsigned int>(
          &(pc.data[data_count+idx_offset]), idx_type, previous_index);
        data_count+=point_step;
      }
    }

  template <class T>
    bool PointCloud2BaseAssembler<T>::assembleScans2Organized(AssembleScans2::Request& req, AssembleScans2::Response& resp)
    {
      this->scan_hist_mutex_.lock();  
      // Determine where in our history we actually are
      unsigned int i = 0 ;

      // Find the beginning of the request. Probably should be a search
      while ( i < BaseAssembler<T, sensor_msgs::PointCloud2>::scan_hist_.size() &&                                                    // Don't go past end of deque
          BaseAssembler<T, sensor_msgs::PointCloud2>::scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
      {
        i++ ;
      }
      unsigned int start_index = i ;
      sensor_msgs::PointCloud2& first_scan = this->scan_hist_[start_index];
      const unsigned int point_step = first_scan.point_step;
      const unsigned int num_fields = first_scan.fields.size();

      // Find the end of the request

      int max_index = -1 ;
      int min_index = INT_MAX ;
      int count_scans = 0;
      const std::string index_field_name("index");

      while ( i < this->scan_hist_.size() &&                              // Don't go past end of deque
          this->scan_hist_[i].header.stamp < req.end )                // Don't go past the end-time of the request
      {
        sensor_msgs::PointCloud2& current_scan = this->scan_hist_[i];
        unsigned int pts_count = this->GetPointsInScan(current_scan) ; 
        if(pts_count>0)
        {
          unsigned int pts_step = current_scan.point_step ;
          sensor_msgs::PointField* index_field = getPointFieldByName(current_scan.fields, index_field_name) ;
          if(!index_field){
            ROS_ERROR("No index information found in cloud! The point field \"index\" is needed to build an organized point cloud 2");
            return false;
          }
          unsigned char* first_index_entry = &(current_scan.data[0+index_field->offset]) ;
          unsigned char* last_index_entry = &(current_scan.data[(pts_count-1)*pts_step+index_field->offset]) ;
          int first_index_value = sensor_msgs::readPointCloud2BufferValue<int>(first_index_entry, index_field->datatype) ;
          int last_index_value = sensor_msgs::readPointCloud2BufferValue<int>(last_index_entry, index_field->datatype) ;
          if(max_index<last_index_value)
            max_index = last_index_value ;
          if(min_index>first_index_value)
            min_index = first_index_value ;
          i += this->downsample_factor_ ;
          count_scans++;
        }
      }

      int index_range = max_index - min_index + 1;
      int index_range_downsampled = (index_range+this->downsample_factor_-1)/this->downsample_factor_;
      unsigned int past_end_index = i ;


      if (start_index == past_end_index)
      {
        resp.cloud.header.frame_id = this->fixed_frame_ ;
        resp.cloud.header.stamp = req.end ;
        resp.cloud.data.resize (0) ;
        resp.cloud.fields.resize (0) ;
        resp.cloud.height = 0;
        resp.cloud.width = 0;
      }
      else
      {
        // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
        // Allocate space for the cloud
        resp.cloud.data.resize (count_scans*index_range_downsampled*point_step);
        resp.cloud.fields.resize (num_fields) ;
        resp.cloud.point_step = point_step ;
        resp.cloud.is_bigendian = first_scan.is_bigendian;

        for (i = 0; i<num_fields; i++)
        {
          resp.cloud.fields[i].name = first_scan.fields[i].name ;
          resp.cloud.fields[i].offset = first_scan.fields[i].offset;
          resp.cloud.fields[i].datatype = first_scan.fields[i].datatype;
          resp.cloud.fields[i].count = 1;
        }
        resp.cloud.header.frame_id = this->fixed_frame_ ;

        unsigned int cloud_count = 0 ;
        unsigned int data_count = 0;
        bool is_dense = true;

        for (i=start_index; i<past_end_index; i+= this->downsample_factor_)
        {
          sensor_msgs::PointCloud2& current_scan = this->scan_hist_[i];

          is_dense &= current_scan.is_dense ;

          if(!hasEqualScanHeader(first_scan, current_scan)){
            return false;
          }

          sensor_msgs::PointField* index_field = getPointFieldByName(current_scan.fields, index_field_name) ;
          std::vector<sensor_msgs::PointField> fields = current_scan.fields;
          int previous_index = min_index-this->downsample_factor_;

          for(unsigned int j=0; j< this->GetPointsInScan(current_scan) * point_step; j+= point_step)
          {
            // read the next index in the cloud
            int index = sensor_msgs::readPointCloud2BufferValue<int>(&(current_scan.data[j+index_field->offset]), index_field->datatype);
            // if the index is smaller than the step size (downsample_factor_), skip it
            if(index < previous_index + this->downsample_factor_){
              continue;
            }
            fillUpIndexHoles(data_count, resp.cloud, previous_index, index, point_step, fields, index_field->offset, index_field->datatype);
            if(previous_index + this->downsample_factor_ == index){
              for(int p_idx=0; p_idx < point_step; p_idx++){
                resp.cloud.data[data_count++] = current_scan.data[j+p_idx] ;
              }
              previous_index = index;
            }

          }
          int max_index_ext = max_index + 1;
          fillUpIndexHoles(data_count, resp.cloud, previous_index, max_index_ext, point_step, fields, index_field->offset, index_field->datatype);
          resp.cloud.header.stamp = current_scan.header.stamp ;
          cloud_count ++ ;
        }

        resp.cloud.height = cloud_count ;
        resp.cloud.width = index_range_downsampled ;
        resp.cloud.row_step = index_range_downsampled * point_step ;
        resp.cloud.is_dense = is_dense ;
      }
      this->scan_hist_mutex_.unlock() ;

      ROS_DEBUG("Point Cloud Results: Aggregated from index %u->%u. BufferSize: %lu. Points in cloud: %u", start_index, past_end_index, this->scan_hist_.size(), resp.cloud.height*resp.cloud.width) ;
      return true ;
    }

  template <class T>
    bool PointCloud2BaseAssembler<T>::assembleScans2Unorganized(AssembleScans2::Request& req, AssembleScans2::Response& resp)
    {
      this->scan_hist_mutex_.lock();  
      // Determine where in our history we actually are
      unsigned int i = 0 ;

      // Find the beginning of the request. Probably should be a search
      while ( i < BaseAssembler<T, sensor_msgs::PointCloud2>::scan_hist_.size() &&                                                    // Don't go past end of deque
          BaseAssembler<T, sensor_msgs::PointCloud2>::scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
      {
        i++ ;
      }
      unsigned int start_index = i ;
      sensor_msgs::PointCloud2& first_scan = this->scan_hist_[start_index];

      unsigned int req_pts = 0 ;          // Keep a total of the points in the current request
      // Find the end of the request

      while ( i < this->scan_hist_.size() &&                              // Don't go past end of deque
          this->scan_hist_[i].header.stamp < req.end )                // Don't go past the end-time of the request
      {
        sensor_msgs::PointCloud2& current_scan = this->scan_hist_[i];
        unsigned int pts_count = this->GetPointsInScan(current_scan) ; 
        unsigned int pts_downsampled = (pts_count+this->downsample_factor_-1)/this->downsample_factor_ ;
        req_pts += pts_downsampled;
        i += this->downsample_factor_ ;
      }
      unsigned int past_end_index = i ;

      if (start_index == past_end_index)
      {
        resp.cloud.header.frame_id = this->fixed_frame_ ;
        resp.cloud.header.stamp = req.end ;
        resp.cloud.data.resize (0) ;
        resp.cloud.fields.resize (0) ;
        resp.cloud.height = 0;
        resp.cloud.width = 0;
      }
      else
      {
        // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
        // Allocate space for the cloud
        const unsigned int point_step = first_scan.point_step;
        const unsigned int num_fields = first_scan.fields.size();
        resp.cloud.data.resize (req_pts * point_step);
        resp.cloud.fields.resize (num_fields) ;
        resp.cloud.point_step = point_step ;
        resp.cloud.is_bigendian = first_scan.is_bigendian;
        resp.cloud.header.frame_id = this->fixed_frame_ ;
        resp.cloud.is_dense = true;
        resp.cloud.height = 1 ;
        resp.cloud.width = req_pts ;
        resp.cloud.row_step = req_pts * point_step ;

        for (i = 0; i<num_fields; i++)
        {
          resp.cloud.fields[i].name = first_scan.fields[i].name ;
          resp.cloud.fields[i].offset = first_scan.fields[i].offset;
          resp.cloud.fields[i].datatype = first_scan.fields[i].datatype;
          resp.cloud.fields[i].count = 1;
        }

        unsigned int data_count = 0;

        bool is_dense = true;
        int max_bytes = req_pts * point_step; 
        for (i=start_index; i<past_end_index; i+= this->downsample_factor_)
        {
          sensor_msgs::PointCloud2& current_scan = this->scan_hist_[i];
          if(!hasEqualScanHeader(first_scan, current_scan)){
            return false;
          }
          for(unsigned int j=0; j< this->GetPointsInScan(current_scan) * point_step; j+= (this->downsample_factor_ * point_step))
          {
            for(unsigned int p_idx=0; p_idx < point_step; p_idx++){
              resp.cloud.data[data_count++] = current_scan.data[j+p_idx] ;
            }
          }
          resp.cloud.header.stamp = current_scan.header.stamp ;
          resp.cloud.is_dense &= current_scan.is_dense ;
        }
      }
      this->scan_hist_mutex_.unlock() ;

      ROS_DEBUG("Point Cloud Results: Aggregated from index %u->%u. BufferSize: %lu. Points in cloud: %u", start_index, past_end_index, this->scan_hist_.size(), req_pts) ;
      return true ;
    }

  template <class T>
    bool PointCloud2BaseAssembler<T>::assembleScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp)
    {
      if(organized){
        return assembleScans2Organized(req, resp);
      }else{
        return assembleScans2Unorganized(req, resp);
      }
    }
}

#endif /* point_cloud2_base_assembler.h */
