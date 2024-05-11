#ifndef GRIDMAP_H_
#define GRIDMAP_H_

#include <iostream>
#include <assert.h>
#include <vector>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using namespace std;

class Grid{
public:
	Grid(){
		center_x_ =	center_y_ = 	grid_resolution_ = 0;
		average_height_= max_negative_gradient_ = max_height_difference_ = pointCount_ = 0 ;
		isEmpty_= true; isObstacle_ = false;
		max_height_ = -10000.0f; min_height_ = 10000.0f;
	}
	Grid(int x, int y, float width) {
		grid_resolution_ = width;
		center_x_ = (float)(x) +  grid_resolution_/ 2;
		center_y_ = (float)(y) + grid_resolution_ / 2;
        //printf("center_x %f y %f x %d y %d width %f \n", center_x_, center_y_, x, y, width);
		average_height_= max_negative_gradient_ = max_height_difference_ = pointCount_ = 0 ;
		isEmpty_= true; isObstacle_ = false;
	}

	void computeAverageHeight(){
		if (!isEmpty_ ){
			float sum = 0.0f;
			for(auto point:pointVector_) {
				sum += point.z;
			}
			if(pointCount_ != 0) average_height_ = sum / pointCount_;
		}
	}

	void computeMaxHeightDiff(){
		if (!isEmpty_ ){
			for(auto point:pointVector_) {
				if(point.z < min_height_) min_height_ = point.z;
				if(point.z > max_height_) max_height_ = point.z;
				max_height_difference_ = max_height_ - min_height_;
			}
		}	
	}

	void addPoint(pcl::PointXYZ &point){
		pointVector_.push_back(point);
		pointCount_ += 1;
		isEmpty_ = false;
	}
	void clearPoint(){
		pointVector_.clear();
		pointCount_ = 0;
		isEmpty_ = true;
		isObstacle_ = false;
		min_height_ = 10000.0f;
		max_height_ = -10000.0f;
	}

	float getAverageHeight() {return average_height_; }
	float getMaxHeightDiff() {return max_height_difference_; }
	float getMaxHeight() { return max_height_; }
	float getMinHeight() {return min_height_; }
	float getMaxNegativeGradient() {return max_negative_gradient_; }
	float getX() { return center_x_; }
	float getY() { return center_y_; }
	int    getPointCount() { return pointCount_; }
	float isObs() {return isObstacle_;}


	float center_x_, center_y_;
	float grid_resolution_;
	float average_height_;
	float max_negative_gradient_;
	float max_height_, min_height_;
	float max_height_difference_;
	bool isObstacle_;
	bool isEmpty_;
	int pointCount_;
	std::vector<pcl::PointXYZ> pointVector_;
	std::vector<float> gradientVector_;
};

class GridMap{
public:
	vector<vector<Grid>> grid_vec2; //2d map
	int width_, length_;
	float grid_resolution_;

	GridMap(int width, int length, float grid_resolution){
		width_ = width; length_ = length; grid_resolution_ = grid_resolution;
		for(int i = 0; i < width_; i ++){
			grid_vec2.push_back(vector<Grid>(length_));
		}

		int i, j;
		for(i = 0; i < width_; i++){
			for(j = 0; j < length_; j++){
				grid_vec2[i][j] = Grid(i, j, grid_resolution_);
                //printf("i %d j %d grid_resolution %f \n", i, j, grid_resolution_);
				// cout<<"Grid x: "<< test.getX()<< "\t y: "<< test.getY()<<endl;
				// cout<<"Grid x: "<< grid_vec2[i][j].getX()<< "\t y: "<< grid_vec2[i][j].getY()<<endl;
			}
			// cout<<"i: "<< i << "\t j: "<<j<<"\t\t";
			
		}
		//cout<<"Grid x: "<< test.getX()<< "\t y: "<< test.getY()<<endl;
	}
	void addToGrid(pcl::PointXYZ &point){
		int index_x = (int)(point.x / grid_resolution_);
		int index_y = (int)(point.y / grid_resolution_);
		grid_vec2[index_x][index_y].addPoint(point);
	}
	void setGridToObs();

	void extractProperties(){
		for(int j = 0; j < length_; j++){
			for(int i = 0; i < width_; i++){
				//cout<<"--- processing [ "<<i<<" , "<<j<<" ]..."<<endl;
				grid_vec2[i][j].computeAverageHeight();
				//grid_vec2[i][j].computeMaxHeightDiff();				
			}
		}
		computeGradients();
	}

	void extractSurface(float thresh){
		for(int j = 1; j < length_ - 1; j++){
			for(int i = 1; i < width_ - 1; i++){
				if(!grid_vec2[i][j].isEmpty_){
					if(grid_vec2[i][j].max_negative_gradient_ < (-1 * thresh)){
						grid_vec2[i][j].isObstacle_ = true;
						//cout<<"max_negative_gradient: "<<grid_vec2[i][j].max_negative_gradient_<<endl;
					}
					else grid_vec2[i][j].isObstacle_ = false;
				}
			}
		}
	}
    
	void generateMem(std::vector<signed char>& v){
		for(int j = 0; j < length_ ; j++){
			for(int i = 0; i < width_ ; i++){
				if(grid_vec2[i][j].isEmpty_){
					v.push_back(0);
				}
				else
					v.push_back(100);
			}
		}
	}
	struct Occu_Data
	{
		double x;
		double y;
		int  color;
	};
	void generateMap(std::list<struct Occu_Data>& p){
		for(int j = 0; j < length_ ; j++){
			for(int i = 0; i < width_ ; i++){
				if(!grid_vec2[i][j].isEmpty_){
					Occu_Data temp_data;
					temp_data.x=(i+0.5)*grid_resolution_;
					temp_data.y=(j+0.5)*grid_resolution_;
					temp_data.color=0;
					p.push_back(temp_data);
				}
				
			}
		}
	}
	void colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points){
		for(int j = 1; j < length_ - 1; j++){
			for(int i = 1; i < width_ - 1; i++){
				if(!grid_vec2[i][j].isEmpty_ ){
					if(grid_vec2[i][j].isObstacle_){
						for(auto point:grid_vec2[i][j].pointVector_) 
						{
							pcl::PointXYZRGB p;
							p.x = point.x;
							p.y = point.y;
							p.z = point.z;
							p.r = 255;
							p.g = 0;
							p.b = 0;
							points->push_back(p);
						}

					}
					else {
						for(auto point:grid_vec2[i][j].pointVector_) 
						{
							pcl::PointXYZRGB p;
							p.x = point.x;
							p.y = point.y;
							p.z = point.z;
							p.r = 0;
							p.g = 255;
							p.b = 0;
							points->push_back(p);
						}

					}
				}
			}
		}
	}

	void filter_surface(pcl::PointCloud<pcl::PointXYZ>::Ptr points){
		for(int j = 1; j < length_ - 1; j++){
			for(int i = 1; i < width_ - 1; i++){
				if(!grid_vec2[i][j].isEmpty_ ){
					if(grid_vec2[i][j].isObstacle_){
						for(auto point:grid_vec2[i][j].pointVector_) 
						{
							pcl::PointXYZ p;
							p.x = point.x;
							p.y = point.y;
							p.z = point.z;
							points->push_back(p);
						}

					}

				}
			}
		}
	}
	void computeGradients(){
		float max_negative_gradient;
		for(int j = 1; j < (length_ - 1); j++){
			for(int i = 1; i < (width_ - 1); i++){
				if(!grid_vec2[i][j].isEmpty_){
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i + 1][j].getAverageHeight() - grid_vec2[i][j].getAverageHeight());
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i + 1][j + 1].getAverageHeight() - grid_vec2[i][j].getAverageHeight());
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i][j + 1].getAverageHeight() - grid_vec2[i][j].getAverageHeight());
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i - 1][j + 1].getAverageHeight() - grid_vec2[i][j].getAverageHeight());
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i - 1][j].getAverageHeight() - grid_vec2[i][j].getAverageHeight());
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i - 1][j - 1].getAverageHeight() - grid_vec2[i][j].getAverageHeight());
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i][j - 1].getAverageHeight() - grid_vec2[i][j].getAverageHeight());
					grid_vec2[i][j].gradientVector_.push_back(grid_vec2[i + 1][j - 1].getAverageHeight() - grid_vec2[i][j].getAverageHeight());

					max_negative_gradient = 1000.0f;//这里是指最大负值
					for(auto gradient:grid_vec2[i][j].gradientVector_){
						if(gradient < max_negative_gradient) max_negative_gradient = gradient;
					}
					grid_vec2[i][j].max_negative_gradient_ = max_negative_gradient;
				}
			}
		}
	}
};
#endif
