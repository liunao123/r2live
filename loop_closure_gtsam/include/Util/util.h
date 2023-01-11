#ifndef _UTIL_UTIL_H_
#define _UTIL_UTIL_H_
#include <vector>
#include <assert.h>
#include <math.h>
#include <iostream>
#include <assert.h>

using namespace std;

#define BIN 50

class Util
{
public:
	/* 高度直方图
	* 说明：将val插入到BIN个箱子的直方图中
	* param:
	* in: val取值范围在[[-1,5]之间
	* out: histogram直方图数组  
	*         histogram_samples所有采样点个数
	*/
	static void bin_into_height_histogram(float val, vector<float>& histogram, int& histogram_samples)
	{
		int index = (int)((val+1)*BIN/6);    //计算点的法向量投影到z轴的
		if(index < 0)
			index =0;
		if(index >BIN-1)
			index =BIN-1;
		histogram[index]++;
		histogram_samples++;
	}
	/* 距离直方图
	* 说明：将val插入到BIN个箱子的直方图中
	* param:
	* in: val取值范围在[[0,30]之间
	* out: histogram直方图数组  
	*         histogram_samples所有采样点个数
	*/
	static void bin_into_distance_histogram(float val, vector<float>& histogram, int& histogram_samples)
	{
		int index = (int)((val)*BIN/30);    //计算点的法向量投影到z轴的
		if(index < 0)
			index =0;
		if(index >BIN-1)
			index =BIN-1;
		histogram[index]++;
		histogram_samples++;
	}
	/* 法向量直方图
	* 说明：将val插入到BIN个箱子的直方图中
	* param:
	* in: val取值范围在[[0,1]之间
	* out: histogram直方图数组  
	*         histogram_samples所有采样点个数
	*/
	static void bin_into_normal_histogram(float val, vector<float>& histogram, int& histogram_samples)
	{
		int index = (int)((val)*BIN/1);    //计算点的法向量投影到z轴的
		if(index < 0)
			index =0;
		if(index >BIN-1)
			index =BIN-1;
		histogram[index]++;
		histogram_samples++;
	}
	/* 曲率直方图
	* 说明：将val插入到BIN个箱子的直方图中
	* param:
	* in: val取值范围在[[0,0.3]之间
	* out: histogram直方图数组  
	*         histogram_samples所有采样点个数
	*/
	static void bin_into_curvature_histogram(float val, vector<float>& histogram, int& histogram_samples)
	{
		int index = (int)(val *BIN/0.3); //计算点的曲率
		if(index < 0)
			index =0;
		if(index >BIN-1)
			index =BIN-1;
		histogram[index]++;
		histogram_samples++;
	}
	/* 形状直方图
	* 说明：将val插入到BIN个箱子的直方图中
	* param:
	* in: val取值范围在[[0.5,2.5]之间
	* out: histogram直方图数组  
	*         histogram_samples所有采样点个数
	*/
	static void bin_into_shape_histogram(float val, vector<float>& histogram, int& histogram_samples)
	{
		int index = (int)((val-0.5)*BIN/2);    //计算点的法向量投影到z轴的
		if(index < 0)
			index =0;
		if(index >BIN-1)
			index =BIN-1;
		histogram[index]++;
		histogram_samples++;
	}
	/*
	* 说明：归一化直方图
	* param:
	* in: histogram直方图数组  
	*      histogram_samples所有采样点个数
	* out: 
	*      histogram直方图数组归一化后  
	*/
	static void normalize_histogram (vector<float>& histogram, int& histogram_samples)
	{
		//std::cout <<"Current frame histogram points: "<<histogram_samples<<std::endl;
		for ( int i = 0; i < histogram.size(); ++i)
		{
			histogram[i] = histogram[i]/histogram_samples;
		}
	}
	/*
	* 说明：打印直方图
	* param:
	* in: histogram直方图数组  
	*
	*/
	static void display_histogram(const vector<float>& histogram)
	{
		std::cout <<"Current frame histogram is: ";
		for ( int i = 0; i < histogram.size(); ++i)
		{
			std::cout <<histogram[i]<<",";
		}
		std::cout <<std::endl;
	}
	/*
	* 说明：计算两个直方图的欧氏距离
	* param:
	* in: histogram1:直方图数组  
	*	 histogram2:直方图数组  
	*/
	static double euclidean_diistance(const vector<float>& histogram1, const vector<float>& histogram2)
	{
		assert(histogram1.size() == histogram2.size());

		double ret =0.0;
		for(int i=0; i< histogram1.size(); i++)
		{
			ret +=(histogram1[i] - histogram2[i]) * (histogram1[i] - histogram2[i]);
		}
		return sqrt(ret);
	};
	static double x2_diistance(const vector<float>& histogram1, const vector<float>& histogram2)
	{
		assert(histogram1.size() == histogram2.size());

		double ret =0.0;
		for(int i=0; i< histogram1.size(); i++)
		{
			if(histogram1[i] + histogram2[i] == 0)
				continue;
			double temp = (histogram1[i] - histogram2[i]) * (histogram1[i] - histogram2[i]);
			ret += temp / (histogram1[i] + histogram2[i]);
		}
		return ret;
	};

};


#endif