#ifndef KEDA_NAVIGATION_ROBOT_H_
#define KEDA_NAVIGATION_ROBOT_H_

#include <math.h>
struct pose
{
	double x;
	double y;
	double theta;
	pose()
	{
		x = 0.0;
		y = 0.0;
		theta = 0.0;
	}
	pose(double x_,double y_,double theta_)
	{
		x = x_;
		y = y_;
		theta = theta_;
	}
};

class TF
{
public:
	TF()
	{
		initial();
	}
	TF(double x, double y, double theta)
	{
		initial();
		setPose(x,y,theta);
	}
	~TF(){}
	void translate(double dx, double dy)
	{
		m_matrix[0][2] += dx;
		m_matrix[1][2] += dy;
	}
	void rotate(double dtheta)
	{
		m_theta +=dtheta;
		double c = cos(m_theta);
		double s = sin(m_theta);
		m_matrix[0][0]=c;
		m_matrix[0][1]=-s;
		m_matrix[1][0]=s;
		m_matrix[1][1]=c;
	}
	void setPose(double x,double y,double theta)
	{
		double c = cos(theta);
		double s = sin(theta);
		m_theta = theta;
		m_matrix[0][0]=c;
		m_matrix[0][1]=-s;
		m_matrix[0][2]=x;
		m_matrix[1][0]=s;
		m_matrix[1][1]=c;
		m_matrix[1][2]=y;
	}
	pose mul(pose& p)
	{
		pose p1;
		p1.x = m_matrix[0][0]*p.x+m_matrix[0][1]*p.y+m_matrix[0][2];
		p1.y = m_matrix[1][0]*p.x+m_matrix[1][1]*p.y+m_matrix[1][2];
		p1.theta = p.theta + m_theta;
		return p1;
	}
private:
	void initial()
	{
		//初始化
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				if (i == j)
					m_matrix[i][j]=1;
				else
					m_matrix[i][j]=0;
			}
		}
		m_theta = 0;
	}
	double m_theta;
	double m_matrix[3][3];
};
















#endif