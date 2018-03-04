#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"

#include <ros/ros.h>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Image.h"

#include "localization_nao_pkg/Points.h"
#include "localization_nao_pkg/PPVector.h"

using namespace cv;
using namespace std;

localization_nao_pkg::PPVector point_to_point_vector;

localization_nao_pkg::Points pts;

vector<float> linefactor;//will keep the factor of the lines we accept

Mat dst, cdst, src;
static const std::string OPENCV_WINDOW = "Image window";

//--------------------------------------------------------------------//
///////////////-----------------------------------------////////////////
//--------------------------------------------------------------------//

class ImageConverter
{
    ros::NodeHandle nh_;
   	ros::Publisher line_pub;
	ros::Subscriber sub;


    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
 
public:
    ImageConverter()
     : it_(nh_)
    {
    	
	    // Subscrive to input video feed and publish output video feed
	    image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 1, &ImageConverter::findLine, this);
	    
	    line_pub = nh_.advertise<localization_nao_pkg::PPVector>("point_to_point_line_with_factor",10);

	    //image_pub_ = it_.advertise("/image_converter/output_video", 1);
	 
	    cv::namedWindow(OPENCV_WINDOW);
    }
   

    ~ImageConverter()
    {
    	cv::destroyWindow(OPENCV_WINDOW);
    }
   

    void findLine(const sensor_msgs::ImageConstPtr& msg)
    {

    	ROS_INFO("Nao Image Seq: [%d]", msg->header.seq);

        cv_bridge::CvImagePtr cv_ptr;
       
        try
        {
        	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
        	ROS_ERROR("cv_bridge exception: %s", e.what());
        	return;
        }

        //we work on cloned image
        src=cv_ptr->image.clone();

        //Detect the edges of the image
   		//Parameters: (Source image, Output, minThreshold, maxThreshold,
   		//kernel size)
		Canny(src, dst, 200, 600, 3);
        //cv::imshow("canny", dst);

		blur( dst, dst, Size(3,3) );
		//cv::imshow("blur", dst);

        //Convert the image to grayscale
        //Parameters: (input, output, color conversion)
   		cvtColor( dst, cdst, COLOR_GRAY2BGR );
   		//cv::imshow("GRAY2BGR", cdst);

   		vector<Vec4i> lines;

   		//Use Probabilistic Hough Transform
   		//Parameters: (output of edge detector in grayscale,
   		//vector to store parameters  (x_start, y_start, x_end, y_end),
   		//resolution of parameter r in pixel, // θ in radians,
   		//threshold - minimum number of intersections to “detect” a line,
   		//minLinLength - minimum number of points that can form a line,
   		//maxLineGap - maximum gap between two points in the same line)
  		HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );

  		int xx,yy,xxx,yyy;
  		int flag[lines.size()];//the line is similar with at least one more, then 1

  		bool s1,s2,s3,s4;

  		int lnum=0;//store num of lines we accept

  		float fact,fact1,fact2;

  		//if existing lines are >1 we dont need this
  		//In the 2 for loops we try to find similar lines to draw only the 
  		//ones that are unique based on point(x,y)
  		if(lines.size()>1)
  		{
	  		for( size_t i = 0; i < lines.size(); i++ )
	  		{

	  			//we store the lines in the vector
	  			Vec4i l = lines[i];

	  			xx=l[0];yy=l[1];xxx=l[2];yyy=l[3];
	  			flag[i]=0;

	  			//i dont want to do the loop for the last point cause
	  			//it will compare itself with itself
	  			if(i!=lines.size()-1)
	  			{
		  			for( size_t j = i+1; j < lines.size(); j++ )
		  			{

		  				l = lines[j];

		  				/*
		  				cout<<xx<<"---"<<l[0]<<endl;
		  				cout<<yy<<"---"<<l[1]<<endl;
		  				cout<<xxx<<"---"<<l[2]<<endl;
		  				cout<<yyy<<"---"<<l[3]<<endl;
		  				*/

		  				//convert to boolean expressions - fabs=absolute
		  				s1=(fabs(xx-l[0]))<10;
		  				s2=(fabs(yy-l[1]))<10;
		  				s3=(fabs(xxx-l[2]))<10;
		  				s4=(fabs(yyy-l[3]))<10;

		  				//cout<<endl<<s1<<s2<<s3<<s4<<endl;

		  				//if points have difference in pixels smaller than 10
		  				//at least 3 bool expressions are 1
		  				if((s1+s2+s3+s4)>2)
		  				{
		  					//then we have similar lines which we dont want
		  					flag[i]=1;

		  				}
		  			}
	  			}

	  			//cout<<endl<<flag[i]<<endl;

	  			if(flag[i]==0)
	  			{

	  				
	  				Vec4i lnew = lines[i];
	  				line( cdst, Point(lnew[0], lnew[1]), Point(lnew[2], lnew[3]), Scalar(0,0,255), 3, CV_AA);
	  			
	  				

		  			fact1=lnew[3]-lnew[1];
		  			fact2=lnew[2]-lnew[0];
	  				fact=fact1/fact2;//(y2-y1)/(x2-x1)
	  				linefactor.push_back(fact);//push value

	  				pts.x1=lnew[0];
	  				pts.y1=lnew[1];
	  				pts.x2=lnew[2];
	  				pts.y2=lnew[3];
					pts.factor=fact;
					
					point_to_point_vector.pp_vector.push_back(pts);

	  				/*point_to_point_vector.pp_vector[lnum].x1=lnew[0];
	  				point_to_point_vector.pp_vector[lnum].y1=lnew[1];
	  				point_to_point_vector.pp_vector[lnum].x2=lnew[2];
	  				point_to_point_vector.pp_vector[lnum].y2=lnew[3];
	  				point_to_point_vector.pp_vector[lnum].factor=fact;
					*/
	  				//cout<<fact1<<"/"<<fact2<<"--->"<<fact<<endl;
	  				//cout<<linefactor[lnum]<<endl;
					
	  				line_pub.publish(point_to_point_vector);

	  				//cout<<point_to_point_vector.pp_vector[lnum]<<endl;

	  				lnum++;

	  			}
	  		}

	  		linefactor.clear();
	  		point_to_point_vector.pp_vector.clear();

  		}


  		cout<<lnum<<endl;
  		cout<<lines.size()<<endl;

        /*// STANDARD HOUGH LINE TRANSFORM 
        vector<Vec2f> lines;
  		HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );
  		for( size_t i = 0; i < lines.size(); i++ )
  		{
	    	float rho = lines[i][0], theta = lines[i][1];
	     	Point pt1, pt2;
	     	double a = cos(theta), b = sin(theta);
	    	double x0 = a*rho, y0 = b*rho;
	     	pt1.x = cvRound(x0 + 1000*(-b));
	     	pt1.y = cvRound(y0 + 1000*(a));
	     	pt2.x = cvRound(x0 - 1000*(-b));
	     	pt2.y = cvRound(y0 - 1000*(a));
	     	line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  		}
		*/

 		for(int x=0;x<src.rows;x++)
 		{
 			for(int y=0;y<src.cols;y++)
 			{


 			}

 		}
   		

        //Update GUI Window
        cv::imshow(OPENCV_WINDOW, cdst);
        cv::waitKey(3);
   
        // Output modified video stream
        //image_pub_.publish(cv_ptr->toImageMsg());
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_finder_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}
