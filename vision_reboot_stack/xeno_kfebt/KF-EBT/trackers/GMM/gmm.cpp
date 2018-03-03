#include "gmm.h"


void GMM::init(Mat frame,Rect region){
  pMOG2 =  createBackgroundSubtractorMOG2();
  pMOG2->apply(frame, fgMaskMOG2);
  image = frame;
  rect_gmm=region;
}
Rect GMM::track(Mat frame,Rect region){

        Mat img = frame.clone();

        pMOG2->apply(frame,fgMaskMOG2);

        Mat img_hsv;

      //  imshow("FG Mask MOG 2", fgMaskMOG2);

        //cvtColor(img,img_ycrcb,CV_BGR2YCrCb);
          cvtColor(img,img_hsv,CV_BGR2HSV);
        //TEST BLUE GOOD IF IT GIVES result

        Mat blue_tub(frame.rows,frame.cols,CV_8UC1);
        Mat blue_ycrcb(frame.rows,frame.cols,CV_8UC1);
        int no_pixels=0;
        for (int i = 0; i < frame.rows; i++)
        {
            for (int j = 0; j < frame.cols; j++)
            {
              //  h = get_h((int)img_hsv.at<Vec3b>(i,j)[0],(int)img_hsv.at<Vec3b>(i,j)[1],(int)img_hsv.at<Vec3b>(i,j)[2]);
                if((int)fgMaskMOG2.at<uchar>(i,j)==255 && (int)img_hsv.at<Vec3b>(i,j)[0] > 150 )
                {
                    blue_tub.at<uchar>(i,j)=255;
                    no_pixels++;
                }
                else  blue_tub.at<uchar>(i,j)= 0 ;

              // blue_ycrcb.at<uchar>(i,j) = (int) blue_ycrcb.at<Vec3b>(i,j)[0];
            }
        }
        cout<<"pixles "<<no_pixels<<endl;
        //  imshow("blue_tub",blue_tub);
          //  waitKey(10);
          GaussianBlur(blue_tub,blue_tub,Size(45,45),2,2);

          vector<vector<Point> > contours;
          vector<Vec4i> hierarchy;

          findContours(blue_tub, contours, hierarchy, RETR_CCOMP,CHAIN_APPROX_SIMPLE);

          cout << contours.size() << '\n';

            if(contours.size() != 0)
            {


                /// Draw contours

                Mat drawing = Mat::zeros( blue_tub.size(), CV_8UC3 );
                Mat dst = Mat::zeros( blue_tub.size(), CV_8UC3 );

                int idx = 0, largestComp = 0;
                double maxArea = 0;

                vector<Point2f>center( contours.size() );
                vector<float>radius( contours.size() );
                vector<vector<Point> > contours_poly( contours.size() );
                Point2f vertex;
                Point2f vertex_opp;

                for( ; idx >= 0; idx = hierarchy[idx][0] )
                {
                    const vector<Point>& c = contours[idx];
                    double area = fabs(contourArea(Mat(c)));
                    if( area > maxArea )
                    {
                        maxArea = area;
                        largestComp = idx;
                    }
                    approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
                    minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );

                }
                Scalar color( 0, 0, 255 );
              rect_gmm=boundingRect( Mat(contours_poly[largestComp]) );

                vertex.y = center[largestComp].y - radius[largestComp];
                vertex.x = center[largestComp].x - radius[largestComp];
                vertex_opp.y = center[largestComp].y + radius[largestComp];
                vertex_opp.x = center[largestComp].x + radius[largestComp];
                drawContours( dst, contours, largestComp, color, FILLED, LINE_8, hierarchy );
                rectangle( drawing, vertex, vertex_opp , color, 2, 8, 0 );


                //circle( drawing, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
                /*
                Point2f vertex;
                Point2f vertex_opp;
                vertex.y = 100;
                vertex.x = 100;
                vertex_opp.y = 150;
                vertex_opp.x = 200;*/
              //  int  keyboard = waitKey( 30 );
            //   rect_gmm= Rect(vertex,vertex_opp);
               cout<<(int)vertex.x<<"  "<<(int)vertex.y<<endl;
               return rect_gmm;
            }

            else { cout<<"madarchod"<<endl;
              return region;
            }
}
