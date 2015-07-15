#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
 
using namespace cv;
using namespace std;
 
int main(){
    
	Mat image = imread("C:/Users/weberliu/Desktop/iisu_test/screenshot.jpg", CV_LOAD_IMAGE_COLOR);
    if(! image.data ){// Check for invalid input
        cout <<  "Could not open or find the image" << endl ;
        system("pause");
        return -1;
    }
 
    namedWindow( "HappyMan - Display window", CV_WINDOW_AUTOSIZE );
    // Create a window for display.
    imshow( "HappyMan - Display window", image );
    // Show our image inside it.
 
    waitKey(0);
    // Wait for a keystroke in the window
    return 0;
}