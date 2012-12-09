
import processing.*;
import processing.core.*;

import org.openkinect.*;
import org.openkinect.processing.*;

import april.jmat.LinAlg;
import hypermedia.video.*;



public class match2Camera extends PApplet{

	// Daniel Shiffman
	// Kinect Point Cloud example
	// http://www.shiffman.net
	// https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing


	// Kinect Library object
	Kinect kinect;
//	OpenCV opencv;
	float a = 0,b=0;

	// Size of kinect image
	int w = 640;
	int h = 480;
	private int cvY, cvX;

	// We'll use a lookup table so that we don't have to repeat the math over and over
	float[] depthLookUp = new float[2048];
	private boolean depth;
	private boolean rgb;
	private boolean toggleMode;
	private int deg;
	private float perspetiveZ;

	public void setup() {
		size(640,480);
		cvX = cvY = 0;
		depth =rgb = true;
		kinect = new Kinect(this);
		kinect.start();
		kinect.enableDepth(depth);
		kinect.enableRGB(rgb);
		// We don't need the grayscale image in this example
		// so this makes it more efficient
		kinect.processDepthImage(false);
		// Lookup table for all possible depth values (0 - 2047)
//		opencv = new OpenCV( this );
//		opencv.capture( width, height );  // open video stream

	}


	public void draw() {
	//	background(0);
		image(kinect.getDepthImage(),0,0);
//		opencv.read();                   // grab frame from camera
//		image( opencv.image(), cvX, cvY );
	}
	int initialX,initialY;
	
	public void mousePressed() {
		if (mouseButton == LEFT) {
			initialX = mouseX;
			initialY = mouseY;
		}
	}

	public void mouseDragged() 
	{
		if (mouseButton == LEFT) {
			cvX =  mouseX- initialX ;
			cvY =  mouseY- initialY ;
		}
	}
	public void mouseReleased() 
	{
		if (mouseButton == LEFT) {

		}
	}
	
	public void keyPressed() {
		System.out.println("key pressed:"+key);
		if (key == 'd') {
			depth = !depth;
			kinect.enableDepth(depth);
		} 
		else if (key == 'r') {
			rgb = !rgb;
			kinect.enableRGB(rgb);
		}
		else if (key == 'q') {
			stop();
			exit();
		}
		else if (key == CODED) {
			if (keyCode == LEFT) {
				cvX--;
			} 
			else if (keyCode == RIGHT) {
				cvX++;
			}
			else if (keyCode == UP) {
				cvY--;
			} 
			else if (keyCode == DOWN) {
				cvY++;
			}
		}
	}

	public void stop() {
	//	opencv.stop();
		kinect.quit();
		super.stop();
	}

}