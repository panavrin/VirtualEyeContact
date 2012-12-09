import processing.core.*;

import org.openkinect.*;
import org.openkinect.processing.*;


public class PlainDepthRGB extends PApplet{
	// Daniel Shiffman
	// Basic Library functionality example
	// http://www.shiffman.net
	// https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing


	Kinect kinect;
	boolean depth = true;
	boolean rgb = true;

	public void setup() {
		size(1280,520);
		kinect = new Kinect(this);
		kinect.start();
		kinect.enableDepth(depth);
		kinect.enableRGB(rgb);
	
		int h = kinect.getVideoImage().height;
		int w = kinect.getVideoImage().width;
		int hd = kinect.getDepthImage().height;
		int wd = kinect.getDepthImage().width;
		System.out.println("h:"+h+" w:"+w + " hd:"+ hd + " wd:"+wd);
		
	}

	int[] cPixels = new int[640*480];

	public void draw() {
		background(0);
		image(kinect.getDepthImage(),640,0);
		image(kinect.getVideoImage(),0,0);
	}
	public void keyPressed() {
		if (key == 'q') {
			stop();
			exit();
			
		}	
	}


	public void stop() {
		kinect.quit();
		super.stop();
	}

}
