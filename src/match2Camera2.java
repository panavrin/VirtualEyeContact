import java.awt.Rectangle;

import processing.*;
import processing.core.*;

import org.openkinect.*;
import org.openkinect.processing.*;

import april.jmat.LinAlg;
import hypermedia.video.*;


public class match2Camera2 extends PApplet{
	OpenCV opencv;

	Kinect kinect;
	boolean depth = false;
	boolean rgb = true;
	boolean ir = false;
	float deg = 0; // Start at 15 degrees
	private int cvY, cvX;
	PImage trailsImg;



	public void setup() {

		size(640,480,P3D);
		kinect = new Kinect(this);
		kinect.start();
		kinect.enableDepth(depth);
		kinect.enableRGB(rgb);
		//	kinect.enableIR(ir);
		kinect.tilt(deg);
		opencv = new OpenCV( this );
		opencv.capture( width, height );  // open video stream
		// opencv.cascade( OpenCV.CASCADE_FRONTALFACE_ALT );    // load the FRONTALFACE description file
		trailsImg = new PImage( width, height );


	}
	
	PImage oImg = new PImage();
	boolean fUpdate = true;
	public void draw() {
	//	background(0);
		tint(255, 200);  // Apply transparency without changing color
		if(fUpdate){
			opencv.read();   
			try {
				oImg =  (PImage) opencv.image().clone();
			} catch (CloneNotSupportedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}	
		/*	
		 */	
		image(kinect.getVideoImage(),0,0);
		//		image(trailsImg,0,0);
		//		trailsImg = kinect.getDepthImage();


		// grab frame from camera
		image( oImg, cvX, cvY );
		//   trailsImg.blend( opencv.image(), 0, 0, 640, 480, 0, 0, 640, 480, EXCLUSION );

		// detect anything ressembling a FRONTALFACE
		/*  Rectangle[] faces = opencv.detect();

	    // draw detected face area(s)
	    noFill();
	    stroke(255,0,0);
	    for( int i=0; i<faces.length; i++ ) {
	        rect( cvX + faces[i].x, cvY + faces[i].y, faces[i].width, faces[i].height ); 
	    }
		 */
		fill(255);
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
		if (key == 'd') {
			depth = !depth;
			kinect.enableDepth(depth);
		} 
		else if (key == 'r') {
			rgb = !rgb;
			if (rgb) ir = false;
			kinect.enableRGB(rgb);
		}
		else if (key == 'i') {
			ir = !ir;
			if (ir) rgb = false;
			kinect.enableIR(ir);
		} 
		else if (key == 'q') {
			stop();
			exit();
		}
		else if (key == 'a'){
			fUpdate = !fUpdate;
			rgb = !rgb;
			if (rgb) ir = false;
			kinect.enableRGB(rgb);
		}
		else if (key == CODED) {
			if (keyCode == UP) {
				if(mousePressed){
					oImg.resize(0,oImg.height+3);
					PImage rImg = null;
					try {
						rImg = (PImage) oImg.clone();
					} catch (CloneNotSupportedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					PImage temp = oImg;
					oImg = rImg;
					temp = null;
					
				}
			} 
			else if (keyCode == DOWN) {
				if(mousePressed){
					oImg.resize(0,oImg.height-3);
					PImage rImg = null;
					try {
						 rImg = (PImage) oImg.clone();
					} catch (CloneNotSupportedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					PImage temp = oImg;
					oImg = rImg;
					temp = null;
				}
			}
			deg = constrain(deg,0,30);
			kinect.tilt(deg);
		}
	}
	public void stop() {
		opencv.stop();
		kinect.quit();
		super.stop();
	}


}
