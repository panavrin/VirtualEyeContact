import processing.core.*;

import org.openkinect.*;
import org.openkinect.processing.*;

import april.jmat.LinAlg;

public class test1 extends PApplet{
	// Daniel Shiffman
	// Basic Library functionality example
	// http://www.shiffman.net
	// https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing


	Kinect kinect;
	boolean depth = true;
	boolean rgb = true;

	float deg = 0; // Start at 15 degrees
	// Size of kinect image
	int w = 640;
	int h = 480;

	boolean toggleMode = true;

	// We'll use a lookup table so that we don't have to repeat the math over and over
	float[] depthLookUp = new float[2048];

	double[][] R = {{ 9.9984628826577793e-01, 1.2635359098409581e-03,-1.7487233004436643e-02}, 
			{-1.4779096108364480e-03,  9.9992385683542895e-01, -1.2251380107679535e-02},
			{ 1.7470421412464927e-02, 1.2275341476520762e-02,  9.9977202419716948e-01 }};

	double[][] T = {{ 1,0,0,1}, 
			{0,1,0,0},
			{ 0,0,1,0 }, 
			{ 0,0,0,1 }};
	double[][] R2 = {{ 1,0,0,1}, 
			{0,1,0,0},
			{ 0,0,1,0 }, 
			{ 0,0,0,1 }};

	public void setup() {
		size(1280,520);
		kinect = new Kinect(this);
		kinect.start();
		kinect.enableDepth(depth);
		kinect.enableRGB(rgb);
		//		kinect.enableIR(ir);
		kinect.tilt(deg);

		// Lookup table for all possible depth values (0 - 2047)
		for (int i = 0; i < depthLookUp.length; i++) {
			depthLookUp[i] = rawDepthToMeters(i);
		}
	}

	int[] cPixels = new int[640*480];

	public void draw() {
		background(0);
		// Get the raw depth as array of integers
		int[] depth = kinect.getRawDepth();

		// We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
		image(kinect.getVideoImage(),0,0);
		//	image(kinect.getDepthImage(),640,0);
		fill(255);
		text("DEPTH FPS: " + (int) kinect.getDepthFPS(),640,495);
		text("Press 'd' to enable/disable depth    Press 'r' to enable/disable rgb image   Press 'i' to enable/disable IR image  UP and DOWN to tilt camera   Framerate: " + frameRate,10,515);
		double maxDepth = 0;
		double minDepth = 100;
		float minX=0, minY=0;
		int minColor=0;
		int skip = 20;
		int offset, color, rawDepth;
		int h = kinect.getVideoImage().height;
		int w = kinect.getVideoImage().width;
	//	System.arraycopy( kinect.getVideoImage().pixels, 0, cPixels, 0, cPixels.length );

		if ( toggleMode){
			skip = 10;
			for(int y=0; y<h; y+=skip) {
				for(int x=0; x<w; x+=skip) {
					//				int offset = (int)result[0]+(int)result[1]*w;
					rawDepth = depth[x+y*w];

					double[] result = worldToRGB(depthToWorld(x,y,rawDepth));
					offset = (int)result[0]+(int)result[1]*w;
				
					//color = cPixels[offset];
					color = kinect.getVideoImage().pixels[offset];
					if(maxDepth < result[2])
						maxDepth = result[2];
					if(minDepth > result[2] && result[2]>0){
						minDepth = result[2];
						minX = (float)result[0];
						minY = (float)result[1];
						minColor = color;
					}

					fill(color); 
					float depthscale =3*(6- (float)result[2]);
					ellipse((float)result[0]+640,(float)result[1],depthscale,depthscale);
					result = null;
				}
			}
			fill(255,0,0);
			ellipse(minX,minY,20,20);

		}
		else{
			skip = 1;
			loadPixels();
			int offset2;
			int pink = color(255, 102, 204);

			for(int y=0; y<h; y+=skip) {
				for(int x=0; x<w; x+=skip) {
					offset = x+y*w;
					offset2 = x+640 + (y*2 * w);
					pixels[offset2] = kinect.getVideoImage().pixels[offset];
				
				}
			}
			updatePixels();
		}
		if(count++%60 == 0)
			System.out.println("maxDepth:" + maxDepth + ", minDepth:" + minDepth + ", minX:"+ minX+ ", minY:"+ minY + ", rgb:" + kinect.isInterrupted()+ ", rgb2:" + kinect.isAlive());
		

	}
	int count=0;
	public void keyPressed() {
		if (key == 'd') {
			depth = !depth;
			kinect.enableDepth(depth);
		} 
		else if (key == 'r') {
			rgb = !rgb;
			kinect.enableRGB(rgb);
		}
		else if (key == 's') {
			toggleMode = !toggleMode;
		}
		else if (key == 'q') {
			stop();
			exit();
		}
		else if (key == CODED) {
			if (keyCode == UP) {
				deg++;
			} 
			else if (keyCode == DOWN) {
				deg--;
			}
			deg = constrain(deg,0,30);
			kinect.tilt(deg);
		}
	}


	// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
	public float rawDepthToMeters(int depthValue) {
		if (depthValue < 2047) {
			return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
		}
		return 0.0f;
	}
	final double fx_d = 1.0 / 5.9421434211923247e+02;
	final double fy_d = 1.0 / 5.9104053696870778e+02;
	final double cx_d = 3.3930780975300314e+02;
	final double cy_d = 2.4273913761751615e+02;
	final double fx_rgb  = 5.2921508098293293e+02;
	final double fy_rgb = 5.2556393630057437e+02;
	final double cx_rgb = 3.2894272028759258e+02;
	final double cy_rgb =2.6748068171871557e+02;

	double[] depthToWorld(int x, int y, int depthValue) {
		double[] result = new double[3];
		double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
		result[0] = (float)((x - cx_d) * depth * fx_d);
		result[1] = (float)((y - cy_d) * depth * fy_d);
		result[2] = (float)(depth);
		return result;
	}

	double[] worldToRGB(double[] depthCoord){
		double[] result = new double[3];
		double[] rotated = LinAlg.matrixAB(R, depthCoord);
		result[0] = rotated[0] * fx_rgb /  rotated[2] + cx_rgb;
		result[1] = rotated[1] * fy_rgb /  rotated[2] + cy_rgb;
		result[2] = depthCoord[2];
		return result;
	}

	public void stop() {
		kinect.quit();
		super.stop();
	}

}
