
import processing.*;
import processing.core.*;

import org.openkinect.*;
import org.openkinect.processing.*;

import april.jmat.LinAlg;

public class pointCloud extends PApplet{

	// Daniel Shiffman
	// Kinect Point Cloud example
	// http://www.shiffman.net
	// https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing


	// Kinect Library object
	Kinect kinect;

	float a = 0;

	// Size of kinect image
	int w = 640;
	int h = 480;


	// We'll use a lookup table so that we don't have to repeat the math over and over
	float[] depthLookUp = new float[2048];

	private boolean depth;

	private boolean rgb;

	private boolean toggleMode;

	private int deg;

	private float perspetiveZ;

	public void setup() {
		size(640,480,P3D);
		deg = 0;
		perspetiveZ = 10;
		depth = true;
		rgb = true;
		kinect = new Kinect(this);
		kinect.start();
		kinect.enableDepth(depth);
		kinect.enableRGB(rgb);
		// We don't need the grayscale image in this example
		// so this makes it more efficient
		kinect.processDepthImage(false);

		// Lookup table for all possible depth values (0 - 2047)
		for (int i = 0; i < depthLookUp.length; i++) {
			depthLookUp[i] = rawDepthToMeters(i);
		}
	}
	int count = 0;
	float maxWidth =1;
	public void draw() {

		background(0);
		fill(255);
		textMode(SCREEN);
		text("Kinect FR: " + (int)kinect.getDepthFPS() 
				+ "\nProcessing FR: " + (int)frameRate
				+ "\nz: " + a
				+ "\nperspetiveZ: "+ perspetiveZ,10,16);

		// Get the raw depth as array of integers
		int[] depth = kinect.getRawDepth();
	//	if (toggleMode)ortho();
		// We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
		int skip = 1;
		float upY = (float)( Math.cos(PI*perspetiveZ/180.0f));
		float upZ = -(float)(Math.sin(PI*perspetiveZ/180.0f));
		camera((float)(width/2.0), (float)(height/2.0), (float)((height/2.0) / Math.tan(PI*27 / 180.0))
	//	camera((float)(width/2.0), (float)(height/2.0), 10
				, (float)(width/2.0), (float)(height/2.0), 0.0f
				, 0.0f,1.0f,0.0f );
	//	System.out.println("upY" + upY + " upZ:"+ upZ);
		// Translate and rotate
		translate(width/2,height/2,0);
		rotateX(a);
		stroke(255);
		
		float factor = 480 ;/// maxWidth;
		float width = 0, leftX = 0, rightX = 0;
		maxWidth = 0;
		int color;
		int prevRawDepth = 10;

		for(int y=0; y<h; y+=skip) {
			for(int x=0; x<w; x+=skip) {
				int offset = x+y*w;

				// Convert kinect data to world xyz coordinate
				int rawDepth = depth[offset];
				if (rawDepth == 2047)
					 rawDepth = prevRawDepth;
				prevRawDepth = rawDepth;
				double[] v = depthToWorld(x,y,rawDepth);
				float[] result = worldToRGB(v);

//				if (x==0) leftX = (float) v[0];
	//			else if (x==w-1) rightX = (float) v[0];
				pushMatrix();
				// Scale up by 1000
				offset = (int)result[0]+(int)result[1]*w;

				//color = cPixels[offset];
				color = kinect.getVideoImage().pixels[offset];
				stroke(color);

				translate((float)v[0]*factor,(float)v[1]*factor,factor-(float)v[2]*factor);
				// Draw a point
				point(0,0);
				popMatrix();
			}
			width = rightX - leftX;
		//	if ( minWidth >width) minWidth = width;
		//	if ( maxWidth <width) maxWidth = width;
		}
	//	if (count++%10==0)
		//	System.out.println("maxWidth:" + maxWidth);
		// Rotate
		//  a += 0.015f;
	}

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
		else if (key == 'i') {
			a+=PI/30.0;
		}
		else if (key == 'k') {
			a-=PI/30.0;
		}
		else if (key == CODED) {
			if (keyCode == UP) {
				perspetiveZ++;
			} 
			else if (keyCode == DOWN) {
				perspetiveZ--;
			}
			else if (keyCode == LEFT) {
				//	perspetiveZ--;
				a-=0.1;
			}
			else if (keyCode == RIGHT) {
				System.out.println("moving");
				//	perspetiveZ++;
				a+=0.1;
			}
			deg = constrain(deg,0,30);
			kinect.tilt(deg);
		}
	}

	// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
	float rawDepthToMeters(int depthValue) {
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
	double[][] R = {{ 9.9984628826577793e-01, 1.2635359098409581e-03,-1.7487233004436643e-02}, 
			{-1.4779096108364480e-03,  9.9992385683542895e-01, -1.2251380107679535e-02},
			{ 1.7470421412464927e-02, 1.2275341476520762e-02,  9.9977202419716948e-01 }};

	double[] depthToWorld(int x, int y, int depthValue) {
		double[] result = new double[3];
		double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
		result[0] = (float)((x - cx_d) * depth * fx_d);
		result[1] = (float)((y - cy_d) * depth * fy_d);
		result[2] = (float)(depth);
		return result;
	}

	float[] worldToRGB(double[] depthCoord){
		float[] result = new float[3];
		double[] rotated = LinAlg.matrixAB(R, depthCoord);
		result[0] = (float) (rotated[0] * fx_rgb /  rotated[2] + cx_rgb);
		result[1] = (float) (rotated[1] * fy_rgb /  rotated[2] + cy_rgb);
		result[2] = (float) depthCoord[2];
		return result;
	}

	public void stop() {

		kinect.quit();
		super.stop();
	}

}
