import processing.core.*;

import org.openkinect.*;
import org.openkinect.processing.*;

import april.jmat.LinAlg;

public class test3 extends PApplet{
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

	int toggleMode = 1;

	// We'll use a lookup table so that we don't have to repeat the math over and over
	float[] depthLookUp = new float[2048];
	double[][] R = {{9.9986583586943278e-01, 1.3933618520368512e-02,
	    8.6118834205556674e-03 },{-1.3802378793147345e-02,
	        9.9979048725103170e-01, -1.5115420668805807e-02},
	        {-8.8206916265604930e-03, 1.4994528244440669e-02,
	        9.9984866831033770e-01}};
	    	
	double [] T = { 2.5519829514162606e-02, 4.5547959459655762e-03,
		       -1.8768902009820901e-03 };

	final double fx_d = 1.0 / 5.8543321103545850e+02;
	final double fy_d = 1.0 / 5.8565103173830676e+02;
	final double cx_d = 3.0986206725285871e+02;
	final double cy_d = 2.3925413480022684e+02;
	final double fx_rgb  = 5.2077977885169787e+02;
	final double fy_rgb = 5.2150824862499417e+02;
	final double cx_rgb = 3.2681302450787661e+02;
	final double cy_rgb =2.6153301732128864e+02;


	double[][] I = {{ 0,0,0,0}, 
			{0,0,0,0},
			{ 0,0,1,0 }};
	
	double[][] R2 = {{ 1,0,0,0}, 
			{0,1,0,0},
			{ 0,0,1,0 }, 
			{ 0,0,0,1 }};
	double xDegree = 0;

	public void setup() {
		size(640,520);
		kinect = new Kinect(this);
		kinect.start();
		kinect.enableDepth(depth);
		kinect.enableRGB(rgb);
		//		kinect.enableIR(ir);
		kinect.tilt(deg);
		I[0][0]=fx_rgb;
		I[0][2]=cx_rgb;
		I[1][1]=fy_rgb;
		I[1][2]=cy_rgb;
		// Lookup table for all possible depth values (0 - 2047)
		for (int i = 0; i < depthLookUp.length; i++) {
			depthLookUp[i] = rawDepthToMeters(i);
		}
		
		int h = kinect.getVideoImage().height;
		int w = kinect.getVideoImage().width;
		int hd = kinect.getDepthImage().height;
		int wd = kinect.getDepthImage().width;
		System.out.println("h:"+h+" w:"+w + " hd:"+ hd + " wd:"+wd);
		
	}

	int[] cPixels = new int[640*480];

	public void draw() {
		background(0);
		// Get the raw depth as array of integers
		int[] depth = kinect.getRawDepth();

		// We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
		//	image(kinect.getDepthImage(),640,0);

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

		if ( toggleMode%3==1){
		//	image(kinect.getVideoImage(),0,0);
			image(kinect.getDepthImage(),0,0);
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
				//	ellipse((float)result[0],(float)result[1],depthscale,depthscale);
					result = null;
				}
			}
			fill(255,0,0);
			ellipse(minX,minY,20,20);

		}
		else if (toggleMode%3==2){
			skip = 1;
			loadPixels();
			int offset2;
			int prevRawDepth = 10;

			double hresult[]= new double[4];
			int px,py;
			int prevMarkedOffset = 0;
		//	image(kinect.getVideoImage(),0,0);
			int pink = color(255, 102, 204);

			int count2047 = 0;
			int countOOR = 0;
			for(int y=0; y<h; y+=skip) {
				for(int x=0; x<w; x+=skip) {
					offset = x+y*w;
				//	pixels[offset] = pink;
				//	if ( true)continue;
					rawDepth = depth[x+y*w];
		/*			if (rawDepth == 2047){
						//	pixels[offset] = 0;
							count2047++;
							continue;
						}
			*/	/*	if (rawDepth == 2047)
						 rawDepth = prevRawDepth;
					prevRawDepth = rawDepth;
					*/
					double result[] = depthWorldToRGBWorld(depthToWorld(x,y,rawDepth));
				//	double result2[] = worldToRGB(depthToWorld(x,y,rawDepth));
					hresult[0]=result[0];
					hresult[1]=result[1];
					hresult[2]=result[2];
					hresult[3]=1;
					double arr[]=LinAlg.matrixAB(I,LinAlg.matrixAB(R2,hresult));
				//	double arr[]=LinAlg.matrixAB(I,hresult);
					px= (int)(arr[0]/arr[2]);
					py= (int)(arr[1]/arr[2]);
					offset2 = px + (py*w) ;
				//	offset2 = (int) (result2[0] + (result2[1]*w)) ;
					/*
					if ((x==0&&y==0) || (x==0&&y==h-1)|| (x==w-1&&y==0)|| (x==w-1&&y==h-1)) {
						System.out.println("(x,y,rawDepth,offset2)"+ "(" + x + "," + y + ","+ rawDepth+","+offset2+")");
						System.out.println("hresult is ");
						LinAlg.print(hresult);
						System.out.println("arr is ");
						LinAlg.print(arr);
					}
					*/
					
					
					if(offset2 <=0 || offset2 >= pixels.length){
					//	pixels[offset] = 0;
						countOOR++;
						continue;
					}
					pixels[offset2] = kinect.getVideoImage().pixels[offset2];
				//	prevMarkedOffset = offset2;
				}
			}
		updatePixels();
		if(count++%60 == 0)
			System.out.println("countOOR:" + countOOR + ", count2047:" + count2047);// + ", minX:"+ minX+ ", minY:"+ minY + ", rgb:" + kinect.isInterrupted()+ ", rgb2:" + kinect.isAlive());

		}
		else
			image(kinect.getVideoImage(),0,0);



	}
	int count=0;
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
		else if (key == 's') {
			toggleMode++;// = !toggleMode;
		}
		else if (key == 'q') {
			stop();
			exit();
			
		}
		else if (key == 'i') {
			xDegree += PI / 180.0;
			R2[1][1] = Math.cos(xDegree);
			R2[1][2] = Math.sin(xDegree);
			R2[2][2] = Math.cos(xDegree);
			R2[2][1] = -Math.sin(xDegree);
		}
		else if (key == 'k') {
			xDegree -= PI / 180.0;
			R2[1][1] = Math.cos(xDegree);
			R2[1][2] = Math.sin(xDegree);
			R2[2][2] = Math.cos(xDegree);
			R2[2][1] = -Math.sin(xDegree);	
		}
		else if (key == 'o') {
			R2[1][3] += 0.01;
		}
		else if (key == 'l') {
			R2[1][3] -= 0.01;
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


	double[] depthToWorld(int x, int y, int depthValue) {
		double[] result = new double[3];
		double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
		result[0] = (float)((x - cx_d) * depth * fx_d);
		result[1] = (float)((y - cy_d) * depth * fy_d);
		result[2] = (float)(depth);
		return result;
	}
	
	double[] depthWorldToRGBWorld(double[] depth3DCoord){
		return LinAlg.add(LinAlg.matrixAB(R, depth3DCoord),T);
	}

	double[] worldToRGB(double[] depthCoord){
		double[] result = new double[3];
		double[] rotated = LinAlg.add(LinAlg.matrixAB(R, depthCoord),T);
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
