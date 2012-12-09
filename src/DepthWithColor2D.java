import processing.core.*;

import org.openkinect.*;
import org.openkinect.processing.*;

import april.jmat.LinAlg;

public class DepthWithColor2D extends PApplet{
	// Daniel Shiffman
	// Basic Library functionality example
	// http://www.shiffman.net
	// https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing

	float a = 0,b=0;

	Kinect kinect;
	boolean depth = true;
	boolean rgb = true;
	boolean tog2 = false;
	boolean tog3 = false;
	float deg = -15; // Start at 15 degrees
	// Size of kinect image
	int w_p = 640;
	int h_p = 480;

	int w_t = 1280;
	int h_t = 520;

	int toggleMode = 1;



	// We'll use a lookup table so that we don't have to repeat the math over and over
	float[] depthLookUp = new float[2048];
	/*
	double[][] R = {{9.9993325764993979e-01, 9.0861494786104342e-03,
	       7.1359745817653570e-03}, {-9.0772145359783105e-03,
	       9.9995797753139359e-01, -1.2834903944932378e-03},
	       {-7.1473366960762364e-03, 1.2186299591261083e-03,
	       9.9997371491413500e-01}};

	double [] T = { 2.3389325473256353e-02, -2.7202403468988970e-05,
		       -1.3654713167350484e-03 };

	final double fx_d = 1.0 / 5.8879533094470173e+02;
	final double fy_d = 1.0 / 5.8971630340371189e+02;
	final double cx_d = 3.1484832682282348e+02;
	final double cy_d = 2.3226042493189806e+02;
	final double fx_rgb  = 5.2508167987857337e+02;
	final double fy_rgb = 5.2584241340432811e+02;
	final double cx_rgb = 3.2921066824099535e+02;
	final double cy_rgb =2.5885913341699677e+02;
	 */

	/*
	double[][] R = {{9.9995670571540585e-01, 8.7985485386960632e-03,
       3.0285703567210255e-03}, {-8.7989438640149950e-03,
       9.9996128167217235e-01, 1.1723235192374918e-04},{
       -3.0274216210023994e-03, -1.4387549698992368e-04,
       9.9999540699853717e-01}};

double [] T = { 2.4985571218730716e-02, -1.0573824644042034e-03,
       -5.5990324217322814e-03 };

final double fx_d = 1.0 / 5.7842024491784127e+02;
final double fy_d = 1.0 / 5.7941099425488346e+02;
final double cx_d = 3.1539675695657883e+02;
final double cy_d = 2.3401322322521568e+02;
final double fx_rgb  = 5.2099136950425077e+02;
final double fy_rgb = 5.2194477658008850e+02;
final double cx_rgb = 3.2891028990693866e+02;
final double cy_rgb =2.5972997812685401e+02;
	 */

	// from webpage
	double[][] R = {{ 9.9984628826577793e-01, 1.2635359098409581e-03,-1.7487233004436643e-02}, 
			{-1.4779096108364480e-03,  9.9992385683542895e-01, -1.2251380107679535e-02},
			{ 1.7470421412464927e-02, 1.2275341476520762e-02,  9.9977202419716948e-01 }};

	double [] T = {1.9985242312092553e-02, -7.4423738761617583e-04,
			-1.0916736334336222e-02 };

	final double fx_d = 1.0 / 5.9421434211923247e+02;
	final double fy_d = 1.0 / 5.9104053696870778e+02;
	final double cx_d = 3.3930780975300314e+02;
	final double cy_d = 2.4273913761751615e+02;
	final double fx_rgb  = 5.2921508098293293e+02;
	final double fy_rgb = 5.2556393630057437e+02;
	final double cx_rgb = 3.2894272028759258e+02;
	final double cy_rgb =2.6748068171871557e+02;


	double[][] I = {{ 0,0,0,0}, 
			{0,0,0,0},
			{ 0,0,1,0 }};

	double[][] R2 = {{ 1,0,0,0}, 
			{0,1,0,0},
			{ 0,0,1,0 }, 
			{ 0,0,0,1 }};
	double xDegree = 0;
	double yDegree = 0;

	public void setup() {
		
 
		//size(w_t,h_t);
		size(w_t,h_t,P3D);
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

	}

	int[] cPixels = new int[640*480];

	private boolean tog4;

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
		//	System.arraycopy( kinect.getVideoImage().pixels, 0, cPixels, 0, cPixels.length );

		if ( toggleMode%3==2){
			noStroke();
			//	image(kinect.getVideoImage(),0,0);
			image(kinect.getDepthImage(),0,0);
			skip = 10;
			for(int y=0; y<h_p; y+=skip) {
				for(int x=0; x<w_p; x+=skip) {
					//				int offset = (int)result[0]+(int)result[1]*w;
					rawDepth = depth[x+y*w_p];

					double[] result = worldToRGB(depthToWorld(x,y,rawDepth));
					offset = (int)result[0]+(int)result[1]*w_p;
					if(offset <0 || offset >=w_p*h_p){
						continue;
					}
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
					ellipse((float)result[0]+w_p,(float)result[1],depthscale,depthscale);
					result = null;
				}
			}
			fill(255,0,0);
			ellipse(minX,minY,20,20);

		}
		else if (toggleMode%3==1){


			skip = 1;
			loadPixels();
			int offset2;
			int prevRawDepth = 10;
			boolean[] test = new boolean[w_t * h_t];

			double hresult[]= new double[4];
			int px,py;
			int px2,py2;
			int prevMarkedOffset = 0;
			//	image(kinect.getVideoImage(),0,0);
			int pink = color(255, 102, 204);
			int blue = color(20, 255, 255);
			int gray = color(80, 80, 80);
			int count2047 = 0;
			int countOOR = 0;
			int offset3;
			for(int y=0; y<h_p; y+=skip) {
				for(int x=0; x<w_p; x+=skip) {

					offset = x+y*w_p;
					//	pixels[offset] = pink;
					//	if ( true)continue;
					rawDepth = depth[x+y*w_p];
					/*	if (rawDepth == 2047&&tog2){
						//	pixels[offset] = 0;
							count2047++;
							offset3++;
							continue;
						}
					 */
					if (tog2){
						if (rawDepth == 2047)
							rawDepth = prevRawDepth;
						prevRawDepth = rawDepth;
					}
					if(tog4){
						if (rawDepth >=800){
							continue;
						}
					}
					double midResult[] = depthToWorld(x,y,rawDepth);
					//	if ( midResult[2] == 0)
					//	exit();
					double result[] = depthWorldToRGBWorld(midResult);
					//	double result2[] = worldToRGB(depthToWorld(x,y,rawDepth));
					hresult[0]=result[0];
					hresult[1]=result[1];
					hresult[2]=result[2];
					hresult[3]=1;
					double arr[]=LinAlg.matrixAB(I,LinAlg.matrixAB(R2,hresult));
					double arr2[]=LinAlg.matrixAB(I,hresult);
					px= (int)(arr[0]/arr[2]);
					py= (int)(arr[1]/arr[2]);
					px2= (int)(arr2[0]/arr2[2]);
					py2= (int)(arr2[1]/arr2[2]);
					offset2 = px + (py*w_t) ;
					offset3 = px2 + (py2*w_p) ;
					//	offset2 =  ((int)result2[0] + ((int)result2[1]*w)) ;
					/*
					if ((x==0&&y==0) || (x==0&&y==h-1)|| (x==w-1&&y==0)|| (x==w-1&&y==h-1)) {
						System.out.println("(x,y,rawDepth,offset2)"+ "(" + x + "," + y + ","+ rawDepth+","+offset2+")");
						System.out.println("hresult is ");
						LinAlg.print(hresult);
						System.out.println("arr is ");
						LinAlg.print(arr);
					}
					 */
					//System.out.println("x,y,offset,offset2 = ("+x+","+y+","+offset+","+offset2+")");

					if(offset2 <0 || offset2 >=w_t*h_t || px + py * w_p >=w_p*h_p){
						//			pixels[offset] = gray;
						countOOR++;
						continue;
					}

					/*	if ( test[offset2] ){
						pixels[offset] = gray;
						offset3++;
						continue;
					}
					 */	test[offset2] = true;

					 //				
					 pixels[offset2] = kinect.getVideoImage().pixels[offset3];

				}
			}
			int offset1 = 0;
			if ( tog3) {
				for(int y=50; y<h_p-50; y+=skip) {
					for(int x=50; x<w_p-50; x+=skip) {
						offset2 = x+y*w_t;
						if ( test[offset2])
						{
							//offset1 = offset2;
							continue;
						}
						pixels[offset2]  = pixels[offset2-w_t];
					}
				}
			}
			updatePixels();
			text("degree(x,y): (" +  Math.round(xDegree*100.0)/100.0 + ","+  Math.round(yDegree*100.0)/100.0 
					+ ")\ntransX:"+ Math.round(R2[0][3]*100.0)/100.0 
					+ " transY:"+ Math.round(R2[1][3]*100.0)/100.0,10,16);
			image(kinect.getVideoImage(),640,0);

			if(count++%60 == 0)
				System.out.println("countOOR:" + countOOR + ", count2047:" + count2047);// + ", minX:"+ minX+ ", minY:"+ minY + ", rgb:" + kinect.isInterrupted()+ ", rgb2:" + kinect.isAlive());
		}

		else
			draw3D();


	}

	public void draw3D() {

		background(0);

		fill(255);
		textMode(SCREEN);
		text("Kinect FR: " + (int)kinect.getDepthFPS() 
				+ "  Processing FR: " + (int)frameRate,10,16);

		// Get the raw depth as array of integers
		int[] depth = kinect.getRawDepth();
		//	if (toggleMode)ortho();
		// We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
		int skip = 2;
		camera((float)(width/2.0), (float)(height/2.0), (float)((height/2.0) / Math.tan(PI*30 / 180.0))
				, (float)(width/2.0), (float)(height/2.0), 0.0f
				, 0.0f,1.0f,0.0f );
		// Translate and rotate
		translate(width/2,height/2,0);
		rotateX(a);
		rotateY(b);


		float factor = (float)480 ;/// maxWidth;
		int prevRawDepth = 10;

		for(int y=0; y<h_p; y+=skip) {
			for(int x=0; x<w_p; x+=skip) {
				int offset = x+y*w_p;
				int rawDepth = depth[offset];
				//	if (rawDepth ==2047)
				//		continue;
				if (tog2){
					if (rawDepth == 2047)
						rawDepth = prevRawDepth;
					prevRawDepth = rawDepth;
				}
				if(tog4){
					if (rawDepth >=800){
						continue;
					}
				}
				double[] v = depthToWorld(x,y,rawDepth);
				double[] result = worldToRGB(v);
				pushMatrix();
				offset = (int)result[0]+(int)result[1]*w_p;
				
				stroke(kinect.getVideoImage().pixels[offset]);
				translate((float)v[0]*factor,(float)v[1]*factor,factor-(float)v[2]*factor);
				point(0,0);
				popMatrix();
			}
		}


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
		}else if (key == 'a') {
			tog2 = !tog2;
		}else if (key == 'z') {
			tog3 = !tog3;
		}else if (key == 'b') {
			tog4 = !tog4;
		}
		else if (key == 'q') {
			stop();
			exit();

		}
		else if (key == 'i') {
			xDegree += PI / 180.0;
			R2[0][2] = Math.sin(yDegree) * Math.cos(xDegree);
			R2[0][1] = -Math.sin(yDegree) * Math.sin(xDegree);
			R2[2][2] = Math.cos(yDegree) * Math.cos(xDegree);
			R2[2][1] = -Math.cos(yDegree) * Math.sin(xDegree);
			R2[1][2] = -Math.sin(xDegree);
			R2[1][1] = Math.cos(xDegree);
		}
		else if (key == 'k') {
			xDegree -= PI / 180.0;
			R2[0][2] = Math.sin(yDegree) * Math.cos(xDegree);
			R2[0][1] = -Math.sin(yDegree) * Math.sin(xDegree);
			R2[2][2] = Math.cos(yDegree) * Math.cos(xDegree);
			R2[2][1] = -Math.cos(yDegree) * Math.sin(xDegree);
			R2[1][2] = -Math.sin(xDegree);
			R2[1][1] = Math.cos(xDegree);
		}
		else if (key == 'j') {
			yDegree -= PI/180.0;
			R2[0][2] = Math.sin(yDegree) * Math.cos(xDegree);
			R2[0][1] = -Math.sin(yDegree) * Math.sin(xDegree);
			R2[2][2] = Math.cos(yDegree) * Math.cos(xDegree);
			R2[2][1] = -Math.cos(yDegree) * Math.sin(xDegree);
			R2[2][0] = Math.sin(yDegree);
			R2[0][0] = Math.cos(yDegree);
		}
		else if (key == 'l') {
			yDegree += PI/180.0;
			R2[0][2] = Math.sin(yDegree) * Math.cos(xDegree);
			R2[0][1] = -Math.sin(yDegree) * Math.sin(xDegree);
			R2[2][2] = Math.cos(yDegree) * Math.cos(xDegree);
			R2[2][1] = -Math.cos(yDegree) * Math.sin(xDegree);
			R2[2][0] = Math.sin(yDegree);
			R2[0][0] = Math.cos(yDegree);
		}else if (key == 'x') {
			R2[1][3] =0;
			R2[0][3] =0;
			xDegree =0;
			yDegree =0;
			R2[1][1] =  R2[0][0] = R2[2][2] = 1;
			R2[0][1] =R2[1][0] =R2[0][2] =R2[2][0] =R2[1][2] =R2[2][1] = 0;	
			a = b = 0;
		}else if (key == 'w') {
			depth = true;
			rgb = true;
			kinect.enableDepth(!depth);
			kinect.enableRGB(!rgb);
			kinect.enableDepth(depth);
			kinect.enableRGB(rgb);
		}
		else if (key == CODED) {
			if (keyCode == UP) {
				if  (toggleMode%3==1)
					R2[1][3] -= 0.01;
				else 
					a+=PI/180.0;

			} 
			else if (keyCode == DOWN) {
				if  (toggleMode%3==1)
					R2[1][3] += 0.01;
				else 
					a-=PI/180.0;

			}
			if (keyCode == LEFT) {
				if  (toggleMode%3==2)
					R2[0][3] -= 0.01;
				else 
					b+=PI/180.0;

			} 
			else if (keyCode == RIGHT) {
				if  (toggleMode%3==2)
					R2[0][3] += 0.01;
				else 
					b-=PI/180.0;

			}
	
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
