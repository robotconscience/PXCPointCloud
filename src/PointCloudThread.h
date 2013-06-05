#pragma once

#include "ofMain.h"
#include "pxcupipeline.h"

class PointCloudThread : protected ofThread {
public:

	~PointCloudThread(){
		waitForThread();
	}

	void setup(){
		mSession = PXCUPipeline_Create();
		if(!PXCUPipeline_Init(mSession, (PXCUPipeline)(PXCU_PIPELINE_COLOR_VGA | PXCU_PIPELINE_DEPTH_QVGA)))
		{
			ofLogNotice() << "Unable to initialize session" << endl;
			return;
		}

		if(PXCUPipeline_QueryRGBSize(mSession, &mCW, &mCH))
		{
			mRGBMap = new unsigned char[mCW*mCH*4];
			for (int i=0; i<mCW * mCH * 4; i++){
				mRGBMap[i] = 0;
			}
			mRGBTexture.allocate(mCW,mCH,GL_RGBA);
		}

		if(PXCUPipeline_QueryDepthMapSize(mSession, &mLW, &mLH))
		{
			mDepthBuffer = new short[mLW*mLH];
			mDepthMap = new unsigned char[mLW*mLH];
			mDepthTexture.allocate(mLW,mLH,GL_LUMINANCE);
		}

		if ( PXCUPipeline_QueryUVMapSize(mSession, &uvW, &uvH )){
			mUVMap = new float [uvW * uvH * 2];
			for (int i=0; i<uvW * uvH * 2; i++){
				mUVMap[i] = 0;
			}
		}

		pointCloud.setupIndicesAuto();
		bufferMesh.setupIndicesAuto();
		startThread(false);
	}

	ofMesh & getMesh(){
		return bufferMesh;
	}

	void draw(){
		bufferMesh.drawVertices();
	}

protected:
	void threadedFunction(){
		while( isThreadRunning()){
			if(PXCUPipeline_AcquireFrame(mSession, false))
			{
				// load RGB as often as we can
				bool bRGB = PXCUPipeline_QueryRGB(mSession, mRGBMap);

				// same with UV
				bool bUV = PXCUPipeline_QueryUVMap(mSession, mUVMap);

				// load depth into point cloud
				bool getDepth = PXCUPipeline_QueryDepthMap(mSession, mDepthBuffer);
				if(getDepth)
				{
					pointCloud.clear();

					int numToSkip = 0;
					int index = 0;
					
					static int xFactor = (float) (mCW / mLW );
					static int yFactor = (float) (mCH / mLH );

					for(int i=0;i<mLW*mLH;++i)
					{
						int x = i % mLW;
						int y = floor( (float) i/mLW);

						int sx=(int)(mUVMap[(y*mLW+x)*2+0]*mLW+0.5) * 2 ;
						int sy=(int)(mUVMap[(y*mLW+x)*2+1]*mLH+0.5) * 2;
						
						if ( sx < 0 || sx > mCW || sy < 0 && sy > mCH ){
							sx = x * 2;
							sy = y * 2;
						}

							float r = mRGBMap[ ( sx + (sy * mCW)) * 4 ] / 255.0f;;
							float g = mRGBMap[ (( sx + (sy * mCW)) * 4) + 1] / 255.0f;
							float b = mRGBMap[ (( sx + (sy * mCW)) * 4) + 2] / 255.0f;
							float a = mRGBMap[ (( sx + (sy * mCW)) * 4) + 3] / 255.0f;

							if ( index == 0 ){
								if ( (float)mDepthBuffer[i] > 0 ){
									pointCloud.addVertex( ofVec3f(x, y, mDepthBuffer[i]) );
									pointCloud.addColor( ofFloatColor(r,g,b,a));
								}
							}
							index++;
							if ( index >= numToSkip ){
								index = 0;
							}
						//}
					}

					//mDepthTexture.loadData(mDepthMap, mLW, mLH, GL_LUMINANCE);
				}
				PXCUPipeline_ReleaseFrame(mSession);

				lock();
				bufferMesh = pointCloud;
				unlock();
			}
		}
	}

private:
	int mCW, mCH, mLW, mLH, uvW, uvH;
	unsigned char *mRGBMap, *mDepthMap;
	float *mUVMap;
	short *mIRBuffer, *mDepthBuffer;

	ofTexture	mDepthTexture, mRGBTexture;
	ofMesh		pointCloud, bufferMesh;

	PXCUPipeline_Instance mSession;
};