#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2\src\ofxKinectForWindows2.h"
#include "ofxOsc\src\ofxOsc.h"
#include "ofxGui\src\ofxGui.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		ofxKFW2::Device kinect;
		ofxOscSender oscSender;
		ofxOscReceiver oscReceiver;

		//gui and params
		ofxPanel gui;

		ofParameter<bool> active;
		ofParameter<bool> depthImage;
		ofParameter<bool> irImage;
		ofParameter<bool> rgbImage;

		ofParameter<string> targetHost;
		ofxButton setHostBT;
		ofParameter<int> targetPort;
		ofxButton setPortBT;
		void setHostPressed();
		void setPortPressed();

		ofParameter<int> sendRate; //fps
		ofParameter<bool> primaryJoints; //spinebase,spineMid,neck,head, hands, feets
		ofParameter<bool> secondaryJoints; //shoulders, arms, fingers, knee, hips,... everything left
		void primaryChanged(bool &value);
		void secondaryChanged(bool &value);

		vector<int> lastBodiesIds;

		void setKinectConnected(bool value, bool force = false);

};

