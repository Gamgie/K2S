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

		ofParameter<bool> primaryJoints; //spinebase,spineMid,neck,head, hands, feets
		ofParameter<bool> secondaryJoints; //shoulders, arms, fingers, knee, hips,... everything left
	
		void ofApp::setHostPressed();
		void ofApp::setPortPressed();

		void ofApp::primaryChanged(bool &value);
		void ofApp::secondaryChanged(bool &value);
};

