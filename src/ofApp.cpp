#include "ofApp.h"

int imageWidth = 640;
int imageHeight = 480;
int panelWidth = 300;

int numJoints = 25;

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetFrameRate(50);

	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	ofSetWindowShape(imageWidth+panelWidth, imageHeight);

	
	targetHost.set("192.168.1.255");
	targetPort.set(9090);

	gui.setup("panel", "settings.xml", imageWidth, 10); // most of the time you don't need a name but don't forget to call setup
	

	gui.add(active.set("active", true));
	gui.setSize(panelWidth, ofGetHeight());
	gui.add(depthImage.set("deptImage",depthImage));
	gui.add(irImage.set("irImage",irImage));
	gui.add(rgbImage.set("rgbImage",rgbImage));

	
	gui.add(setHostBT.setup("Set Host"));
	gui.add(setPortBT.setup("Set Port"));
	gui.add(targetHost.set("Target Host",targetHost));
	gui.add(targetPort.set("Target Port", targetPort));
	
	setHostBT.addListener(this, &ofApp::setHostPressed);
	setPortBT.addListener(this, &ofApp::setPortPressed);
	
	
	gui.add(primaryJoints.set("Primary Joints",primaryJoints));
	gui.add(secondaryJoints.set("Secondary Joints", secondaryJoints));
	
	gui.loadFromFile("settings.xml");

	primaryJoints.addListener(this, &ofApp::primaryChanged);
	secondaryJoints.addListener(this, &ofApp::secondaryChanged);
	
	oscSender.setup(targetHost.get(),targetPort.get());

}

//--------------------------------------------------------------
void ofApp::update(){
	kinect.update();

	//--
	//Getting joint positions (skeleton tracking)
	//--
	//

	//printf("check skeletons\n");
	
	auto bodies = kinect.getBodySource()->getBodies();
	for (auto body : bodies) {

		for (auto joint : body.joints) {
			//now do something with the joints
			
			bool sendThisJoint = false;
			switch (joint.first)
			{
			case 0:
			case 1:
			case 2:
			case 3:
			case 6:
			case 10:
			case 5:
			case 9:
			case 20:
				sendThisJoint = primaryJoints.get();
				break;

			default:
				sendThisJoint = secondaryJoints.get();
				break;
			}

			if (!sendThisJoint) continue;

			ofxOscMessage msg;
			msg.setAddress("/k2s/joint");
			msg.addIntArg(body.bodyId);
			msg.addIntArg(joint.first);

			ofVec3f pos = joint.second.getPosition();
			msg.addFloatArg(pos.x);
			msg.addFloatArg(pos.y);
			msg.addFloatArg(pos.z);
			msg.addIntArg(joint.second.getTrackingState());
			oscSender.sendMessage(msg, false);

			//printf("sending osc\n");

		}
	}
	
	//--
	//Getting bones (connected joints)
	//--
	//
	/*
	{
		// Note that for this we need a reference of which joints are connected to each other.
		// We call this the 'boneAtlas', and you can ask for a reference to this atlas whenever you like
		auto bodies = kinect.getBodySource()->getBodies();
		auto boneAtlas = ofxKinectForWindows2::Data::Body::getBonesAtlas();

		for (auto body : bodies) {
			for (auto bone : boneAtlas) {
				auto firstJointInBone = body.joints[bone.first];
				auto secondJointInBone = body.joints[bone.second];
			}
		}
	}
	*/


}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(20);

	if (active)
	{
		if (depthImage.get())
		{
			kinect.getDepthSource()->draw(0, 0, imageWidth, imageHeight);  // note that the depth texture is RAW so may appear dark
		}

		if (irImage.get())
		{
			kinect.getInfraredSource()->draw(0, 0, imageWidth, imageHeight);
		}

		if (rgbImage.get())
		{
			kinect.getColorSource()->draw(0, 0, imageWidth, imageHeight);
		}
	}

	//kinect.getBodyIndexSource()->draw(0, 0, imageWidth, imageHeight);
	kinect.getBodySource()->drawProjected(0, 0, imageWidth, imageHeight);
	

	//gui
	ofDrawBitmapString("K2S", imageWidth + panelWidth / 2-10, 20);
	gui.draw();
}

void ofApp::setHostPressed()
{
	string host = ofSystemTextBoxDialog("Target host :", targetHost);
	targetHost.set(host);
};


void ofApp::setPortPressed()
{
	string port = ofSystemTextBoxDialog("Target port :", targetPort.toString());
	targetPort.set(ofToInt(port));
};

void ofApp::primaryChanged(bool &value)
{
	ofxOscMessage msg;
	msg.setAddress("/k2s/primary");

	msg.addIntArg(primaryJoints.get() ? 1 : 0);

	oscSender.sendMessage(msg, false);
}

void ofApp::secondaryChanged(bool &value)
{
	ofxOscMessage msg;
	msg.setAddress("/k2s/secondary");
	msg.addIntArg(secondaryJoints.get() ? 1 : 0);
	oscSender.sendMessage(msg, false);
}
//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::exit()
{
	gui.saveToFile("settings.xml");
}