#include "ofApp.h"

int imageWidth = 640;
int imageHeight = 480;
int panelWidth = 300;

int numJoints = 25;

float lastNewFrameTime = 0;
float maxNoNewFrameTime = .5;
bool kinectIsConnected = false;

int oscReceivePort = 9091;

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetFrameRate(50);
	ofSetWindowShape(imageWidth + panelWidth, imageHeight);

	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();
	
	targetHost.set("192.168.1.255");
	targetPort.set(9090);

	gui.setup("panel", "settings.xml", imageWidth, 10); // most of the time you don't need a name but don't forget to call setup

	gui.add(active.set("active", true));

	gui.setSize(panelWidth, ofGetHeight());
	gui.add(depthImage.set("deptImage",depthImage));
	gui.add(irImage.set("irImage",irImage));
	gui.add(rgbImage.set("rgbImage", rgbImage));
	
	gui.add(setHostBT.setup("Set Host"));
	gui.add(setPortBT.setup("Set Port"));
	gui.add(targetHost.set("Target Host",targetHost));
	gui.add(targetPort.set("Target Port", targetPort, 2000, 10000));
	setHostBT.addListener(this, &ofApp::setHostPressed);
	setPortBT.addListener(this, &ofApp::setPortPressed);
	
	gui.add(primaryJoints.set("Primary Joints",primaryJoints));
	gui.add(secondaryJoints.set("Secondary Joints", secondaryJoints));
	

	gui.loadFromFile("settings.xml");

	
	oscSender.setup(targetHost.get(),targetPort.get());
	oscReceiver.setup(oscReceivePort);
	

	primaryJoints.addListener(this, &ofApp::primaryChanged);
	secondaryJoints.addListener(this, &ofApp::secondaryChanged);

	lastNewFrameTime = 0;
}

//--------------------------------------------------------------
void ofApp::update(){

	//OSC

	// check for waiting messages
	while (oscReceiver.hasWaitingMessages()) {
		// get the next message
		ofxOscMessage m;
		oscReceiver.getNextMessage(m);

		// check for mouse moved message
		if (m.getAddress() == "/k2s/active") {
			active = m.getArgAsInt(0) > 0;
		}
		// check for mouse button message
		else if (m.getAddress() == "/k2s/device/connected") {
			// the single argument is a string
			setKinectConnected(kinectIsConnected, true);
		}
		// check for an image being sent (note: the size of the image depends greatly on your network buffer sizes - if an image is too big the message won't come through ) 
		else if (m.getAddress() == "/k2s/primary") {
			primaryJoints = m.getArgAsInt(0) > 0;
		}
		else if (m.getAddress() == "/k2s/secondary") {
			secondaryJoints = m.getArgAsInt(0) > 0;
		}

	}


	//KINECT PROCESSING

	if (!active) return;

	kinect.update();
	
	if (kinect.isFrameNew())
	{
		lastNewFrameTime = ofGetElapsedTimef();
		setKinectConnected(true);
	}else
	{
		if (ofGetElapsedTimef() - lastNewFrameTime > maxNoNewFrameTime)
		{
			setKinectConnected(false);
		}
	}

	auto bodies = kinect.getBodySource()->getBodies();

	//search for new bodies
	for (auto body : bodies) {
		if (body.tracked)
		{
			bool found = false;
			for (int id : lastBodiesIds)
			{
				if (id == body.bodyId) found = true;
			}

			if (!found)
			{
				//new body detected
				printf("Body entered : %i / %i\n", body.bodyId);
				ofxOscMessage msg;
				msg.setAddress("/k2s/body/entered");
				msg.addIntArg(body.bodyId);
				oscSender.sendMessage(msg,false);
			}
		}
	}

	//Seach for left bodies
	for (int id: lastBodiesIds) {
		bool found = false;
		for (auto body : bodies)
		{
			if (body.tracked && id == body.bodyId) found = true;
		}

		if (!found)
		{
			//body left detected
			ofxOscMessage msg;
			msg.setAddress("/k2s/body/left");
			msg.addIntArg(id);
			oscSender.sendMessage(msg,false);

			printf("Body left : %i\n", id);

		}
	}

	lastBodiesIds.clear();

	for (auto body : bodies) {

		if (body.tracked)
		{
			lastBodiesIds.push_back(body.bodyId);

			ofxOscMessage msg;
			msg.setAddress("/k2s/body/update");
			msg.addIntArg(body.bodyId);
			msg.addIntArg((int)body.leftHandState);
			msg.addIntArg((int)body.rightHandState);
			oscSender.sendMessage(msg, false);
		}

		for (auto joint : body.joints) {
			
			bool sendThisJoint = false;
			switch (joint.first)
			{
			case JointType_SpineBase:
			case JointType_SpineMid:
			case JointType_Neck:
			case JointType_Head:
			case JointType_WristLeft:
			case JointType_WristRight:
			case JointType_ElbowLeft:
			case JointType_ElbowRight:
			case JointType_SpineShoulder:
				sendThisJoint = primaryJoints.get();
				break;

			default:
				sendThisJoint = secondaryJoints.get();
				break;
			}

			if (!sendThisJoint) continue;

			ofxOscMessage jointMsg;
			jointMsg.setAddress("/k2s/joint");
			jointMsg.addIntArg(body.bodyId);
			jointMsg.addIntArg(joint.first);

			ofVec3f pos = joint.second.getPosition();
			jointMsg.addFloatArg(pos.x);
			jointMsg.addFloatArg(pos.y);
			jointMsg.addFloatArg(pos.z);

			ofVec3f rot = joint.second.getOrientation().getEuler();
			jointMsg.addFloatArg(rot.x);
			jointMsg.addFloatArg(rot.y);
			jointMsg.addFloatArg(rot.z);

			jointMsg.addIntArg(joint.second.getTrackingState());
			oscSender.sendMessage(jointMsg, false);

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
		//bool imageAv = kinect.getDepthSource()->getTexture().isAllocated();

		if(kinectIsConnected)
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

			//kinect.getBodyIndexSource()->draw(0, 0, imageWidth, imageHeight);
			kinect.getBodySource()->drawProjected(0, 0, imageWidth, imageHeight);

		}
		else
		{
			ofPushMatrix();
			ofPushStyle();
			ofTranslate(50, 50);
			ofScale(3, 3);
			ofSetColor(ofColor::orangeRed);
			ofDrawBitmapString("KINECT IS NOT CONNECTED", 0,0);
			ofPopStyle();
			ofPopMatrix();

		}
	}

	
	//gui
	ofDrawBitmapString("K2S", imageWidth + panelWidth / 2-10, 20);
	gui.draw();
}


void ofApp::setKinectConnected(bool value, bool force)
{
	if (kinectIsConnected == value && !force) return;
	kinectIsConnected = value;

	printf("Kinect Connection Changed : %i\n",kinectIsConnected);
	ofxOscMessage msg;
	msg.setAddress("/k2s/device/connected");
	msg.addIntArg(kinectIsConnected?1:0);
	oscSender.sendMessage(msg, false);
}

void ofApp::setHostPressed()
{
	string host = ofSystemTextBoxDialog("Target host :", targetHost);
	targetHost.set(host);
	oscSender.setup(targetHost.get(), targetPort.get());
};


void ofApp::setPortPressed()
{
	string port = ofSystemTextBoxDialog("Target port :", targetPort.toString());
	targetPort.set(ofToInt(port));
	oscSender.setup(targetHost.get(), targetPort.get());
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