# K2S
K2S is a OSC kinect v2 streamer.

# OSC messages sent
- /k2s/joint bodyId jointType xPosition yPosition zPosition xRotation yRotation zRotation trackingState
- /k2s/body/update bodyId leftHandState rightHandState
- /k2s/body/entered bodyId 
- /k2s/body/left bodyId

