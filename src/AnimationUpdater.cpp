#include "AnimationUpdater.h"

using namespace Ogre;

void AnimationUpdater::keyPressed (const OIS::KeyEvent &e){
    if (e.key == OIS::KC_W){ 
	_forward=true;
    }
    if (e.key == OIS::KC_S){ 
	_jump=true;
    }

}

void AnimationUpdater::keyReleased (const OIS::KeyEvent &e){
  if (e.key == OIS::KC_W) _forward=false;
  if (e.key == OIS::KC_S) _jump=false;
  
}

void AnimationUpdater::update(const Ogre::FrameEvent& evt){

 
	if(_forward==false && _jump==false){
		_animBlender->blend("Stand", AnimationBlender::Switch, 0, false);
  	}
	if(_forward){
		_animBlender->blend("Run", AnimationBlender::Blend, 0.5, false);
	}
	if(_jump){
		_animBlender->blend("Jump", AnimationBlender::Blend, 0.5, false);
	}
 
	_animBlender->addTime(evt.timeSinceLastFrame);
}
