#include "PlayState.h"
#include "PauseState.h"
#include "IntroState.h"
#include "Shapes/OgreBulletCollisionsTrimeshShape.h"	
#include "Shapes/OgreBulletCollisionsSphereShape.h"	
#include "Utils/OgreBulletCollisionsMeshToShapeConverter.h"
#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>

using namespace Ogre;

template<> PlayState* Ogre::Singleton<PlayState>::msSingleton = 0;

bool inAbsoluteRange(float checkedFloat, float maximum){
  return checkedFloat < maximum && checkedFloat > -maximum;
}

bool inAbsoluteRange(btVector3 playerVelocityVector, float maximum){
  return inAbsoluteRange(playerVelocityVector.x(),maximum) && 
  inAbsoluteRange(playerVelocityVector.y(),maximum) && 
  inAbsoluteRange(playerVelocityVector.z(),maximum);
}

void scaleMesh(const Ogre::Entity *_ent, const Ogre::Vector3 &_scale)
{
    bool added_shared = false;
    Ogre::Mesh* mesh = _ent->getMesh().getPointer();
    Ogre::Vector3 Minimum=mesh->getBounds().getMaximum();
    Ogre::Vector3 Maximum=mesh->getBounds().getMinimum();

    // Run through the submeshes, modifying the data
    for(int i = 0;i < mesh->getNumSubMeshes();i++)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
            }

            const Ogre::VertexElement* posElem = vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
            Ogre::HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());
            // lock buffer for read and write access
            unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_NORMAL));
            Ogre::Real* pReal;

            for(size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                // modify x coord
                (*pReal) *= _scale.x;
                ++pReal;

                // modify y coord
                (*pReal) *= _scale.y;
                ++pReal;

                // modify z coord
                (*pReal) *= _scale.z;
                pReal-=2;

                Minimum.x=Minimum.x<(*pReal)?Minimum.x:(*pReal);
                Maximum.x=Maximum.x>(*pReal)?Maximum.x:(*pReal);
                ++pReal;
                Minimum.y=Minimum.y<(*pReal)?Minimum.y:(*pReal);
                Maximum.y=Maximum.y>(*pReal)?Maximum.y:(*pReal);
                ++pReal;
                Minimum.z=Minimum.z<(*pReal)?Minimum.z:(*pReal);
                Maximum.z=Maximum.z>(*pReal)?Maximum.z:(*pReal);
            }
            vbuf->unlock();
        }
    }
    mesh->_setBounds(Ogre::AxisAlignedBox(Minimum,Maximum),false);
}

void
PlayState::enter ()
{
	initSDL (); 
	_root = Ogre::Root::getSingletonPtr();
	
	//Overlay
	_overlayManager = OverlayManager::getSingletonPtr();
	Overlay *overlay = _overlayManager->getByName("Info2");
	overlay->show();

	// Se recupera el gestor de escena y la cámara.
	_sceneMgr = _root->getSceneManager("SceneManager");
	_sceneMgr -> setAmbientLight(Ogre::ColourValue(0.6, 0.6, 0.6));
  	_sceneMgr -> setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
	_camera = _sceneMgr->getCamera("IntroCamera");
	_viewport = _root->getAutoCreatedWindow()->addViewport(_camera);
	
	//Camera configuration
	_camera->setNearClipDistance(0.1);
	_camera->setFarClipDistance(10);

	// Nuevo background colour.
	_viewport->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 0.0));
	
	//Sound Managers
	 _pTrackManager = new TrackManager;
 	_pSoundFXManager = new SoundFXManager;
 	(_pTrackManager->load("lightintro.ogg"))->play();
  
  //Ground and Lights initialization
  _sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);	
  _sceneMgr->setShadowColour(Ogre::ColourValue(0.5, 0.5, 0.5) );
  _sceneMgr->setAmbientLight(Ogre::ColourValue(0.9, 0.9, 0.9));

  _sceneMgr->setShadowTextureCount(2);
  _sceneMgr->setShadowTextureSize(512);
  
   light = _sceneMgr->createLight("Light1");
  light->setPosition(-11,16,71);
  light->setType(Ogre::Light::LT_SPOTLIGHT); 
  light->setDirection(Ogre::Vector3(1,-1,0));
  light->setSpotlightInnerAngle(Ogre::Degree(25.0f));
  light->setSpotlightOuterAngle(Ogre::Degree(60.0f));
  light->setSpotlightFalloff(5.0f);
  light->setCastShadows(true);

  Ogre::Light* light2 = _sceneMgr->createLight("Light2");
  light2->setPosition(3,12,3);
  light2->setDiffuseColour(0.2,0.2,0.2);
  light2->setType(Ogre::Light::LT_SPOTLIGHT);
  light2->setDirection(Ogre::Vector3(-0.3,-1,0));
  light2->setSpotlightInnerAngle(Ogre::Degree(25.0f));
  light2->setSpotlightOuterAngle(Ogre::Degree(60.0f));
  light2->setSpotlightFalloff(10.0f);
  light2->setCastShadows(true);

  _changes = 0;

	_exitGame = false;

  //=============PHYSICS===========//
  // Creacion del mundo (definicion de los limites y la gravedad) ---
  AxisAlignedBox worldBounds = AxisAlignedBox (
    Vector3 (-10000, -10000, -10000), 
    Vector3 (10000,  10000,  10000));
  Vector3 gravity = Vector3(0, -9.8, 0);

  _world = new OgreBulletDynamics::DynamicsWorld(_sceneMgr,
 	   worldBounds, gravity);
  //_world->setShowDebugShapes (true);  // Muestra los collision shapes

  // Creacion de los elementos iniciales del mundo

  // Creacion del track --------------------------------------------------
  Entity *entity = _sceneMgr->createEntity("Level1Mesh.mesh");
  SceneNode *trackNode = _sceneMgr->createSceneNode("track");
  scaleMesh(entity,Vector3(2,2,2));
  trackNode->attachObject(entity);


  _sceneMgr->getRootSceneNode()->addChild(trackNode);
  OgreBulletCollisions::StaticMeshToShapeConverter *trimeshConverter = new 
    OgreBulletCollisions::StaticMeshToShapeConverter(entity);

  OgreBulletCollisions::TriangleMeshCollisionShape *trackTrimesh = 
    trimeshConverter->createTrimesh();

  OgreBulletDynamics::RigidBody *rigidTrack = new 
    OgreBulletDynamics::RigidBody("track", _world);
  rigidTrack->setShape(trackNode, trackTrimesh, 0.8, 0.95, 0, Vector3(20,-49.5,85), 
		      Quaternion(-180,0,-180,1));

  delete trimeshConverter;

  // Creacion del rebotador--------------------------------------------------
  entity = _sceneMgr->createEntity("RebotadorMesh.mesh");
  SceneNode* rebotadorNode = _sceneMgr->createSceneNode("rebotador1");
  //scaleMesh(entity,Vector3(2,2,2));
  rebotadorNode->attachObject(entity);


  _sceneMgr->getRootSceneNode()->addChild(rebotadorNode);
  rebotadorNode->setPosition(Vector3(-47,-33,71));
  rebotadorNode->setScale(Vector3(2,2,2));
/*  trimeshConverter = new 
    OgreBulletCollisions::StaticMeshToShapeConverter(entity);

  trackTrimesh = 
    trimeshConverter->createTrimesh();

  rigidTrack = new 
    OgreBulletDynamics::RigidBody("track", _world);
  rigidTrack->setShape(rebotadorNode, trackTrimesh, 0.8, 0.95, 0, Vector3(20,-49.5,85), 
		      Quaternion(-180,0,-180,1));

  delete trimeshConverter;*/
  // Creacion de la entidad y del SceneNode ------------------------
  Plane plane1(Vector3(0,1,0), -50);    // Normal y distancia
  MeshManager::getSingleton().createPlane("plane1",
	ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane1,
	200, 200, 1, 1, true, 1, 20, 20, Vector3::UNIT_Z);
  SceneNode* node = _sceneMgr->createSceneNode("ground");
  Entity* groundEnt = _sceneMgr->createEntity("planeEnt", "plane1");
  groundEnt->setMaterialName("Ground");
  node->attachObject(groundEnt);
  _sceneMgr->getRootSceneNode()->addChild(node);

  // Creamos forma de colision para el plano ----------------------- 
  OgreBulletCollisions::CollisionShape *Shape;
  Shape = new OgreBulletCollisions::StaticPlaneCollisionShape
   (Ogre::Vector3(0,1,0), -50);   // Vector normal y distancia
  OgreBulletDynamics::RigidBody *rigidBodyPlane = new 
  OgreBulletDynamics::RigidBody("rigidBodyPlane", _world);

  // Creamos la forma estatica (forma, Restitucion, Friccion) ------
  rigidBodyPlane->setStaticShape(Shape, 0.1, 0.8); 
  
	//Player Initialization
	Ogre::Entity* ent1 = _sceneMgr->createEntity("Robotillo", "RobotilloMesh.mesh");
	ent1->setQueryFlags(PLAYER);
  	std::shared_ptr<SceneNode> player(_sceneMgr->createSceneNode("Player"));
	_player = player;
	_player->attachObject(ent1);
	_sceneMgr->getRootSceneNode()->addChild(_player.get());
	_player->setScale(1,1,1);
	//_player->setPosition(-10,-47,-40);

  //DEBUG ONLY Coordinator Situate
	Ogre::Entity* ent2 = _sceneMgr->createEntity("DEBUG SEE", "RobotilloMesh.mesh");
  	std::shared_ptr<SceneNode> visor(_sceneMgr->createSceneNode("DEBUG"));
	_coordVisor = visor;
	_coordVisor->attachObject(ent2);
	_sceneMgr->getRootSceneNode()->addChild(_coordVisor.get());
	_coordVisor->setScale(1,1,1);
	_coordVisor->setPosition(0,-40,0);
  //NOT DEBUG

  OgreBulletCollisions::BoxCollisionShape *boxShape = new 
    OgreBulletCollisions::BoxCollisionShape(Vector3(2,2,2));
 // and the Bullet rigid body
 rigidBoxPlayer = new 
    OgreBulletDynamics::RigidBody("rrigidBoxPlayer" + 
       StringConverter::toString(2), _world);

 _camera->lookAt(Vector3(-10,-47,47));
  rigidBoxPlayer->setShape(_player.get(), boxShape,
		     0.6 /* Restitucion */, 0.6 /* Friccion */,
		     5.0 /* Masa */, Vector3(-40,-47,87)/* Posicion inicial */,
		     Quaternion(0,0,-180,1) /* Orientacion */);
 rigidBoxPlayer->getBulletRigidBody()->setLinearFactor(btVector3(1,1,1));
 rigidBoxPlayer->getBulletRigidBody()->setAngularFactor(btVector3(0,1,0));
	//Robot Animation
	//_animBlender = new AnimationBlender(_sceneMgr->getEntity("Robotillo"));
  
  _animationUpdater = std::make_shared<AnimationUpdater>(_player);
  _inputHandler = std::make_shared<InputHandler>(_camera,_player);

  // Anadimos los objetos Shape y RigidBody ------------------------
 // _shapes.push_back(Shape);
  _bodies.push_back(rigidBodyPlane);

   _forward = false;
   _back = false;
   _left = false;
   _right = false;
   _ball = false;
   _firstperson = false;
   _leftShooting = false;
   _win = false;
   _cameraZoom = 0;
   _newtons = 0;
}

void
PlayState::exit ()
{
  _sceneMgr->clearScene();
  _root->getAutoCreatedWindow()->removeAllViewports();
}

void
PlayState::pause()
{
}

void
PlayState::resume()
{
  // Se restaura el background colour.
  _viewport->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 1.0));
}

bool
PlayState::frameStarted
(const Ogre::FrameEvent& evt)
{ 
  //_animBlender->addTime(evt.timeSinceLastFrame);
  _lastTime= evt.timeSinceLastFrame;
  _world->stepSimulation(_lastTime); // Actualizar simulacion Bullet

  //Win Logic
  if(4 > _player->getPosition().distance(Vector3(-95,-28,31))){
    _win = true;
    std::cout << "Win Condition!" << std::endl;
    Overlay *overlay = _overlayManager->getByName("Victory");
    overlay->show();
  }
  //Bouncer Logic
  if(5 > _player->getPosition().distance(Vector3(-47,-32,71))){
    std::cout << "impulse" << std::endl;
      rigidBoxPlayer->disableDeactivation();
      Vector3 destiny;
      destiny = _player->getPosition() - Vector3(0,10,0);
      Vector3 delta = destiny - _player->getPosition();
      Vector3 normalisedDelta = delta.normalisedCopy();
        rigidBoxPlayer->getBulletRigidBody()->
        applyCentralForce(btVector3(0,10,0)*18000*_lastTime);

  }
  

  //Movement Logic
  btVector3 playerVelocity = rigidBoxPlayer->getBulletRigidBody()->getLinearVelocity();

  Quaternion prueba = _camera->getOrientation();

  if (_forward) {
      rigidBoxPlayer->disableDeactivation();
      Vector3 destiny;
      if(!_ball)
        destiny = _player->convertLocalToWorldPosition(Vector3(0,0,-1));
      else{
        if(_firstperson){
          Vector3 direction = _camera->getDirection();
          direction.y = 0;
          destiny = _player->getPosition() +  direction * 10;
        }else{
          destiny = _player->getPosition() - Vector3(0,0,10);
        }
      }
      Vector3 delta = destiny - _player->getPosition();
      Vector3 normalisedDelta = delta.normalisedCopy();
      if(inAbsoluteRange(playerVelocity,7) || _ball){
        rigidBoxPlayer->getBulletRigidBody()->
        applyCentralForce(btVector3(normalisedDelta.x,normalisedDelta.y,normalisedDelta.z)*8000*_lastTime);
      }
  }
  if (_back) {
    rigidBoxPlayer->disableDeactivation();
    Vector3 destiny;
      if(!_ball)
        destiny = _player->convertLocalToWorldPosition(Vector3(0,0,1));
      else{
        if(_firstperson){
          Vector3 direction = _camera->getDirection();
          direction.y = 0;
          destiny = _player->getPosition() -  direction * 10;
        }else{
          destiny = _player->getPosition() - Vector3(0,0,-10);
        }
      }
    Vector3 delta = destiny - _player->getPosition();
    Vector3 normalisedDelta = delta.normalisedCopy();
    if(inAbsoluteRange(playerVelocity,7) || _ball){
      rigidBoxPlayer->getBulletRigidBody()->
      applyCentralForce(btVector3(normalisedDelta.x,-normalisedDelta.y,normalisedDelta.z)*8000*_lastTime);
    }
  }
  float maxAngular = 0.5;
  if (_left) {
    rigidBoxPlayer->disableDeactivation();
    if(!_ball){
      float velocity = rigidBoxPlayer->getBulletRigidBody()->getAngularVelocity().y();
      if(velocity < maxAngular){
        rigidBoxPlayer->getBulletRigidBody()->
          applyTorque(btVector3(0,200,0));
      }
    }else{
      Vector3 destiny;
      if(_firstperson){
          Vector3 direction = _camera->getDirection();
          direction.y = 0;
          destiny = _player->getPosition() - Quaternion(Degree(-90),Vector3::UNIT_Y) * direction * 10;
      }else{
        destiny = _player->getPosition() - Vector3(10,0,0);
      }
      Vector3 delta = destiny - _player->getPosition();
      Vector3 normalisedDelta = delta.normalisedCopy();
      if(inAbsoluteRange(playerVelocity,7) || _ball){
        rigidBoxPlayer->getBulletRigidBody()->
        applyCentralForce(btVector3(normalisedDelta.x,-normalisedDelta.y,normalisedDelta.z)*8000*_lastTime);
      }
    }
  }
  if (_right) {
    if(!_ball){
      rigidBoxPlayer->disableDeactivation();
      float velocity = rigidBoxPlayer->getBulletRigidBody()->getAngularVelocity().y();
      if(velocity > -maxAngular){
      rigidBoxPlayer->getBulletRigidBody()->
        applyTorque(btVector3(0,-200,0));
      }
    }else{
      Vector3 destiny;
      if(_firstperson){
          Vector3 direction = _camera->getDirection();
          direction.y = 0;
          destiny = _player->getPosition() - Quaternion(Degree(90),Vector3::UNIT_Y) * direction * 10;
      }else{
          destiny = _player->getPosition() - Vector3(-10,0,0);
      }
      Vector3 delta = destiny - _player->getPosition();
      Vector3 normalisedDelta = delta.normalisedCopy();
      if(inAbsoluteRange(playerVelocity,7) || _ball){
        rigidBoxPlayer->getBulletRigidBody()->
        applyCentralForce(btVector3(normalisedDelta.x,-normalisedDelta.y,normalisedDelta.z)*8000*_lastTime);
      }
    }
  }
  // Shoot Logic
  if(_ball && _firstperson && _leftShooting){
    _newtons += evt.timeSinceLastFrame ;
    OverlayElement *oe;
    oe = _overlayManager->getOverlayElement("segundos");
    std::ostringstream string;
    string << _newtons*100;
    oe->setCaption(string.str());
  }
  if(_ball && _firstperson && !_leftShooting && _newtons != 0){
      rigidBoxPlayer->disableDeactivation();
      Vector3 destiny;
      Vector3 direction = _camera->getDirection();
      destiny = _player->getPosition() +  direction * 10;
      Vector3 delta = destiny - _player->getPosition();
      Vector3 normalisedDelta = delta.normalisedCopy();
      if(inAbsoluteRange(playerVelocity,7) || _ball){
        rigidBoxPlayer->getBulletRigidBody()->
        applyCentralForce(btVector3(normalisedDelta.x,normalisedDelta.y,normalisedDelta.z)
            *2000*_newtons);
                        (_pSoundFXManager->load("sqau.wav"))->play();
      }
      _newtons = 0;
  }
  
  _animationUpdater->update(evt);
  _inputHandler->update(evt,_player->getPosition(),_ball, _firstperson, _cameraZoom);
  return true;
}

bool
PlayState::frameEnded
(const Ogre::FrameEvent& evt)
{
  if (_exitGame)
    return false;
  
  Real deltaT = evt.timeSinceLastFrame;
  _world->stepSimulation(deltaT); // Actualizar simulacion Bullet
  return true;
}

void
PlayState::keyPressed
(const OIS::KeyEvent &e)
{
  // Tecla p --> PauseState.
  if (e.key == OIS::KC_P) {
    pushState(PauseState::getSingletonPtr());
    _exitGame = true;
  }
  if (e.key == OIS::KC_W) {
    _forward = true;
  }
  if (e.key == OIS::KC_S) {
    _back = true;
  }

  if (e.key == OIS::KC_A) {
    _left = true;
  }
  if (e.key == OIS::KC_D) {
    _right = true;
  }
  if (e.key == OIS::KC_E) {
    _ball = !_ball;
    if(_ball){
      OgreBulletCollisions::CollisionShape *ballShape = new 
        OgreBulletCollisions::SphereCollisionShape(1.5);

      Vector3 samePosition = rigidBoxPlayer->getSceneNode()->getPosition();
      _changes++;

      rigidBoxPlayer = new 
        OgreBulletDynamics::RigidBody("rigidBoxPlayer" + 
           StringConverter::toString(_changes), _world);
      rigidBoxPlayer->setShape(_player.get(), ballShape,
             0.6 /* Restitucion */, 0.6 /* Friccion */,
             5.0 /* Masa */, samePosition/* Posicion inicial */,
             Quaternion(0,0,-180,1) /* Orientacion */);
    }else{
      _firstperson = false;
      OgreBulletCollisions::BoxCollisionShape *boxShape = new 
        OgreBulletCollisions::BoxCollisionShape(Vector3(2,2,2));

      Vector3 samePosition = rigidBoxPlayer->getSceneNode()->getPosition();
      samePosition += Vector3(0,4,0);
      _changes++;

      rigidBoxPlayer = new 
        OgreBulletDynamics::RigidBody("rigidBox" + 
           StringConverter::toString(_changes), _world);

      rigidBoxPlayer->setShape(_player.get(), boxShape,
             0.6 /* Restitucion */, 0.6 /* Friccion */,
             5.0 /* Masa */, samePosition/* Posicion inicial */,
             Quaternion(0,0,-180,1) /* Orientacion */);
 rigidBoxPlayer->getBulletRigidBody()->setLinearFactor(btVector3(1,1,1));
 rigidBoxPlayer->getBulletRigidBody()->setAngularFactor(btVector3(0,1,0));
//   rigidBoxPlayer->getBulletRigidBody()->setLinearFactor(btVector3(1,1,1));
 //  rigidBoxPlayer->getBulletRigidBody()->setAngularFactor(btVector3(0,1,0));
    }


  
  } 

  if (e.key == OIS::KC_Q) {
      std::cout << "heh"<< std::endl;
    if(!_firstperson && _ball){
      _firstperson = true;
      _player->setVisible(false);
      std::cout << "false"<< std::endl;
    }else if(_firstperson && _ball){
      _firstperson = false;
      _player->setVisible(true);
    }
  }
  if (e.key == OIS::KC_UP) {
    _cameraZoom += 1;
  }
  if (e.key == OIS::KC_DOWN) {
    _cameraZoom -= 1;
  }
  // === DEBUG === //
  

  Vector3 position = _coordVisor->getPosition();
  if (e.key == OIS::KC_NUMPAD1) {
    _coordVisor->translate(Vector3(1,0,0));
  }
  if (e.key == OIS::KC_NUMPAD2) {
    _coordVisor->translate(Vector3(-1,0,0));
  }
  if (e.key == OIS::KC_NUMPAD4) {
    _coordVisor->translate(Vector3(0,1,0));
  }
  if (e.key == OIS::KC_NUMPAD5) {
    _coordVisor->translate(Vector3(0,-1,0));
  }
  if (e.key == OIS::KC_NUMPAD7) {
    _coordVisor->translate(Vector3(0,0,1));
  }
  if (e.key == OIS::KC_NUMPAD8) {
    _coordVisor->translate(Vector3(0,0,-1));
  }
  if (e.key == OIS::KC_NUMPAD0) {
    Vector3 position = _coordVisor->getPosition();
    std::cout << 
      "X: " << position.x << std::endl << 
      "Y: " << position.y << std::endl << 
      "Z: " << position.z << std::endl;
  }
  //light->setPosition(position);
  _animationUpdater->keyPressed(e);
 // _inputHandler->keyPressed(e);
}

void
PlayState::keyReleased
(const OIS::KeyEvent &e)
{
  if (e.key == OIS::KC_ESCAPE) {
    _exitGame = true;
  }
  
  if (e.key == OIS::KC_W) {
    _forward = false;
  }
  if (e.key == OIS::KC_S) {
    _back = false;
  }
  if (e.key == OIS::KC_A) {
    _left = false;
  }
  if (e.key == OIS::KC_D) {
    _right = false;
  }
  _animationUpdater->keyReleased(e);
  _inputHandler->keyReleased(e);
}

void
PlayState::mouseMoved
(const OIS::MouseEvent &e)
{
  _inputHandler->mouseMoved(e);
}

void
PlayState::mousePressed
(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
  if (id == OIS::MB_Left) {
    _leftShooting = true;
  }
}

void
PlayState::mouseReleased
(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
  if (id == OIS::MB_Left) {
    _leftShooting = false;
  }
}

PlayState*
PlayState::getSingletonPtr ()
{
return msSingleton;
}

PlayState&
PlayState::getSingleton ()
{ 
  assert(msSingleton);
  return *msSingleton;
}

double PlayState::getTimeSinceLastTime(){
  return _lastTime;
}

bool PlayState::initSDL () {
 // Inicializando SDL...
 if (SDL_Init(SDL_INIT_AUDIO) < 0)
 return false;
 // Llamar a SDL_Quit al terminar.
 atexit(SDL_Quit);

 // Inicializando SDL mixer...
 if (Mix_OpenAudio(MIX_DEFAULT_FREQUENCY, MIX_DEFAULT_FORMAT,MIX_DEFAULT_CHANNELS, 4096) < 0)
 return false;

 // Llamar a Mix_CloseAudio al terminar.
 atexit(Mix_CloseAudio);

 return true;
}



