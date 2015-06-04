#include "IntroState.h"
#include "PlayState.h"


template<> IntroState* Ogre::Singleton<IntroState>::msSingleton = 0;

void
IntroState::enter ()
{
  _root = Ogre::Root::getSingletonPtr();
  _sceneMgr = _root->createSceneManager(Ogre::ST_INTERIOR, "SceneManager");
  _sceneMgr -> setAmbientLight(Ogre::ColourValue(0.6, 0.6, 0.6));
  _sceneMgr -> setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

  _camera = _sceneMgr->createCamera("IntroCamera");
  _camera -> lookAt(Vector3(15,15,0));
  _camera -> setOrientation(Ogre::Quaternion::IDENTITY);
  _camera -> setPosition(80,20,-200);
  _camera -> yaw(Ogre::Degree(-122));
  _camera -> pitch(Ogre::Degree(12));

  //Camaras para pruebas
//	_camera -> setPosition(60,1000,-270);
//	_camera -> pitch(Ogre::Degree(-90));
//	_camera -> setPosition(100,160,180);
//	_camera -> pitch(Ogre::Degree(-10));


  _viewport = _root->getAutoCreatedWindow()->addViewport(_camera);
  _viewport->setBackgroundColour(Ogre::ColourValue(0, 0, 0));
  
  Ogre::Entity* ent2 = _sceneMgr->createEntity("bottom", "Level1Mesh.mesh");
  ent2->setCastShadows(true);
  Ogre::SceneNode* bottom = _sceneMgr->createSceneNode("bottom");
  bottom->attachObject(ent2);
  _sceneMgr->getRootSceneNode()->addChild(bottom);
  bottom->setScale(10,10,10);
  bottom->setPosition(0,0,0);

  //Robot Initialization
  Ogre::Entity* ent1 = _sceneMgr->createEntity("Robotillo", "RobotilloMesh.mesh");
  ent1->setCastShadows(true);
//  std::shared_ptr<SceneNode> player(_sceneMgr->createSceneNode("Player"));
  Ogre::SceneNode* player = _sceneMgr->createSceneNode("Player");
  player->attachObject(ent1);
  _sceneMgr->getRootSceneNode()->addChild(player);
  player->setScale(10,10,10);
  player->setPosition(170,9,-110);
  player->yaw(Ogre::Degree(36));

//Robot Initialization
//  Ogre::Entity* ent1 = _sceneMgr->createEntity("Robotillo", "RebotadorMesh.mesh");
//  ent1->setCastShadows(true);
//  std::shared_ptr<SceneNode> player(_sceneMgr->createSceneNode("Player"));
//  Ogre::SceneNode* player = _sceneMgr->createSceneNode("Player");
//  player->attachObject(ent1);
//  _sceneMgr->getRootSceneNode()->addChild(player);
//  player->setScale(10,10,10);
//  player->setPosition(170,9,-110);
//  player->yaw(Ogre::Degree(36));


  //Inicializamos las luces
  Ogre::Light* light1 = _sceneMgr->createLight("Light1");
  light1->setType(Ogre::Light::LT_SPOTLIGHT);
  light1->setPosition(170,120,500);
  light1->setPowerScale(60);
  light1->setDirection(Ogre::Vector3(0,0,-1));
  
  Ogre::Light* light2 = _sceneMgr->createLight("Light2");
  light2->setType(Ogre::Light::LT_SPOTLIGHT);
//  light2->setType(Ogre::Light::LT_POINT);
//  light2->setType(Ogre::Light::LT_DIRECTIONAL);
  light2->setPosition(Vector3(-200,100,100));
  light2->setPowerScale(5);
  light2->setDirection(Ogre::Vector3(1,0,-1));


//  light -> setDiffuseColour (1, 1, 1);
//  light -> setSpecularColour (1, 1, 1);
//  light->setSpotlightInnerAngle(Ogre::Degree(1.0f));
//  light->setSpotlightOuterAngle(Ogre::Degree(1.0f));
//  light->setSpotlightFalloff(10.0f);
//  light->setCastShadows(true);

  _overlayManager = OverlayManager::getSingletonPtr();
  Overlay *overlay = _overlayManager->getByName("Info");
  overlay->show();

  //Inicializamos animaciones
  _animState = _sceneMgr->getEntity("Robotillo")->getAnimationState("Standing");
// _animState = _sceneMgr->getEntity("Robotillo")->getAnimationState("Boing");
  _animState->setEnabled(false);

  //Inicializamos menu
  renderer = &CEGUI::OgreRenderer::bootstrapSystem();
  createGUI();
  initMenu();

  _initGameControl = true;
  _exitGame = false;
}

void
IntroState::exit()
{
  _sceneMgr->clearScene();
  Overlay *overlay = _overlayManager->getByName("Info");
  overlay->hide();
  _root->getAutoCreatedWindow()->removeAllViewports();
}

void
IntroState::pause ()
{
}

void
IntroState::resume ()
{
}

bool
IntroState::frameStarted
(const Ogre::FrameEvent& evt) 
{
  _timeSinceLastFrame = evt.timeSinceLastFrame;
  CEGUI::System::getSingleton().injectTimePulse(_timeSinceLastFrame);

  if (!_animState->getEnabled()) {
    _animState->setTimePosition(0.0);
    _animState->setEnabled(true);
    _animState->setLoop(true);
  }

  if (_animState->getEnabled() && !_animState->hasEnded()) 
    _animState->addTime(_timeSinceLastFrame);

  return true;
}

bool
IntroState::frameEnded
(const Ogre::FrameEvent& evt)
{
  if (_exitGame)
    return false;
  
  return true;
}

void
IntroState::keyPressed
(const OIS::KeyEvent &e)
{

  CEGUI::System::getSingleton().injectKeyDown(e.key);
  CEGUI::System::getSingleton().injectChar(e.text);

  // Transición al siguiente estado.
  // Espacio --> PlayState
  if (e.key == OIS::KC_SPACE) {
	if (_initGameControl == true){
		CEGUI::MouseCursor::getSingleton().hide( );
		CEGUI::WindowManager::getSingletonPtr()->destroyWindow("MenuWin");
		changeState(PlayState::getSingletonPtr());
	}
  }
}

void
IntroState::keyReleased
(const OIS::KeyEvent &e )
{
  CEGUI::System::getSingleton().injectKeyUp(e.key);
  if (e.key == OIS::KC_ESCAPE) {
    _exitGame = true;
  }
}

void IntroState::mouseMoved (const OIS::MouseEvent &e)
{
	CEGUI::System::getSingleton().injectMouseMove(e.state.X.rel, e.state.Y.rel);
}

void IntroState::mousePressed (const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
	CEGUI::System::getSingleton().injectMouseButtonDown(convertMouseButton(id));
}

void IntroState::mouseReleased (const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
	CEGUI::System::getSingleton().injectMouseButtonUp(convertMouseButton(id));
}

CEGUI::MouseButton IntroState::convertMouseButton(OIS::MouseButtonID id)
{
	CEGUI::MouseButton ceguiId;
	switch(id)
	{
		case OIS::MB_Left:
			ceguiId = CEGUI::LeftButton;
			break;
		case OIS::MB_Right:
			ceguiId = CEGUI::RightButton;
			break;
		case OIS::MB_Middle:
			ceguiId = CEGUI::MiddleButton;
			break;
		default:
			ceguiId = CEGUI::LeftButton;
	}
	return ceguiId;
}

IntroState*
IntroState::getSingletonPtr ()
{
return msSingleton;
}

IntroState&
IntroState::getSingleton ()
{ 
  assert(msSingleton);
  return *msSingleton;
}

void IntroState::createGUI(){
	//CEGUI

	//renderer = &CEGUI::OgreRenderer::bootstrapSystem();
	CEGUI::Scheme::setDefaultResourceGroup("Schemes");
	CEGUI::Imageset::setDefaultResourceGroup("Imagesets");
	CEGUI::Font::setDefaultResourceGroup("Fonts");
	CEGUI::WindowManager::setDefaultResourceGroup("Layouts");
	CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");

	CEGUI::SchemeManager::getSingleton().create("Physigui.scheme");
	CEGUI::ImagesetManager::getSingleton().create("Physigui.imageset");
	//CEGUI::System::getSingleton().setDefaultFont("tenby-five");
	if(! CEGUI::FontManager::getSingleton().isDefined( "tenby-five" ) )
	CEGUI::FontManager::getSingleton().createFreeTypeFont( "tenby-five", 17, true, "tenby-five.otf", "Fonts" );
	CEGUI::System::getSingleton().setDefaultFont( "tenby-five" );
	CEGUI::System::getSingleton().setDefaultMouseCursor("Physigui","MouseArrow");
	CEGUI::MouseCursor::getSingleton().show( );
}

void IntroState::initMenu(){
	//Sheet
	CEGUI::Window* sheet = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow","MenuWin");

	//Config Window
	CEGUI::Window* formatWin = CEGUI::WindowManager::getSingleton().loadWindowLayout("Init.layout");

	//Game Window
	CEGUI::Window* gameButton = CEGUI::WindowManager::getSingleton().getWindow("FormatWin/GameButton");
	gameButton->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&IntroState::initGame, this));

	//Controls Window
	CEGUI::Window* controlButton = CEGUI::WindowManager::getSingleton().getWindow("FormatWin/LoadButton");
	controlButton->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&IntroState::controls, this));

	//Designers Window
	CEGUI::Window* designersButton = CEGUI::WindowManager::getSingleton().getWindow("FormatWin/OptionButton");
	designersButton->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&IntroState::credits, this));

	//Exit Window
	CEGUI::Window* exitButton = CEGUI::WindowManager::getSingleton().getWindow("FormatWin/ExitButton");
	exitButton->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&IntroState::quit, this));
	
	//Attaching buttons
	sheet->addChildWindow(formatWin);
	CEGUI::System::getSingleton().setGUISheet(sheet);
}

void IntroState::controlMenu(){
	//Sheet
	CEGUI::Window* sheet = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow","CommandWin");

	//Config Window
	CEGUI::Window* formatWin = CEGUI::WindowManager::getSingleton().loadWindowLayout("Controls.layout");

	//Setting Text!
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text1")->setText(" [vert-alignment='centre']Controles");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text2")->setText(" [vert-alignment='centre']  P --> Pausar");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text3")->setText(" [vert-alignment='centre']  W/A/S/D --> Mover");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text4")->setText(" [vert-alignment='centre']  E --> Transformarse");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text5")->setText(" [vert-alignment='centre']  Click Izq. --> Autoimp.");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text6")->setText(" [vert-alignment='centre']  Q --> Cambiar camara");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text7")->setText(" [vert-alignment='centre']  Up/Down --> Zoom");

	//Back Window
	CEGUI::Window* backButton = CEGUI::WindowManager::getSingleton().getWindow("FormatWin/BackButton");
	backButton->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&IntroState::back, this));
	
	//Attaching buttons
	sheet->addChildWindow(formatWin);
	CEGUI::System::getSingleton().setGUISheet(sheet);
}

void IntroState::creditMenu(){
	//Sheet
	CEGUI::Window* sheet = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow","CreditWin");

	//Config Window
	CEGUI::Window* formatWin = CEGUI::WindowManager::getSingleton().loadWindowLayout("Credits.layout");

	//Setting Text!
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text1")->setText("[vert-alignment='centre'] Diseñado por:");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text2")->setText("[vert-alignment='centre']   - Juan Carlos");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text3")->setText("[vert-alignment='centre']       Fernandez Duran");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text4")->setText("[vert-alignment='centre']   - Ivan Martinez Heras");

	//Back Window
	CEGUI::Window* backButton = CEGUI::WindowManager::getSingleton().getWindow("FormatWin/BackButton");
	backButton->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&IntroState::back, this));
	
	//Attaching buttons
	sheet->addChildWindow(formatWin);
	CEGUI::System::getSingleton().setGUISheet(sheet);
}

bool IntroState::initGame(const CEGUI::EventArgs &e){
	CEGUI::MouseCursor::getSingleton().hide( );
	CEGUI::WindowManager::getSingletonPtr()->destroyAllWindows();
	changeState(PlayState::getSingletonPtr());

	return true;
}

bool IntroState::controls(const CEGUI::EventArgs &e){
	_initGameControl=false;
	CEGUI::WindowManager::getSingletonPtr()->destroyWindow("MenuWin");
	controlMenu();
	return true;
}

bool IntroState::credits(const CEGUI::EventArgs &e){
	_initGameControl=false;
	CEGUI::WindowManager::getSingletonPtr()->destroyWindow("MenuWin");
	creditMenu();
	return true;
}

bool IntroState::back(const CEGUI::EventArgs &e){
	CEGUI::WindowManager::getSingletonPtr()->destroyAllWindows();
	_initGameControl=true;
	initMenu();

	return true;
}

bool IntroState::quit(const CEGUI::EventArgs &e){
	CEGUI::WindowManager::getSingletonPtr()->destroyAllWindows();
	_exitGame = true;
	return true;
}
