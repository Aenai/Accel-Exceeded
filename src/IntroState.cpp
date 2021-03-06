#include "IntroState.h"
#include "PlayState.h"

template<> IntroState* Ogre::Singleton<IntroState>::msSingleton = 0;


std::vector<std::string> IntroState::getRecords(){
	
	std::vector<std::string> records;
	
  std::string line;
  std::ifstream myfile ("records.txt");
	if (myfile.is_open()){
		while ( getline (myfile,line) ){
			records.push_back(line);
		}
	myfile.close();
	}
	
	int bestRecord=999999, indexRecord=-1;
	std::vector<std::string> top;
	for(int j=0; j<3 && !records.empty(); j++){
		for(unsigned int i=0; i<records.size(); i++){
			int boxes= atoi((split(records.at(i),':')[0]).c_str())/1000;
      //std::cout << boxes << " en la bussqueda" << std::endl;
			if(boxes<bestRecord && boxes != 999999){
				bestRecord=boxes;
				indexRecord=i;
			}
				
		}
		if(indexRecord != -1){
			top.push_back(records.at(indexRecord));

/*			OverlayElement *oe;
			
			std::ostringstream string;
			string << "r" << j << "_records";
			
			oe = _overlayManager->getOverlayElement(string.str());
			std::ostringstream r_string;
			r_string << bestRecord << " cajas por abrir en " << (split(records.at(indexRecord),':')[0]) << " segundos";
			oe->setCaption(r_string.str());*/
			
      if(j==0){
         firstRecord = bestRecord;
      }else if(j==1){
         secondRecord = bestRecord;
      }else{
         thirdRecord = bestRecord;
      }
      std::cout << bestRecord << " cajas por abrir en " << std::endl; 
			records[indexRecord] = "999999:999";
			indexRecord=-1;
			bestRecord=999999;
		}
	}

	
	return records;
	
}

std::vector<std::string> &IntroState::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> IntroState::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void
IntroState::enter ()
{
  _root = Ogre::Root::getSingletonPtr();

  _sceneMgr = _root->createSceneManager(Ogre::ST_GENERIC, "SceneManager");
  _camera = _sceneMgr->createCamera("IntroCamera");
  _viewport = _root->getAutoCreatedWindow()->addViewport(_camera);
  _viewport->setBackgroundColour(Ogre::ColourValue(1.0, 1.0, 1.0));

  _overlayManager = Ogre::OverlayManager::getSingletonPtr();
  Ogre::Overlay *overlay = _overlayManager->getByName("Info");
  overlay->show();

  renderer = &CEGUI::OgreRenderer::bootstrapSystem();
  createGUI();
  initMenu();

  std::cout << "asucar" << std::endl;
  getRecords();
  _exitGame = false;
}

void
IntroState::exit()
{
  _sceneMgr->clearScene();
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
    CEGUI::MouseCursor::getSingleton().hide( );
    CEGUI::WindowManager::getSingletonPtr()->destroyAllWindows();
    Ogre::Overlay *overlay = _overlayManager->getByName("Info");
    overlay->hide();	
    changeState(PlayState::getSingletonPtr());
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

	CEGUI::SchemeManager::getSingleton().create("Excegui.scheme");
	CEGUI::ImagesetManager::getSingleton().create("Excegui.imageset");
	//CEGUI::System::getSingleton().setDefaultFont("tenby-five");
	if(! CEGUI::FontManager::getSingleton().isDefined( "Epyval" ) )
	CEGUI::FontManager::getSingleton().createFreeTypeFont( "Epyval", 19, true, "Epyval.ttf", "Fonts" );
	CEGUI::System::getSingleton().setDefaultFont( "Epyval" );
	CEGUI::System::getSingleton().setDefaultMouseCursor("Excegui","MouseArrow");
	CEGUI::MouseCursor::getSingleton().show( );
}

void IntroState::initMenu(){
	//Sheet
	CEGUI::Window* sheet = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow","MenuWin");

	//Config Window
	CEGUI::Window* formatWin = CEGUI::WindowManager::getSingleton().loadWindowLayout("InitMenu.layout");

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
	CEGUI::Window* formatWin = CEGUI::WindowManager::getSingleton().loadWindowLayout("Records.layout");

	//Setting Text!
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text1")->setText("[vert-alignment='centre'] [colour='FF000000'] Records:");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text2")->setText("[vert-alignment='centre'] [colour='FF000000']  - 1: " + StringConverter::toString(firstRecord) );
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text3")->setText("[vert-alignment='centre'] [colour='FF000000']  - 2: " + StringConverter::toString(secondRecord));
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text4")->setText("[vert-alignment='centre'] [colour='FF000000']  - 3: " + StringConverter::toString(thirdRecord));

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
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text1")->setText("[vert-alignment='centre'] [colour='FF000000'] Creado por:");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text2")->setText("[vert-alignment='centre'] [colour='FF000000']  - Juan Carlos Fernandez Duran");
	CEGUI::WindowManager::getSingleton().getWindow("FormatWin/Text3")->setText("[vert-alignment='centre'] [colour='FF000000']  - Ivan Martinez Heras");

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
    	Ogre::Overlay *overlay = _overlayManager->getByName("Info");
    	overlay->hide();
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
