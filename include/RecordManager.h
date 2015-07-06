
#ifndef RecordManager_H
#define RecordManager_H

#include <Ogre.h>
#include <OIS/OIS.h>
#include <algorithm>    // std::sort
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "GameState.h"

#define PLAYER 1 << 0  // Mascara para el escenario
#define CUBE1 1 << 1  // Mascara para objetos de tipo 1
#define CUBE2 1 << 2  // Mascara para objetos de tipo 2

class RecordManager {
	
 public:
	RecordManager ();
	
	//Record Logic
	std::vector<int> getRecords();
	void keepRecord(int points, int level);
	
	//Points logic
	int getPoints();
	void addPoints(int points);

  

 protected:
 	int _points;


  
};

#endif
