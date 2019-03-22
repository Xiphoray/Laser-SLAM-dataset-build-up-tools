#include "showmap.h"
#include "map.h"


bool cmp(double x, double y);

Showmap::Showmap() {
	CImg<unsigned char> s(MAP_HEIGHT, MAP_WIDTH, 1, 1);
	SrcImg = s;
	SrcImg.fill(255);
	main_disp.assign(SrcImg);
}

Showmap::~Showmap() {}

void Showmap::Todisplay() {

	cimg_forXY(SrcImg, x, y) {
		SrcImg(x, y, 0) = rawmap[x][y] * 255;
	}
	

	SrcImg.display(main_disp);

	
}