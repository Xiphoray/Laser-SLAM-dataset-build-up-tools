#pragma once
#include "CImg.h"
#include <cmath>
#include <string>
using namespace cimg_library;
const double pi(3.14159265);
class Showmap
{
public:
	Showmap();
	~Showmap();
	void Todisplay();
	void change();


private:
	int i = 0;
	CImg<unsigned char> SrcImg, window; //定义一副图片
	CImgDisplay main_disp, draw_disp;
};
