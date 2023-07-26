#pragma once

#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include "glad.h"

class Screen
{

public:

	static Screen* Instance();

	bool Initialize();
	void ClearScreen();
	void Present();
	void Shutdown();

private:

	Screen();
	Screen(const Screen&);
	Screen& operator=(const Screen&);

	SDL_Window* window;
	SDL_GLContext context;

};