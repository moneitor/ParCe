#pragma once

#include <string>
#include "glad.h"

class Texture
{

public:

	Texture();

	void Bind() const;
	bool Load(const std::string& filename);
	void Unbind();
	void Unload();

private:

	GLuint m_ID;

};