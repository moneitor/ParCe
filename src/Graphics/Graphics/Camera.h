#pragma once

#include <glm/glm.hpp>
#include "glad.h"
#include "Shader.h"
#include <glm/gtc/matrix_transform.hpp>
#include "Input.h"
#include "Shader.h"

class Camera
{

public:

	Camera();

	void Projection();
	void Update();
	void SendToShader(const Shader& shader);

protected:

	glm::mat4 m_view;
	glm::mat4 m_proj;

	glm::vec3 m_position;
	glm::vec3 m_direction;
	glm::vec3 m_up;

};