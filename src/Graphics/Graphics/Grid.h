#pragma once

#include <string>
#include "glad.h"
#include "Shader.h"
#include "Buffer.h"
#include "Input.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/glm.hpp>

class Grid
{
public:
    Grid();
    ~Grid();

    void Update();
    void Render(const Shader &shader);

private:

    Buffer m_buffer;

    glm::mat4 m_model;
    glm::vec3 m_position;
    glm::vec3 m_rotation;
};