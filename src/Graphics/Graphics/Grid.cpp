#include "Grid.h"

Grid::Grid()
    :m_position{glm::vec3(0.0f)}, m_rotation{glm::vec3(0.0f)}
{
    const auto SIZE = 25;
    const auto QUADRANTS = 4;
    const auto VERTICES_PER_LINE = 2;

    const auto BYTES_PER_LINE_VERTEX = VERTICES_PER_LINE * static_cast<GLuint>(Buffer::ComponentType::XYZ) * sizeof(GLint);
    const auto BYTES_PER_LINE_COLOR = VERTICES_PER_LINE * static_cast<GLuint>(Buffer::ComponentType::RGBA) * sizeof(GLfloat);

    const auto BYTES_VERTEX_VBO = (SIZE + 1) * QUADRANTS * BYTES_PER_LINE_VERTEX;
    const auto BYTES_COLOR_VBO = (SIZE + 1) * QUADRANTS * BYTES_PER_LINE_COLOR;

    m_buffer.CreateBuffer((SIZE + 1) * QUADRANTS * VERTICES_PER_LINE, false);

    m_buffer.FillVBO(Buffer::VBOType::VertexBuffer, (GLint*)nullptr, BYTES_VERTEX_VBO, Buffer::FillType::Ongoing);
    m_buffer.FillVBO(Buffer::VBOType::ColorBuffer, (GLfloat*)nullptr, BYTES_COLOR_VBO, Buffer::FillType::Ongoing);

    GLuint offsetColor = 0;
    GLuint offsetVertex = 0;

    // Negative vertical lines
    for (int i = 0; i <= SIZE; i++)
    {
        GLint vertices[] = {-SIZE + i, 0, SIZE,
                            -SIZE + i, 0, -SIZE};

        GLfloat colors[] = {0.5f, 0.5f, 0.5f, 1.0f,
                            0.5f, 0.5f, 0.5f, 1.0f};

        m_buffer.AppendVBO(Buffer::VBOType::VertexBuffer, vertices, sizeof(vertices), offsetVertex);
        m_buffer.AppendVBO(Buffer::VBOType::ColorBuffer, colors, sizeof(colors), offsetColor);

        offsetVertex += BYTES_PER_LINE_VERTEX;
        offsetColor += BYTES_PER_LINE_COLOR;
    }

    // Positive vertical lines
    for (int i = 1; i < SIZE + 1; i++)
    {
        GLint vertices[] = {0 + i, 0, SIZE,
                            0 + i, 0, -SIZE};

        GLfloat colors[] = {0.5f, 0.5f, 0.5f, 1.0f,
                            0.5f, 0.5f, 0.5f, 1.0f};

        m_buffer.AppendVBO(Buffer::VBOType::VertexBuffer, vertices, sizeof(vertices), offsetVertex);
        m_buffer.AppendVBO(Buffer::VBOType::ColorBuffer, colors, sizeof(colors), offsetColor);

        offsetVertex += BYTES_PER_LINE_VERTEX;
        offsetColor += BYTES_PER_LINE_COLOR;
    }

    // Negative horizontal lines
    for (int i = 0; i <= SIZE; i++)
    {
        GLint vertices[] = {-SIZE, 0, -SIZE + i,
                             SIZE, 0, -SIZE + i};

        GLfloat colors[] = {0.5f, 0.5f, 0.5f, 1.0f,
                            0.5f, 0.5f, 0.5f, 1.0f};

        m_buffer.AppendVBO(Buffer::VBOType::VertexBuffer, vertices, sizeof(vertices), offsetVertex);
        m_buffer.AppendVBO(Buffer::VBOType::ColorBuffer, colors, sizeof(colors), offsetColor);

        offsetVertex += BYTES_PER_LINE_VERTEX;
        offsetColor += BYTES_PER_LINE_COLOR;
    }

    // Positive horizontal lines
    for (int i = 1; i < SIZE + 1; i++)
    {
        GLint vertices[] = {-SIZE, 0, 0 + i,
                             SIZE, 0, 0 + i};

        GLfloat colors[] = {0.5f, 0.5f, 0.5f, 1.0f,
                            0.5f, 0.5f, 0.5f, 1.0f};

        m_buffer.AppendVBO(Buffer::VBOType::VertexBuffer, vertices, sizeof(vertices), offsetVertex);
        m_buffer.AppendVBO(Buffer::VBOType::ColorBuffer, colors, sizeof(colors), offsetColor);

        offsetVertex += BYTES_PER_LINE_VERTEX;
        offsetColor += BYTES_PER_LINE_COLOR;
    }
}

Grid::~Grid()
{
}

void Grid::Update()
{
    if (Input::Instance()->IsLeftButtonClicked())
	{
		m_rotation.x += Input::Instance()->GetMouseMotionY();
		m_rotation.y += Input::Instance()->GetMouseMotionX();
	}

	m_model = glm::mat4(1.0f);
	m_model = glm::translate(m_model, m_position);
	m_model = glm::rotate(m_model, glm::radians(m_rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
	m_model = glm::rotate(m_model, glm::radians(m_rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));

}

void Grid::Render(const Shader &shader)
{
    shader.SendUniformData("model", m_model);
	
	shader.SendUniformData("isLit", false);
	shader.SendUniformData("isTextured", false);

	m_buffer.LinkVBO(shader, "vertexIn", Buffer::VBOType::VertexBuffer, Buffer::ComponentType::XYZ, Buffer::DataType::IntData);
	m_buffer.LinkVBO(shader, "colorIn", Buffer::VBOType::ColorBuffer, Buffer::ComponentType::RGBA, Buffer::DataType::FloatData);

	m_buffer.Render(Buffer::DrawType::Lines);

}
