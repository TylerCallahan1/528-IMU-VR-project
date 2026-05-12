#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <glad/glad.h>
#include<GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include "udp_receiver.hpp"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

const char* vertexShaderSource = "#version 330 core \n"
"layout (location = 0) in vec3 aPos;\n"
"layout (location = 1) in vec3 aColor;\n"
"uniform mat4 model;\n"
"uniform mat4 view;\n"
"uniform mat4 projection;\n"
"out vec3 vertexColor;\n"
"void main()\n"
"{\n"
	"gl_Position = projection * view * model * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
	"vertexColor = aColor;\n"
"}\0";

const char* fragmentShaderSource = "#version 330 core\n"
"out vec4 FragColor;\n"
"in vec3 vertexColor;\n"
"void main()\n"
"{\n"
	"FragColor = vec4(vertexColor,1.0);\n"
"}\n";

IMUData baseline;
bool baselineSet = false;


int main() {

	//initialize glfw and make OpenGL window object
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_SAMPLES, 4);

	GLFWwindow* window = glfwCreateWindow(1200, 1000, "LearnOpenGL", NULL, NULL);

	if (window == NULL) {
		std::cout << "Failed to create window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	//initialize GLAD, tells glad the address of opengl function pointers

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD";
		return -1;
	}


	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);



	//creates and compiles a vertex shader that maps 1 to 1  
	unsigned int vertexShader;
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);

	//frag shader
	unsigned int fragmentShader;
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);

	//linking shaders

	unsigned int shaderProgram;
	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);


	glUseProgram(shaderProgram);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);









	//creates a buffer that can be sent to the vertex shader pipeline. Puts info in gpu
	float vertices[] = {
		// Front face - coral
		-0.5f, -0.5f,  0.5f,   1.0f, 0.5f, 0.31f,
		 0.5f, -0.5f,  0.5f,   1.0f, 0.5f, 0.31f,
		 0.5f,  0.5f,  0.5f,   1.0f, 0.5f, 0.31f,
		 0.5f,  0.5f,  0.5f,   1.0f, 0.5f, 0.31f,
		-0.5f,  0.5f,  0.5f,   1.0f, 0.5f, 0.31f,
		-0.5f, -0.5f,  0.5f,   1.0f, 0.5f, 0.31f,

		// Back face - teal
		-0.5f, -0.5f, -0.5f,   0.2f, 0.7f, 0.7f,
		 0.5f,  0.5f, -0.5f,   0.2f, 0.7f, 0.7f,
		 0.5f, -0.5f, -0.5f,   0.2f, 0.7f, 0.7f,
		 0.5f,  0.5f, -0.5f,   0.2f, 0.7f, 0.7f,
		-0.5f, -0.5f, -0.5f,   0.2f, 0.7f, 0.7f,
		-0.5f,  0.5f, -0.5f,   0.2f, 0.7f, 0.7f,

		// Left face - gold
		-0.5f,  0.5f,  0.5f,   1.0f, 0.84f, 0.0f,
		-0.5f,  0.5f, -0.5f,   1.0f, 0.84f, 0.0f,
		-0.5f, -0.5f, -0.5f,   1.0f, 0.84f, 0.0f,
		-0.5f, -0.5f, -0.5f,   1.0f, 0.84f, 0.0f,
		-0.5f, -0.5f,  0.5f,   1.0f, 0.84f, 0.0f,
		-0.5f,  0.5f,  0.5f,   1.0f, 0.84f, 0.0f,

		// Right face - lavender
		 0.5f,  0.5f,  0.5f,   0.7f, 0.5f, 0.9f,
		 0.5f, -0.5f, -0.5f,   0.7f, 0.5f, 0.9f,
		 0.5f,  0.5f, -0.5f,   0.7f, 0.5f, 0.9f,
		 0.5f, -0.5f, -0.5f,   0.7f, 0.5f, 0.9f,
		 0.5f,  0.5f,  0.5f,   0.7f, 0.5f, 0.9f,
		 0.5f, -0.5f,  0.5f,   0.7f, 0.5f, 0.9f,

		 // Bottom face - mint
		 -0.5f, -0.5f, -0.5f,   0.6f, 0.95f, 0.7f,
		  0.5f, -0.5f, -0.5f,   0.6f, 0.95f, 0.7f,
		  0.5f, -0.5f,  0.5f,   0.6f, 0.95f, 0.7f,
		  0.5f, -0.5f,  0.5f,   0.6f, 0.95f, 0.7f,
		 -0.5f, -0.5f,  0.5f,   0.6f, 0.95f, 0.7f,
		 -0.5f, -0.5f, -0.5f,   0.6f, 0.95f, 0.7f,

		 // Top face - salmon
		 -0.5f,  0.5f, -0.5f,   1.0f, 0.6f, 0.6f,
		  0.5f,  0.5f,  0.5f,   1.0f, 0.6f, 0.6f,
		  0.5f,  0.5f, -0.5f,   1.0f, 0.6f, 0.6f,
		  0.5f,  0.5f,  0.5f,   1.0f, 0.6f, 0.6f,
		 -0.5f,  0.5f, -0.5f,   1.0f, 0.6f, 0.6f,
		 -0.5f,  0.5f,  0.5f,   1.0f, 0.6f, 0.6f,
	};

	unsigned int VBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);


	//telling gpu how to interperate vertex data
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
		(void*)0);
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
		(void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);


	//unbinding VBO and VAO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);


	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);


	UDPReceiver imu(5005);
	if (!imu.start()) {
		std::cout << "Failed to start UDP receiver" << std::endl;
		return -1;
	}




	//main render loop, polls for events and swaps the color buffer
	while (!glfwWindowShouldClose(window)) {
		//input
		processInput(window);

		//rendering here
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		glUseProgram(shaderProgram);

		//PROJECT LOGIC BELOW


		IMUData d = imu.getData();

		// Capture first valid packet as the zero point
		if (d.valid && !baselineSet) {
			baseline = d;
			baselineSet = true;
		}

		glm::mat4 model = glm::mat4(1.0f);
		if (d.valid && baselineSet) {
			// Offset everything relative to the baseline
			float dx = d.x - baseline.x;
			float dy = d.y - baseline.y;
			float dz = d.z - baseline.z;

			float dRoll = d.roll - baseline.roll;
			float dPitch = d.pitch - baseline.pitch;
			float dYaw = d.yaw - baseline.yaw;

			float scale = 5.0f;

			model = glm::translate(model, glm::vec3(dx * scale, dz * scale,  -1 * dy * scale));

			model = glm::rotate(model, glm::radians(dRoll), glm::vec3(1.0f, 0.0f, 0.0f));
			model = glm::rotate(model, glm::radians(dYaw), glm::vec3(0.0f, 1.0f, 0.0f));
			model = glm::rotate(model, glm::radians(-1 * dPitch), glm::vec3(0.0f, 0.0f, 1.0f));
		}


		//IMUData d = imu.getData();

		//glm::mat4 model = glm::mat4(1.0f);
		//if (d.valid) {
		//	// Position: translate the cube based on IMU x/y/z
		//	float scale = 5.0f;  // tweak this; metres are tiny on screen
		//	model = glm::translate(model, glm::vec3(d.x * scale, d.y * scale, d.z * scale));

		//	// Orientation: apply yaw, pitch, roll in order
		//	model = glm::rotate(model, glm::radians(d.yaw), glm::vec3(0.0f, 1.0f, 0.0f));
		//	model = glm::rotate(model, glm::radians(d.pitch), glm::vec3(1.0f, 0.0f, 0.0f));
		//	model = glm::rotate(model, glm::radians(d.roll), glm::vec3(0.0f, 0.0f, 1.0f));
		//}



















		//PROJECT LOGIC ABOVE
		glm::mat4 view = glm::mat4(1.0f);
		view = glm::translate(view, glm::vec3(0.0f, 0.0f, -5.0f));

		glm::mat4 projection;
		projection = glm::perspective(glm::radians(45.0f), 1200.0f / 1000.0f, 0.1f,
			100.0f);

		int modelLoc = glGetUniformLocation(shaderProgram, "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

		int viewLoc = glGetUniformLocation(shaderProgram, "view");
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

		int projLoc = glGetUniformLocation(shaderProgram, "projection");
		glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

		//render shape
		glBindVertexArray(VAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		//check and call events and swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();
	}




	glfwTerminate();
	return 0;

	
}


void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
	}
	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
		baselineSet = false;
	}
}
