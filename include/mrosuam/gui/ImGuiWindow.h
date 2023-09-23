#pragma once

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>

#include <implot.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <iostream>

#pragma comment(lib, "legacy_stdio_definitions");

namespace mrosuam::gui
{

    const ImVec4 CLEAR_COLOUR(0.45f, 0.55f, 0.60f, 1.00f);

    static void glfw_error_callback(int error, const char* description)
    {
        fprintf(stderr, "Glfw Error %d: %s\n", error, description);
    }

    class ImGuiWindow
    {
    public:
        
        
        
        bool setupWindow()
        {
            glfwSetErrorCallback(glfw_error_callback);
            
            std::cout << "Setting up window" << std::endl;
            
            if (glfwInit() == GLFW_FALSE)
                return 1;

            //const char* glsl_version = "#version 130";
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

            window = glfwCreateWindow(1800, 1000, "Rosuam test window", NULL, NULL);
            if (window == nullptr)
                return 1;
            
            glfwMakeContextCurrent(window);
            glfwSwapInterval(0);

            if (!gladLoadGL())
                return 1;

            IMGUI_CHECKVERSION();

            ImGui::CreateContext();
            ImPlot::CreateContext();

            ImGuiIO& io = ImGui::GetIO();
            (void)io;
            io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

            ImGui_ImplGlfw_InitForOpenGL(window, true);
            ImGui_ImplOpenGL2_Init();

            ImGuiStyle& style = ImGui::GetStyle();
            style.WindowRounding = 0.0f;
            style.Colors[ImGuiCol_WindowBg].w = 1.0f;
            
            return 0;
        }

        void beginFrame()
        {
            glfwPollEvents();

            ImGui_ImplOpenGL2_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            
            ImGuiDockNodeFlags dockspaceFlags = ImGuiDockNodeFlags_None;
            
            ImGuiWindowFlags dockWindowFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar;
            dockWindowFlags |= ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
            dockWindowFlags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
            static bool dockspaceOpen = true;
            
            ImGuiViewport* mainViewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(mainViewport->Pos);
            ImGui::SetNextWindowSize(mainViewport->Size);
            ImGui::SetNextWindowViewport(mainViewport->ID);
            
            ImGui::Begin("Dockspace window", &dockspaceOpen, dockWindowFlags);
            
            ImGuiID dockspaceID = ImGui::GetID("MyDockspace");
            ImGui::DockSpace(dockspaceID, {0, 0}, dockspaceFlags);
        }

        void endFrame()
        {
            // End dockspace window
            ImGui::End();
            
            ImGui::Render();
            int display_w, display_h;
            glfwGetFramebufferSize(window, &display_w, &display_h);
            glViewport(0, 0, display_w, display_h);
            glClearColor(CLEAR_COLOUR.x * CLEAR_COLOUR.w, CLEAR_COLOUR.y * CLEAR_COLOUR.w, CLEAR_COLOUR.z * CLEAR_COLOUR.w, CLEAR_COLOUR.w);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);

            glfwSwapBuffers(window);
        }

        void destroyWindow()
        {
            ImGui_ImplOpenGL2_Shutdown();
            ImGui_ImplGlfw_Shutdown();

            ImGui::DestroyContext();

            glfwDestroyWindow(window);
            glfwTerminate();
        }
        
        bool shouldClose()
        {
            return glfwWindowShouldClose(window);
        }
        
    private:
        
        GLFWwindow* window = nullptr;
        
    };
    
}
