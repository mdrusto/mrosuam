#include "mrosuam/gui/AtomicVariable.h"

#include <chrono>
#include <mutex>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include <ros/ros.h>

#include <mavros_msgs/State.h>

#include <sensor_msgs/CompressedImage.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt64.h>

#include <jpgd.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>

#include <implot.h>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#pragma comment(lib, "legacy_stdio_definitions");

namespace m_rosuam::gui {

    AtomicVariable<mavros_msgs::State> currentState;
    
    AtomicVariable<std::vector<double>> rawGcsCpuUserLoad(std::vector<double>(4)), rawGcsCpuSystemLoad(std::vector<double>(4));
    AtomicVariable<std::vector<double>> averagedGcsCpuUserLoad(std::vector<double>(4)), averagedGcsCpuSystemLoad(std::vector<double>(4));
    AtomicVariable<std::vector<double>> rawObcCpuUserLoad(std::vector<double>(4)), rawObcCpuSystemLoad(std::vector<double>(4));
    AtomicVariable<std::vector<double>> averagedObcCpuUserLoad(std::vector<double>(4)), averagedObcCpuSystemLoad(std::vector<double>(4));
    
    AtomicVariable<uint64_t> gcsMemUsed, obcMemUsed;

    AtomicVariable<sensor_msgs::CompressedImage> currentImage;
    std::mutex imageMutex;
    //sensor_msgs::CompressedImage currentImage;
    //int numTimesSet = 0;

    const ImVec4 CLEAR_COLOUR(0.45f, 0.55f, 0.60f, 1.00f);

    static GLFWwindow* s_window = nullptr;

    static void glfw_error_callback(int error, const char* description)
    {
        fprintf(stderr, "Glfw Error %d: %s\n", error, description);
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& state)
    {
        currentState = *state;
    }
    
    void rawGcsCpuUserLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        rawGcsCpuUserLoad = ptr->data;
    }
    
    void rawGcsCpuSystemLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        rawGcsCpuSystemLoad = ptr->data;
    }
    
    void averagedGcsCpuUserLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        averagedGcsCpuUserLoad = ptr->data;
    }
    
    void averagedGcsCpuSystemLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        averagedGcsCpuSystemLoad = ptr->data;
    }
    
    void gcsMemUsedCallback(const std_msgs::UInt64::ConstPtr& ptr)
    {
        gcsMemUsed = ptr->data;
    }
    
    void rawObcCpuUserLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        rawObcCpuUserLoad = ptr->data;
    }
    
    void rawObcCpuSystemLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        rawObcCpuSystemLoad = ptr->data;
    }
    
    void averagedObcCpuUserLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        averagedObcCpuUserLoad = ptr->data;
    }
    
    void averagedObcCpuSystemLoadCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
    {
        averagedObcCpuSystemLoad = ptr->data;
    }
    
    void obcMemUsedCallback(const std_msgs::UInt64::ConstPtr& ptr)
    {
        obcMemUsed = ptr->data;
    }
    void cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& image)
    {
        imageMutex.lock();
        currentImage = *image;
        //std::cout << "Changing image, data point: " << currentImage.data[100] << "\n";
        imageMutex.unlock();
    }

    bool imGuiSetupWindow()
    {
        glfwSetErrorCallback(glfw_error_callback);

        if (!glfwInit())
            return 1;

        //const char* glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

        s_window = glfwCreateWindow(1800, 1000, "Rosuam test window", NULL, NULL);
        if (s_window == nullptr)
            return 1;
        
        glfwMakeContextCurrent(s_window);
        glfwSwapInterval(0);

        if (!gladLoadGL())
            return 1;

        IMGUI_CHECKVERSION();

        ImGui::CreateContext();
        ImPlot::CreateContext();

        ImGuiIO& io = ImGui::GetIO();
        (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

        ImGui_ImplGlfw_InitForOpenGL(s_window, true);
        ImGui_ImplOpenGL2_Init();

        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
        
        return 0;
    }

    void imGuiBeginFrame()
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

    void imGuiEndFrame()
    {
        // End dockspace window
        ImGui::End();
        
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(s_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(CLEAR_COLOUR.x * CLEAR_COLOUR.w, CLEAR_COLOUR.y * CLEAR_COLOUR.w, CLEAR_COLOUR.z * CLEAR_COLOUR.w, CLEAR_COLOUR.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        GLFWwindow* backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_current_context);

        glfwSwapBuffers(s_window);
    }

    void imGuiDestroyWindow()
    {
        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGlfw_Shutdown();

        ImGui::DestroyContext();

        glfwDestroyWindow(s_window);
        glfwTerminate();
    }
    
    struct ScrollingBuffer
    {
        int m_maxSize;
        int m_offset;
        ImVector<ImVec2> m_data;
        
        ScrollingBuffer(int max_size = 1000)
        {
            m_maxSize = max_size;
            m_offset = 0;
            m_data.reserve(m_maxSize);
        }
        
        void addPoint(float x, float y)
        {
            if (m_data.size() < m_maxSize)
                m_data.push_back(ImVec2(x, y));
            else
            {
                m_data[m_offset] = ImVec2(x, y);
                m_offset = (m_offset + 1) % m_maxSize;
            }
        }
        
        void erase()
        {
            if (m_data.size() > 0)
            {
                m_data.shrink(0);
                m_offset = 0;
            }
        }
    };
    
    int run(int argc, char** argv)
    {
        
        if (imGuiSetupWindow() > 0)
            return 1;
        
        GLuint renderedTexture;
        glGenTextures(1, &renderedTexture);
        glBindTexture(GL_TEXTURE_2D, renderedTexture);
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        
    #if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    #endif
        
        if (renderedTexture == 0)
        {
            std::cout << "Rendered texture was 0!\n";
            return 1;
        }
        
        ros::init(argc, argv, "gui");
        ros::NodeHandle nodeHandle;
        
        //ros::Subscriber stateSuscriber = 
        //        nodeHandle.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
        
        ros::Subscriber cameraSubsciber = 
                nodeHandle.subscribe<sensor_msgs::CompressedImage>("/raspicam_node/image/compressed", 
                10, cameraCallback);
        
        ros::Subscriber rawGcsCpuUserLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/gcs/raw/cpu_user_load", 10, rawGcsCpuUserLoadCallback);
        
        ros::Subscriber rawGcsCpuSystemLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/gcs/raw/cpu_system_load", 10, rawGcsCpuSystemLoadCallback);
        
        ros::Subscriber averagedGcsCpuUserLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/gcs/averaged/cpu_user_load", 10, averagedGcsCpuUserLoadCallback);
        
        ros::Subscriber averagedGcsCpuSystemLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/gcs/averaged/cpu_system_load", 10, averagedGcsCpuSystemLoadCallback);
        
        ros::Subscriber gcsMemUsedSubscriber = 
                nodeHandle.subscribe<std_msgs::UInt64>("/mrosuam/resources/gcs/virtual_mem_used", 10, gcsMemUsedCallback);
        
        ros::Subscriber rawObcCpuUserLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/obc/raw/cpu_user_load", 10, rawObcCpuUserLoadCallback);
        
        ros::Subscriber rawObcCpuSystemLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/obc/raw/cpu_system_load", 10, rawObcCpuSystemLoadCallback);
        
        ros::Subscriber averagedObcCpuUserLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/obc/averaged/cpu_user_load", 10, averagedObcCpuUserLoadCallback);
        
        ros::Subscriber averagedObcCpuSystemLoadSubscriber = 
                nodeHandle.subscribe<std_msgs::Float64MultiArray>("/mrosuam/resources/obc/averaged/cpu_system_load", 10, averagedObcCpuSystemLoadCallback);
        
        ros::Subscriber obcMemUsedSubscriber = 
                nodeHandle.subscribe<std_msgs::UInt64>("/mrosuam/resources/obc/virtual_mem_used", 10, obcMemUsedCallback);
        
        
        ros::AsyncSpinner spinner(4);
        
        
        spinner.start();
        
        struct sysinfo gcsMemInfo;
        sysinfo(&gcsMemInfo);
        
        uint64_t gcsTotalVirtualMem = gcsMemInfo.totalram;
        gcsTotalVirtualMem += gcsMemInfo.totalswap;
        gcsTotalVirtualMem += gcsMemInfo.mem_unit;
        
        struct sysinfo obcMemInfo;
        sysinfo(&obcMemInfo);
        
        uint64_t obcTotalVirtualMem = obcMemInfo.totalram;
        obcTotalVirtualMem += obcMemInfo.totalswap;
        obcTotalVirtualMem += obcMemInfo.mem_unit;
        
        // Main GUI loop
        
        while (!glfwWindowShouldClose(s_window))
        {
            float frameDurationMillis;
            
            {
                using namespace std::chrono;
                auto currentFrameTime = steady_clock::now();
                static auto lastFrameTime = currentFrameTime;
                frameDurationMillis = duration_cast<microseconds>(currentFrameTime - lastFrameTime).count() / 1.0e3f;
                lastFrameTime = currentFrameTime;
            }
            
            imGuiBeginFrame();
            
            //ImGui::ShowDemoWindow();
            
            ImGui::Begin("Variables");
            
            {
                
                imageMutex.lock();
                std::vector<uint8_t> imageDataVec = currentImage.get().data;
                unsigned char* imageDataPtr = (unsigned char*)&imageDataVec[0];
                
                int width = 0, height = 0, actualComps;
                
                unsigned char* decompImage = jpgd::decompress_jpeg_image_from_memory(
                    (const unsigned char*)imageDataPtr, imageDataVec.size(), &width, &height, &actualComps, 4);
                
                
                //unsigned char* decompImage = jpgd::decompress_jpeg_image_from_file(
                //    "/home/matt/catkin_ws/src/rosuam/sample.jpe", &width, &height, &actualComps, 3);
                
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, decompImage);
                
                glEnable(GL_TEXTURE_2D);
                ImGui::Image((void*)(intptr_t)renderedTexture, ImVec2(width, height), ImVec2(1, 1), ImVec2(0, 0));
                glDisable(GL_TEXTURE_2D);
                
                //glDeleteTextures(1, &renderedTexture);
                
                //ImGui::Text("Frame ID: %s", currentImage.get().header.frame_id.c_str());
                //ImGui::Text("Timestamp: %d", currentImage.get().header.stamp.nsec);
                
                imageMutex.unlock();
            }
            
            ImGui::Text("Error: %d", glGetError());
            
            ImGui::End();
            
            ImGui::Begin("Stats");
            int fps = 1000 / frameDurationMillis;
            ImGui::Text("Current frame time: %f ms (%d fps)", frameDurationMillis, fps);
            
            ImGui::Separator();
            
            ImPlotFlags memoryPlotFlags = ImPlotFlags_NoLegend;
            ImPlotAxisFlags memoryPlotXAxisFlags = ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickLabels;
            ImPlotAxisFlags memoryPlotYAxisFlags = ImPlotAxisFlags_LockMin | ImPlotAxisFlags_LockMax;
            
            {
                
                ImGui::Text("GCS Resources");
                
                std::vector<double> rawCpuUserLoad = rawGcsCpuUserLoad.get();
                std::vector<double> averagedCpuUserLoad = averagedGcsCpuUserLoad.get();
                
                uint64_t gcsMemUsedInt = gcsMemUsed.get();
                
                /*for (int i = 0; i < 4; i++)
                {
                    ImGui::Text("Cpu #%d user load: %f%%", i, rawCpuUserLoad[i] * 100);
                    ImGui::Text("Cpu #%d system load: %f%%", i, rawCpuUserLoad[i] * 100);
                }
                
                ImGui::Text("Virtual memory used: %d MB / %d MB", (int)(gcsMemUsedInt / 1e6), (int) (gcsTotalVirtualMem / 1e6));*/
                
                if (gcsMemUsedInt / gcsTotalVirtualMem > 0.95)
                {
                    ROS_INFO("GCS virtual memory in use exceeded 95%% (%lu / %lu)! Killing process.", gcsMemUsedInt, gcsTotalVirtualMem);
                    return 1;
                }
                
                static float t = 0.0f;
                const static float HISTORY = 10.0f;
                t += frameDurationMillis / 1e3f;
                
                
                // Laptop memory plot
                
                static ScrollingBuffer memScrollBuffer;
                
                memScrollBuffer.addPoint(t, gcsMemUsedInt / 1e6f);
                
                if (ImPlot::BeginPlot("GCS Virtual Memory", {-1, 150}, memoryPlotFlags))
                {
                    
                    ImPlot::SetupAxes("Time (s)", "Memory", 
                            memoryPlotXAxisFlags, 
                            memoryPlotYAxisFlags);
                    ImPlot::SetupAxisLimits(ImAxis_X1, t - HISTORY * 0.9f, t + HISTORY * 0.1f, ImGuiCond_Always);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, gcsTotalVirtualMem / 1e6f, ImGuiCond_Always);
                    
                    ImPlot::PlotLine("Cpu Usage", &memScrollBuffer.m_data[0].x, &memScrollBuffer.m_data[0].y, 
                            memScrollBuffer.m_data.size(), 0, memScrollBuffer.m_offset, 2* sizeof(float));
                    
                    ImPlot::EndPlot();
                }
                
                // Laptop Cpu plots
                
                ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 1);
                
                static std::vector<ScrollingBuffer> rawCpuUsageScrollBuffers(4), averagedCpuUsageScrollBuffers(4);
                
                for (size_t i = 0; i < 4; i++) {
                    
                    if (i == 1 || i == 3)
                        ImGui::SameLine();
                    
                    ImVec2 plotSize(100, 150);
                    
                    ImPlotAxisFlags yAxisFlags = ImPlotAxisFlags_LockMin | ImPlotAxisFlags_LockMax | ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoHighlight;
                    ImPlotAxisFlags xAxisFlags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoHighlight;
                    ImPlotFlags plotFlags = ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText | ImPlotFlags_NoMenus;
                    
                    rawCpuUsageScrollBuffers[i].addPoint(t, rawCpuUserLoad[i] * 100);
                    averagedCpuUsageScrollBuffers[i].addPoint(t, averagedCpuUserLoad[i] * 100);
                    
                    char titleFormatted[12];
                    sprintf(titleFormatted, "CPU %lu Usage", i);
                    if (ImPlot::BeginPlot(titleFormatted, plotSize, plotFlags))
                    {
                        
                        ImPlot::SetupAxes("##", "##", xAxisFlags, yAxisFlags);
                        ImPlot::SetupAxisLimits(ImAxis_X1, t - HISTORY * 0.9f, t + HISTORY * 0.1f, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 100, ImGuiCond_Always);
                        
                        char rawPlotNameFormatted[16];
                        sprintf(rawPlotNameFormatted, "Raw CPU %lu Usage", i);
                        ImPlot::PlotScatter(rawPlotNameFormatted, &rawCpuUsageScrollBuffers[i].m_data[0].x, &rawCpuUsageScrollBuffers[i].m_data[0].y, 
                                rawCpuUsageScrollBuffers[i].m_data.size(), 0, rawCpuUsageScrollBuffers[i].m_offset, 2* sizeof(float));
                        
                        char averagedPlotNameFormatted[21];
                        sprintf(averagedPlotNameFormatted, "Averaged CPU %lu Usage", i);
                        ImPlot::PlotLine(averagedPlotNameFormatted, &averagedCpuUsageScrollBuffers[i].m_data[0].x, &averagedCpuUsageScrollBuffers[i].m_data[0].y, 
                                averagedCpuUsageScrollBuffers[i].m_data.size(), 0, averagedCpuUsageScrollBuffers[i].m_offset, 2* sizeof(float));
                        
                        ImPlot::EndPlot();
                    }
                    
                }
                
                ImPlot::PopStyleVar();
                
            }
            
            ImGui::Separator();
            
            {
                
                ImGui::Text("OBC Resources");
                
                std::vector<double> cpuLoad = rawObcCpuUserLoad.get();
                
                uint64_t obcMemUsedInt = obcMemUsed.get();
                
                /*for (int i = 0; i < 1; i++)
                {
                    ImGui::Text("Cpu #%d user load: %f%%", i, cpuLoad[i] * 100);
                    ImGui::Text("Cpu #%d system load: %f%%", i, cpuLoad[i] * 100);
                }
                
                ImGui::Text("Virtual memory used: %d MB / %d MB", (int)(obcMemUsedInt / 1e6), (int) (obcTotalVirtualMem / 1e6));
                */
               
                if (obcMemUsedInt / obcTotalVirtualMem > 0.95)
                {
                    ROS_INFO("OBC virtual memory in use exceeded 95%% (%lu / %lu)! Killing process.", obcMemUsedInt, obcTotalVirtualMem);
                    return 1;
                }
                
                static float t = 0.0f;
                const static float HISTORY = 10.0f;
                t += frameDurationMillis / 1e3f;
                
                // OBC memory plot
                
                static ScrollingBuffer memScrollBuffer;
                
                memScrollBuffer.addPoint(t, obcMemUsedInt / 1e6f);
                
                if (ImPlot::BeginPlot("OBC Virtual Memory", {-1, 150}, memoryPlotFlags))
                {
                    
                    ImPlot::SetupAxes("Time (s)", "Memory", 
                            memoryPlotXAxisFlags, 
                            memoryPlotYAxisFlags);
                    ImPlot::SetupAxisLimits(ImAxis_X1, t - HISTORY * 0.9f, t + HISTORY * 0.1f, ImGuiCond_Always);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, obcTotalVirtualMem / 1e6f, ImGuiCond_Always);
                    
                    ImPlot::PlotLine("OBC Virtual Memory", &memScrollBuffer.m_data[0].x, &memScrollBuffer.m_data[0].y, 
                            memScrollBuffer.m_data.size(), 0, memScrollBuffer.m_offset, 2* sizeof(float));
                    
                    ImPlot::EndPlot();
                }
                
                // OBC Cpu plot
                
                
                {
                    ImVec2 plotSize(-1, 150);
                    
                    ImPlotAxisFlags yAxisFlags = ImPlotAxisFlags_LockMin | ImPlotAxisFlags_LockMax | ImPlotAxisFlags_NoHighlight;
                    ImPlotAxisFlags xAxisFlags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoHighlight;
                    ImPlotFlags plotFlags = ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText | ImPlotFlags_NoMenus;
                    
                    static ScrollingBuffer rawCpu1UsageScrollBuffer;
                    
                    rawCpu1UsageScrollBuffer.addPoint(t, cpuLoad[0] * 100);
                    
                    if (ImPlot::BeginPlot("OBC Cpu Usage", plotSize, plotFlags))
                    {
                        
                        ImPlot::SetupAxes("Time (s)", "Cpu Usage (%)", xAxisFlags, yAxisFlags);
                        ImPlot::SetupAxisLimits(ImAxis_X1, t - HISTORY * 0.9f, t + HISTORY * 0.1f, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 100, ImGuiCond_Always);
                        
                        ImPlot::PlotLine("Raw Cpu Usage (%)", &rawCpu1UsageScrollBuffer.m_data[0].x, &rawCpu1UsageScrollBuffer.m_data[0].y, 
                                rawCpu1UsageScrollBuffer.m_data.size(), 0, rawCpu1UsageScrollBuffer.m_offset, 2* sizeof(float));
                              
                        
                        
                        ImPlot::EndPlot();
                    }
                }
                
            }
            
            ImGui::End();
            
            imGuiEndFrame();
        }
        
        imGuiDestroyWindow();
        
        spinner.stop();
        
        return 0;
    }
    
}


int main(int argc, char** argv)
{
    m_rosuam::gui::run(argc, argv);
}
