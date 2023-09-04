#include "mrosuam/gui/ROSTopicVariable.h"

#include <chrono>
#include <mutex>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include <ros/ros.h>

#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>

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

namespace mrosuam::gui {
    
    
    ROSTopicVariable<mavros_msgs::State, mavros_msgs::State> currentState([] (mavros_msgs::State::ConstPtr state) -> mavros_msgs::State { return *state; });
    
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> rawGcsCpuUserLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> rawGcsCpuSystemLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> averagedGcsCpuUserLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> averagedGcsCpuSystemLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
    ROSTopicVariable<uint64_t, std_msgs::UInt64> gcsMemUsed([] (std_msgs::UInt64::ConstPtr msg) -> uint64_t { return msg->data; });
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> rawObcCpuUserLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> rawObcCpuSystemLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> averagedObcCpuUserLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
    ROSTopicVariable<std::vector<double>, std_msgs::Float64MultiArray> averagedObcCpuSystemLoad([] (std_msgs::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
    ROSTopicVariable<uint64_t, std_msgs::UInt64> obcMemUsed([] (std_msgs::UInt64::ConstPtr msg) -> uint64_t { return msg->data; });
    ROSTopicVariable<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> cameraImage([] (sensor_msgs::CompressedImage::ConstPtr msg) -> sensor_msgs::CompressedImage { return *msg; });
    
    AtomicVariable<geometry_msgs::PoseStamped> uavPose;
    
    std::mutex imageMutex;
    //sensor_msgs::CompressedImage currentImage;
    //int numTimesSet = 0;

    const ImVec4 CLEAR_COLOUR(0.45f, 0.55f, 0.60f, 1.00f);

    static GLFWwindow* s_window = nullptr;

    static void glfw_error_callback(int error, const char* description)
    {
        fprintf(stderr, "Glfw Error %d: %s\n", error, description);
    }
    
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
    {
        uavPose = *pose;
    }

    bool imGuiSetupWindow()
    {
        glfwSetErrorCallback(glfw_error_callback);
        
        std::cout << "Setting up window" << std::endl;
        
        if (glfwInit() == GLFW_FALSE)
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
        ros::init(argc, argv, "gui");
        
        ROS_INFO("Starting GUI");
        
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
        
        ros::NodeHandle nodeHandle;
        
        currentState.initialize(nodeHandle, "/mavros/state", 10);
        
        rawGcsCpuUserLoad.initialize(nodeHandle, "/mrosuam/resources/gcs/raw/cpu_user_load", 10);
        rawGcsCpuSystemLoad.initialize(nodeHandle, "/mrosuam/resources/gcs/raw/cpu_system_load", 10);
        averagedGcsCpuUserLoad.initialize(nodeHandle, "/mrosuam/resources/gcs/averaged/cpu_user_load", 10);
        averagedGcsCpuSystemLoad.initialize(nodeHandle, "/mrosuam/resources/gcs/averaged/cpu_system_load", 10);
        gcsMemUsed.initialize(nodeHandle, "/mrosuam/resources/gcs/virtual_mem_used", 10);
        rawObcCpuUserLoad.initialize(nodeHandle, "/mrosuam/resources/obc/raw/cpu_user_load", 10);
        rawObcCpuSystemLoad.initialize(nodeHandle, "/mrosuam/resources/obc/raw/cpu_system_load", 10);
        averagedObcCpuUserLoad.initialize(nodeHandle, "/mrosuam/resources/obc/averaged/cpu_user_load", 10);
        averagedObcCpuSystemLoad.initialize(nodeHandle, "/mrosuam/resources/obc/averaged/cpu_system_load", 10);
        obcMemUsed.initialize(nodeHandle, "/mrosuam/resources/obc/virtual_mem_used", 10);
        cameraImage.initialize(nodeHandle, "/raspicam_node/image/compressed", 10);
        
        
        ros::Subscriber uavPoseSubscriber = nodeHandle.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, uavPoseCallback);
        
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
            
            // Camera/CV window
            
            if (ImGui::Begin("Camera"))
            {
                
                imageMutex.lock();
                std::vector<uint8_t> imageDataVec = cameraImage.get().data;
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
                
                ImGui::Text("Error: %d", glGetError());
                
                ImGui::End();
                
            }
            
            
            
            // Stats Window
            
            if (ImGui::Begin("Stats")) {
                
                float statsWindowWidth = ImGui::GetContentRegionAvailWidth();
                
                int fps = 1000 / frameDurationMillis;
                ImGui::Text("Current frame time: %f ms (%d fps)", frameDurationMillis, fps);
                
                ImGui::Separator();
                
                ImPlotFlags memoryPlotFlags = ImPlotFlags_NoLegend | ImPlotFlags_NoFrame;
                ImPlotAxisFlags memoryPlotXAxisFlags = ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickLabels;
                ImPlotAxisFlags memoryPlotYAxisFlags = ImPlotAxisFlags_LockMin | ImPlotAxisFlags_LockMax;
                
                ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(1, 1));
                
                {
                    
                    ImGui::Text("GCS Resources");
                    
                    std::vector<double> rawCpuUserLoad = rawGcsCpuUserLoad.get();
                    std::vector<double> averagedCpuUserLoad = averagedGcsCpuUserLoad.get();
                    
                    uint64_t gcsMemUsedInt = gcsMemUsed.get();
                    
                    if (gcsMemUsedInt / gcsTotalVirtualMem > 0.95)
                    {
                        ROS_INFO("GCS virtual memory in use exceeded 95%% (%lu / %lu)! Killing process.", gcsMemUsedInt, gcsTotalVirtualMem);
                        return 1;
                    }
                    
                    static float t = 0.0f;
                    const static float HISTORY = 10.0f;
                    t += frameDurationMillis / 1e3f;
                    
                    ImPlot::PushStyleColor(ImPlotCol_PlotBorder, {0, 128, 255, 1});
                    
                    // Laptop memory plot
                    
                    static ScrollingBuffer memScrollBuffer;
                    
                    memScrollBuffer.addPoint(t, gcsMemUsedInt / 1e6f);
                    
                    if (ImPlot::BeginPlot("GCS Virtual Memory", {statsWindowWidth, 140}, memoryPlotFlags))
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
                    
                    ImPlot::PopStyleColor();
                    
                    // Laptop Cpu plots
                    
                    ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 1);
                    
                    ImPlot::PushStyleColor(ImPlotCol_PlotBorder, {255, 128, 0, 1});
                    
                    static std::vector<ScrollingBuffer> rawCpuUsageScrollBuffers(4), averagedCpuUsageScrollBuffers(4);
                    
                    for (size_t i = 0; i < 4; i++) {
                        
                        if (i == 1 || i == 3)
                            ImGui::SameLine();
                        
                        ImVec2 plotSize(statsWindowWidth / 2, 140);
                        
                        ImPlotAxisFlags yAxisFlags = ImPlotAxisFlags_LockMin | ImPlotAxisFlags_LockMax | ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoHighlight;
                        ImPlotAxisFlags xAxisFlags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoHighlight;
                        ImPlotFlags plotFlags = ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText | ImPlotFlags_NoMenus | ImPlotFlags_NoFrame;
                        
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
                    
                    ImPlot::PopStyleColor();
                    
                    ImPlot::PopStyleVar();
                    
                }
                
                ImGui::Separator();
                
                {
                    
                    ImGui::Text("OBC Resources");
                    
                    std::vector<double> rawObcCpuUserLoadVal = rawObcCpuUserLoad.get();
                    std::vector<double> averagedObcCpuUserLoadVal = averagedObcCpuUserLoad.get();
                    
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
                    
                    ImPlot::PushStyleColor(ImPlotCol_PlotBorder, {0, 128, 255, 1});
                    
                    static ScrollingBuffer memScrollBuffer;
                    
                    memScrollBuffer.addPoint(t, obcMemUsedInt / 1e6f);
                    
                    if (ImPlot::BeginPlot("OBC Virtual Memory", {statsWindowWidth, 150}, memoryPlotFlags))
                    {
                        
                        ImPlot::SetupAxes("Time (s)", "Memory", 
                                memoryPlotXAxisFlags, 
                                memoryPlotYAxisFlags);
                        ImPlot::SetupAxisLimits(ImAxis_X1, t - HISTORY * 0.9f, t + HISTORY * 0.1f, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 500, ImGuiCond_Always);
                        
                        ImPlot::PlotLine("OBC Virtual Memory", &memScrollBuffer.m_data[0].x, &memScrollBuffer.m_data[0].y, 
                                memScrollBuffer.m_data.size(), 0, memScrollBuffer.m_offset, 2* sizeof(float));
                        
                        ImPlot::EndPlot();
                    }
                    
                    ImPlot::PopStyleColor();
                    
                    // OBC Cpu plot
                    
                    
                    {
                        ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 1);
                    
                        ImPlot::PushStyleColor(ImPlotCol_PlotBorder, {255, 128, 0, 1});
                        
                        ImVec2 plotSize(statsWindowWidth, 150);
                        
                        ImPlotAxisFlags yAxisFlags = ImPlotAxisFlags_LockMin | ImPlotAxisFlags_LockMax | ImPlotAxisFlags_NoHighlight;
                        ImPlotAxisFlags xAxisFlags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoHighlight;
                        ImPlotFlags plotFlags = ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText | ImPlotFlags_NoMenus | ImPlotFlags_NoFrame;
                        
                        static ScrollingBuffer rawCpuUsageScrollBuffer, averagedCpuUsageScrollBuffer;
                        
                        rawCpuUsageScrollBuffer.addPoint(t, rawObcCpuUserLoadVal[0] * 100);
                        averagedCpuUsageScrollBuffer.addPoint(t, averagedObcCpuUserLoadVal[0] * 100);
                        
                        if (ImPlot::BeginPlot("OBC Cpu Usage", plotSize, plotFlags))
                        {
                            
                            ImPlot::SetupAxes("Time (s)", "Cpu Usage (%)", xAxisFlags, yAxisFlags);
                            ImPlot::SetupAxisLimits(ImAxis_X1, t - HISTORY * 0.9f, t + HISTORY * 0.1f, ImGuiCond_Always);
                            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 100, ImGuiCond_Always);
                            
                            ImPlot::PlotScatter("Raw OBC CPU Usage", &rawCpuUsageScrollBuffer.m_data[0].x, &rawCpuUsageScrollBuffer.m_data[0].y, 
                                    rawCpuUsageScrollBuffer.m_data.size(), 0, rawCpuUsageScrollBuffer.m_offset, 2 * sizeof(float));
                                
                            ImPlot::PlotLine("Averaged OBC CPU Usage", &averagedCpuUsageScrollBuffer.m_data[0].x, &averagedCpuUsageScrollBuffer.m_data[0].y, 
                                    averagedCpuUsageScrollBuffer.m_data.size(), 0, averagedCpuUsageScrollBuffer.m_offset, 2 * sizeof(float));
                            
                            ImPlot::EndPlot();
                        }
                        
                        ImPlot::PopStyleVar();
                    }
                    
                }
                
                ImPlot::PopStyleColor();
                
                ImPlot::PopStyleVar();
                
                ImGui::End();
                
            }
            
            // MAVROS Window
            
            if (ImGui::Begin("MAVROS")) {
                
                ImGui::Text("Connected: %d", currentState.get().connected);
                
                geometry_msgs::PoseStamped pose = uavPose.get();
                
                ImGui::Text("Rotation: x %lf, y %lf, z %lf", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
                
                ImGui::End();
            }
            
            
            
            imGuiEndFrame();
        }
        
        imGuiDestroyWindow();
        
        spinner.stop();
        
        return 0;
    }
    
}


int main(int argc, char** argv)
{
    mrosuam::gui::run(argc, argv);
}
