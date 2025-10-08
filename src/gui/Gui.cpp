#include "mrosuam/gui/ImGuiWindow.h"
#include "mrosuam/gui/AtomicVariable.h"
#include "mrosuam/gui/ScrollingBuffer.h"
//#include "mrosuam/gui/ConnectionStatus.h"

#include <chrono>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <jpgd.h>


using namespace std::chrono_literals;

namespace mrosuam::gui {
    
    class Gui : public rclcpp::Node
    {
    public:
        Gui() : Node("mrosuam_gui")
        {
            // Initialize topic variables
            
            stateSubscription = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, [this] (mavros_msgs::msg::State::ConstSharedPtr msg) -> void { mavrosState = *msg; });
            rawGcsCpuUserLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/gcs/raw/cpu_user_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { rawGcsCpuUserLoad = msg->data; });
            rawGcsCpuSystemLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/gcs/raw/cpu_system_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { rawGcsCpuSystemLoad = msg->data; });
            averagedGcsCpuUserLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/gcs/averaged/cpu_user_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { averagedGcsCpuUserLoad = msg->data; });
            averagedGcsCpuSystemLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/gcs/averaged/cpu_system_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { averagedGcsCpuSystemLoad = msg->data; });
            gcsMemUsedSubscription = this->create_subscription<std_msgs::msg::UInt64>("/mrosuam/resources/gcs/virtual_mem_used", 10, [this] (std_msgs::msg::UInt64::ConstSharedPtr msg) -> void { gcsMemUsed = msg->data; });
            rawObcCpuUserLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/obc/raw/cpu_user_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { rawObcCpuUserLoad = msg->data; });
            rawObcCpuSystemLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/obc/raw/cpu_system_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { rawObcCpuSystemLoad = msg->data; });
            averagedObcCpuUserLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/obc/averaged/cpu_user_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { averagedObcCpuUserLoad = msg->data; });
            averagedObcCpuSystemLoadSubscription = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mrosuam/resources/obc/averaged/cpu_system_load", 10, [this] (std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) -> void { averagedObcCpuSystemLoad = msg->data; });
            obcMemUsedSubscription = this->create_subscription<std_msgs::msg::UInt64>("/mrosuam/resources/obc/virtual_mem_used", 10, [this] (std_msgs::msg::UInt64::ConstSharedPtr msg) -> void { obcMemUsed = msg->data; });
            cameraImageSubscription = this->create_subscription<sensor_msgs::msg::CompressedImage>("/raspicam_node/image/compressed", 10, [this] (sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) -> void { cameraImage = *msg; });
            
            //obcConnectionStatus.initialize(nodeHandle);
            //cameraConnectionStatus.initialize(nodeHandle);
            //fcuConnectionStatus.initialize(nodeHandle);
            
            /*
            
            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setModeClient = guiNode.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
            
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr localPosePub = guiNode.create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            
            
            for (int i = 0; i < 100; i++)
            {
                localPosePub->publish(pose);
                rclcpp::sleep_for(100ms);
            }
            
            
            mavros_msgs::srv::SetMode offboardSetMode;
            offboardSetMode.request.custom_mode = "OFFBOARD";
            
            while (mavrosState.get().mode != "OFFBOARD")
            {
                RCLCPP_INFO(guiNode.get_logger(), "Tried setting offboard mode - %s, %s response", 
                        setModeClient.call(offboardSetMode) ? "sent" : "not sent", 
                        offboardSetMode.response.mode_sent ? "with" : "without");
                
                rclcpp::sleep_for(5ms);
            }
            */
            
            
            RCLCPP_INFO(this->get_logger(), "Starting GUI");
            
            if (window.setupWindow() > 0)
                return;
            
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
                return;
            }
            
            // Determine total amount of memory on GCS and OBC
            
            struct sysinfo gcsMemInfo;
            sysinfo(&gcsMemInfo);
            
            gcsTotalVirtualMem = gcsMemInfo.totalram;
            gcsTotalVirtualMem += gcsMemInfo.totalswap;
            gcsTotalVirtualMem += gcsMemInfo.mem_unit;
            
            struct sysinfo obcMemInfo;
            sysinfo(&obcMemInfo);
            
            obcTotalVirtualMem = obcMemInfo.totalram;
            obcTotalVirtualMem += obcMemInfo.totalswap;
            obcTotalVirtualMem += obcMemInfo.mem_unit;
            
            const GLubyte* version = glGetString(GL_VERSION);
            std::cout << "OpenGL version: " << version << std::endl;
            
            timer = this->create_wall_timer(16ms, std::bind(&Gui::gui_iteration, this));
            
            
        }
        
        private:
        
        rclcpp::TimerBase::SharedPtr timer;
        
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rawGcsCpuUserLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rawGcsCpuSystemLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr averagedGcsCpuUserLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr averagedGcsCpuSystemLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr gcsMemUsedSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rawObcCpuUserLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rawObcCpuSystemLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr averagedObcCpuUserLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr averagedObcCpuSystemLoadSubscription;
        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr obcMemUsedSubscription;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr cameraImageSubscription;
        
        AtomicVariable<mavros_msgs::msg::State> mavrosState;
        AtomicVariable<std::vector<double>> rawGcsCpuUserLoad{std::vector<double>(4, 0.0f)};
        AtomicVariable<std::vector<double>> rawGcsCpuSystemLoad{std::vector<double>(4, 0.0f)};
        AtomicVariable<std::vector<double>> averagedGcsCpuUserLoad{std::vector<double>(4, 0.0f)};
        AtomicVariable<std::vector<double>> averagedGcsCpuSystemLoad{std::vector<double>(4, 0.0f)};
        AtomicVariable<uint64_t> gcsMemUsed;
        AtomicVariable<std::vector<double>> rawObcCpuUserLoad{std::vector<double>(1, 0.0f)};
        AtomicVariable<std::vector<double>> rawObcCpuSystemLoad{std::vector<double>(1, 0.0f)};
        AtomicVariable<std::vector<double>> averagedObcCpuUserLoad{std::vector<double>(1, 0.0f)};
        AtomicVariable<std::vector<double>> averagedObcCpuSystemLoad{std::vector<double>(1, 0.0f)};
        AtomicVariable<uint64_t> obcMemUsed;
        AtomicVariable<sensor_msgs::msg::CompressedImage> cameraImage;
        
        uint64_t gcsTotalVirtualMem;
        uint64_t obcTotalVirtualMem;
        GLuint renderedTexture;
        
    
        ImGuiWindow window;
        
        /*
        ROSTopicVariable<mavros_msgs::msg::State, mavros_msgs::msg::State> currentState([] (mavros_msgs::msg::State::SharedPtr state) -> mavros_msgs::msg::State { return *state; });
        
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> rawGcsCpuUserLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> rawGcsCpuSystemLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> averagedGcsCpuUserLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> averagedGcsCpuSystemLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(4));
        ROSTopicVariable<uint64_t, std_msgs::msg::UInt64> gcsMemUsed([] (std_msgs::msg::UInt64::ConstPtr msg) -> uint64_t { return msg->data; });
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> rawObcCpuUserLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> rawObcCpuSystemLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> averagedObcCpuUserLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
        ROSTopicVariable<std::vector<double>, std_msgs::msg::Float64MultiArray> averagedObcCpuSystemLoad([] (std_msgs::msg::Float64MultiArray::ConstPtr msg) -> std::vector<double> { return msg->data; }, std::vector<double>(1));
        ROSTopicVariable<uint64_t, std_msgs::msg::UInt64> obcMemUsed([] (std_msgs::msg::UInt64::ConstPtr msg) -> uint64_t { return msg->data; });
        ROSTopicVariable<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage> cameraImage([] (sensor_msgs::msg::CompressedImage::ConstPtr msg) -> sensor_msgs::msg::CompressedImage { return *msg; });
        */
        bool obcConnected, cameraConnected, fcuConnected;
        
        //ConnectionStatus<std_msgs::msg::UInt64> obcConnectionStatus("/mrosuam/resources/obc/virtual_mem_used", 1.0f);
        //ConnectionStatus<sensor_msgs::msg::CompressedImage> cameraConnectionStatus("/raspicam_node/image/compressed", 1.0f);
        //ConnectionStatus<mavros_msgs::msg::State> fcuConnectionStatus("/mavros/state", 1.0f);
        
        void gui_iteration()
        {
            
                
            float frameDurationMillis;
            
            {
                using namespace std::chrono;
                auto currentFrameTime = steady_clock::now();
                static auto lastFrameTime = currentFrameTime;
                frameDurationMillis = duration_cast<microseconds>(currentFrameTime - lastFrameTime).count() / 1.0e3f;
                lastFrameTime = currentFrameTime;
            }
            
            window.beginFrame();
            
            //obcConnected = obcConnectionStatus.checkStatus();
            //cameraConnected = cameraConnectionStatus.checkStatus();
            //fcuConnected = fcuConnectionStatus.checkStatus();
            
            //ImGui::ShowDemoWindow();
            //ImPlot::ShowDemoWindow();
            
            // Camera/CV window
            
            if (ImGui::Begin("Camera"))
            {
                
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
                
                //ImGui::Text("Error: %d", glGetError());
                
                ImGui::Text("OBC connected: %d", obcConnected);
                ImGui::Text("Camera connected: %d", cameraConnected);
                ImGui::Text("FCU connected: %d", fcuConnected);
                
                ImGui::End();
                
            }
            
            
            
            // Stats Window
            
            if (ImGui::Begin("Stats")) {
                
                float statsWindowWidth = ImGui::GetContentRegionAvail().x;
                
                int fps = 1000 / frameDurationMillis;
                ImGui::Text("Current frame time: %f ms (%d fps)", frameDurationMillis, fps);
                
                ImGui::Separator();
                
                ImPlotFlags memoryPlotFlags = ImPlotFlags_NoLegend | ImPlotFlags_NoFrame;
                ImPlotAxisFlags memoryPlotXAxisFlags = ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickLabels;
                ImPlotAxisFlags memoryPlotYAxisFlags = ImPlotAxisFlags_LockMin | ImPlotAxisFlags_LockMax;
                
                ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(1, 1));
                
                ImPlotTextFlags textFlags = ImPlotTextFlags_None;
                
                //ImPlot::PushStyleVar(ImPlotStyleVar_)
                
                {
                    
                    ImGui::Text("GCS Resources");
                    
                    std::vector<double> rawCpuUserLoad = rawGcsCpuUserLoad.get();
                    std::vector<double> averagedCpuUserLoad = averagedGcsCpuUserLoad.get();
                    
                    uint64_t gcsMemUsedInt = gcsMemUsed.get();
                    
                    if (gcsMemUsedInt / gcsTotalVirtualMem > 0.95)
                    {
                        RCLCPP_INFO(this->get_logger(), "GCS virtual memory in use exceeded 95%% (%lu / %lu)! Killing process.", gcsMemUsedInt, gcsTotalVirtualMem);
                        return;
                    }
                    
                    static float t = 0.0f;
                    const static float HISTORY = 10.0f;
                    t += frameDurationMillis / 1e3f;
                    
                    ImPlot::PushStyleColor(ImPlotCol_PlotBorder, {0, 128, 255, 1});
                    
                    // GCS memory plot
                    
                    static ScrollingBuffer memScrollBuffer;
                    
                    memScrollBuffer.addPoint(t, gcsMemUsedInt / 1e6f);
                    
                    if (ImPlot::BeginPlot("GCS Virtual Memory", {statsWindowWidth, 120}, memoryPlotFlags))
                    {
                        
                        ImPlot::SetupAxes("Time (s)", "Memory", 
                                memoryPlotXAxisFlags, 
                                memoryPlotYAxisFlags);
                        ImPlot::SetupAxisLimits(ImAxis_X1, t - HISTORY * 0.9f, t + HISTORY * 0.1f, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, gcsTotalVirtualMem / 1e6f, ImGuiCond_Always);
                        
                        ImPlot::PlotLine("Cpu Usage", &memScrollBuffer.m_data[0].x, &memScrollBuffer.m_data[0].y, 
                                memScrollBuffer.m_data.size(), 0, memScrollBuffer.m_offset, 2* sizeof(float));
                        
                        std::string text = std::to_string(gcsMemUsedInt / (int)1e6) + "/6000";
                        ImPlot::PlotText(text.c_str(), t - HISTORY * 0.7f, 5400, {0, 0}, textFlags);
                        
                        ImPlot::EndPlot();
                    }
                    
                    ImPlot::PopStyleColor();
                    
                    // GCS CPU plots
                    
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
                            
                            std::string text = std::to_string((int)(averagedCpuUserLoad[i] * 100)) + "%";
                            ImPlot::PlotText(text.c_str(), t - HISTORY * 0.8f, 90, {0, 0}, textFlags);
                            
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
                
                    if (obcMemUsedInt / obcTotalVirtualMem > 0.95)
                    {
                        RCLCPP_INFO(this->get_logger(), "OBC virtual memory in use exceeded 95%% (%lu / %lu)! Killing process.", obcMemUsedInt, obcTotalVirtualMem);
                        return;
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
                        
                        std::string text = std::to_string(obcMemUsedInt / (int)1e6) + "/500";
                        ImPlot::PlotText(text.c_str(), t - HISTORY * 0.7f, 450, {0, 0}, textFlags);
                        
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
                            
                            std::string text = std::to_string((int)(averagedObcCpuUserLoadVal[0] * 100)) + "%";
                            ImPlot::PlotText(text.c_str(), t - HISTORY * 0.8f, 90, {0, 0}, textFlags);
                            
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
                
                //ImGui::Text("Connected: %d", currentState.get().connected);
                
                ImGui::End();
            }
            
            window.endFrame();
            
            if (window.shouldClose())
            {
                RCLCPP_INFO(this->get_logger(), "Window requested close, stopping node.");
                timer->cancel();
                
                window.destroyWindow();
                rclcpp::shutdown();
            }
            
            //window.destroyWindow();
            
            //return;
        }
    };
    
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<mrosuam::gui::Gui>());
    
    rclcpp::shutdown();
}
