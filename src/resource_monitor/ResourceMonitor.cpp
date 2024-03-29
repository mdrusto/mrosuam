
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <chrono>
#include <deque>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt64.h>

namespace mrosuam::ResourceMonitor
{
	
	int numCPUs;
	
	const static int CPU_FILTER_LENGTH = 20;
	
	int createNode(int argc, char** argv)
	{
		ros::init(argc, argv, "resource_monitor");
		ros::NodeHandle nodeHandle;
		
		const int MAX_CHARS = 20;
		char hostname_char[MAX_CHARS];
		ROS_ASSERT(gethostname(hostname_char, MAX_CHARS - 1) == 0);
		std::string hostname(hostname_char);
		
		std::string machineID;
		if (hostname == "baela")
			machineID = "gcs";
		else
			machineID = "obc";
		
		ros::Publisher rawCpuUserLoadPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>(
				"/mrosuam/resources/" + machineID + "/raw/cpu_user_load", 100);
		ros::Publisher rawCpuSystemLoadPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>(
				"/mrosuam/resources/" + machineID + "/raw/cpu_system_load", 100);
		ros::Publisher averagedCpuUserLoadPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>(
				"/mrosuam/resources/" + machineID + "/averaged/cpu_user_load", 100);
		ros::Publisher averagedCpuSystemLoadPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>(
				"/mrosuam/resources/" + machineID + "/averaged/cpu_system_load", 100);
		
		ros::Publisher virtualMemUsedPublisher = nodeHandle.advertise<std_msgs::UInt64>("/mrosuam/resources/" + machineID + "/virtual_mem_used", 100);
		
		//ros::MultiThreadedSpinner spinner;
		ros::SingleThreadedSpinner spinner;
		ros::Rate loopRate(20);
		
		ROS_INFO("Started resource monitor on %s", hostname.c_str());
		
		{
			std::ifstream initStatStream("/proc/stat", std::ios_base::in);
			
			std::string unused;
			// Initial line is total for all CPUs
			std::getline(initStatStream, unused);
			
			while (std::getline(initStatStream, unused))
			{
				if (unused.substr(0, 3) != "cpu")
					break;
				numCPUs++;
			}
			
			ROS_INFO("Inited resource monitor: %d CPUs", numCPUs);
		}
		
		while (ros::ok())
		{
			
			float frameDurationMillis;
            
            {
                using namespace std::chrono;
                auto currentFrameTime = steady_clock::now();
                static auto lastFrameTime = currentFrameTime;
                frameDurationMillis = duration_cast<microseconds>(currentFrameTime - lastFrameTime).count() / 1.0e3f;
                lastFrameTime = currentFrameTime;
            }
			
			std::ifstream statStream("/proc/stat", std::ios_base::in);
			std::string unused;
			std::getline(statStream, unused);
			
			static std::vector<int> lastUserJiffies(numCPUs), lastSystemJiffies(numCPUs), lastTotalJiffies(numCPUs);
			
			std::vector<double> rawCpuUserPercentage(numCPUs), rawCpuSystemPercentage(numCPUs);
			
			static std::deque<std::vector<double>> cpuUserLoadDeque, cpuSystemLoadDeque;
			
			for (int i = 0; i < numCPUs; i++)
			{
				std::string lineText;
				
				// First line is total for all CPUs
				std::getline(statStream, lineText);
				
				// Split line into individual "words"
				std::istringstream iss(lineText);
				std::string word;
				// Ignore first word
				std::getline(iss, word, ' ');
				int j = 0;
				int totalJiffies = 0, userJiffies, systemJiffies;
				while(std::getline(iss, word, ' '))
				{
					if (word.empty())
						continue;
					
					int jiffies = std::stoi(word);
					totalJiffies += jiffies;
					
					switch (j)
					{
						case 0:
							userJiffies = jiffies;
						case 2:
							systemJiffies = jiffies;
					}
					
					j++;
				}
				
				rawCpuUserPercentage[i] = (userJiffies - lastUserJiffies[i]) / (double)(totalJiffies - lastTotalJiffies[i]);
				rawCpuSystemPercentage[i] = (systemJiffies - lastSystemJiffies[i]) / (double)(totalJiffies - lastTotalJiffies[i]);
				
				//ROS_INFO("User: %d (last %d), system: %d (last %d), total: %d (last %d) - %f%%, %f%%", 
				//		userJiffies, lastUserJiffies[i], systemJiffies, lastSystemJiffies[i], totalJiffies, lastTotalJiffies[i], userPercentage[i] * 100, systemPercentage[i] * 100);
				
				lastUserJiffies[i] = userJiffies;
				lastSystemJiffies[i] = systemJiffies;
				lastTotalJiffies[i] = totalJiffies;
				
			}
			
			// Average CPU values
			
			cpuUserLoadDeque.push_back(rawCpuUserPercentage);
			
			cpuSystemLoadDeque.push_back(rawCpuSystemPercentage);
			
			if (cpuUserLoadDeque.size() == CPU_FILTER_LENGTH) {
				cpuUserLoadDeque.pop_front();
				cpuSystemLoadDeque.pop_front();
			}
			
			std::vector<double> averagedCpuUserLoad(numCPUs), averagedCpuSystemLoad(numCPUs);
			
			// First loop over all CPUs
			
			uint8_t dequeSize = cpuUserLoadDeque.size(); 
			
			for (size_t i = 0; i < numCPUs; i++) {
				
				// For each CPU, sum total of load percentages
				
				double userTotal = 0, systemTotal = 0;
				
				for (size_t j = 0; j < dequeSize; j++) {
					userTotal += cpuUserLoadDeque[j][i];
					systemTotal += cpuSystemLoadDeque[j][i];
				}
				
				averagedCpuUserLoad[i] = userTotal / dequeSize;
				averagedCpuSystemLoad[i] = systemTotal / dequeSize;
			}
			
			
			// Publish messages
			
			std_msgs::Float64MultiArray rawCpuUserLoadMsg;
			rawCpuUserLoadMsg.data = rawCpuUserPercentage;
			
			std_msgs::Float64MultiArray rawCpuSystemLoadMsg;
			rawCpuSystemLoadMsg.data = rawCpuSystemPercentage;
			
			rawCpuUserLoadPublisher.publish<std_msgs::Float64MultiArray>(rawCpuUserLoadMsg);
			rawCpuSystemLoadPublisher.publish<std_msgs::Float64MultiArray>(rawCpuSystemLoadMsg);
			
			std_msgs::Float64MultiArray filteredCpuUserLoadMsg;
			filteredCpuUserLoadMsg.data = averagedCpuUserLoad;
			
			std_msgs::Float64MultiArray filteredCpuSystemLoadMsg;
			filteredCpuSystemLoadMsg.data = averagedCpuSystemLoad;
			
			averagedCpuUserLoadPublisher.publish<std_msgs::Float64MultiArray>(filteredCpuUserLoadMsg);
			averagedCpuSystemLoadPublisher.publish<std_msgs::Float64MultiArray>(filteredCpuSystemLoadMsg);
			
			
			struct sysinfo memInfo;
			sysinfo(&memInfo);
			
			uint64_t totalVirtualMem = memInfo.totalram;
			totalVirtualMem += memInfo.totalswap;
			totalVirtualMem += memInfo.mem_unit;
			
			uint64_t virtualMemUsed = memInfo.totalram - memInfo.freeram;
			virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
			virtualMemUsed += memInfo.mem_unit;
			
			std_msgs::UInt64 virtualMemUsedMsg;
			virtualMemUsedMsg.data = virtualMemUsed;
			
			virtualMemUsedPublisher.publish<std_msgs::UInt64>(virtualMemUsedMsg);
			
			
			//ROS_INFO("Virtual memory used: %fMB / %dMB", (float)(virtualMemUsed / 1e6), (int)(totalVirtualMem / 1e6));
			
			//ROS_INFO("Resource monitor iteration time: %f ms", frameDurationMillis);
			
			loopRate.sleep();
		}
		
		return 0;
		/*
		double vmUsage, residentSet;
		
		std::string pid, comm, state, ppid, pgrp, session, tty_nr;
		std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
		std::string utime, stime, cutime, cstime, priority, nice;
		std::string o, itrealvalue, starttime;
		
		unsigned long vsize;
		long rss;
		
		statStream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
			>> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
			>> utime >> stime >> cutime >> cstime >> priority >> nice
			>> o >> itrealvalue >> starttime >> vsize >> rss;
		
		statStream.close();
		
		long pageSizeKb = sysconf(_SC_PAGE_SIZE) / 1024;
		vmUsage = vsize / 1024.0f;
		residentSet = rss * pageSizeKb;
		*/
	}
	
}

int main(int argc, char** argv)
{
	return mrosuam::ResourceMonitor::createNode(argc, argv);
}
